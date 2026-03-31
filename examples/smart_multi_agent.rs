/// SMART-3D Multi-Agent — Shared Map Discovery + Mutual Avoidance
///
/// Multiple agents explore an unknown map simultaneously. Each agent has its own
/// SMART RRT* tree, but discovered walls are shared globally: when Agent A's LiDAR
/// discovers a wall, ALL agents' trees are updated via update_static_obstacles().
///
/// Agents treat each other as dynamic obstacles (SphericalObstacleWithOhz) and
/// use SMART's reactive replanning to avoid collisions.
///
/// Usage:
///   cargo run --release --example smart_multi_agent -- \
///     --scenario mega_airport --num-agents 4 --agent-mode cross \
///     --playback-speed 5 --include-tree \
///     | python3 ~/smart_rviz_bridge/smart_rviz_bridge.py
///
///   Modes:
///     cross    — agents start at corners, go to opposite corners (path crossing)
///     converge — agents start at corners, all go to center (deadlock stress test)
///     scatter  — agents start at center, go to corners (expansion test)

use clap::Parser;
use json::{object, JsonValue};
use rayon::prelude::*;
use smart_3d::obstacles::random_dynamic::RandomDirectionCircle;
use smart_3d::obstacles::SphericalObstacle;
use smart_3d::obstacles::{
    AnalyticValidityChecker, SpatialRectValidityChecker, DynamicSphericalObstacle, RectangularObstacle,
    StaticRectangularObstacle, StaticSphericalObstacle,
};
use smart_3d::rrt::KdTreeNearestNeighbors;
use smart_3d::rrt::{
    rrt_star::optimal_gamma,
    termination::{MaxIterationsTermination, MaxTimeTermination},
    validity_checker::AlwaysValid,
    RRTstar, RealVectorState, SamplingDistribution, UniformDistribution, ValidityChecker,
};
use smart_3d::smart::{SmartUpdateResult, SphericalObstacleWithOhz, SMART};
use rand::Rng;
use std::io::{self, Write};
use std::panic;
use std::time::{Duration, Instant};

// ==================== Goal-biased sampling ====================
struct GoalBiasedDistribution {
    uniform: UniformDistribution<f32, 2>,
    bias_target: RealVectorState<f32, 2>,
    goal_bias: f64,
}
impl GoalBiasedDistribution {
    fn new(ranges: [(f32, f32); 2], bias_target: RealVectorState<f32, 2>, goal_bias: f64) -> Self {
        Self { uniform: UniformDistribution::new(ranges), bias_target, goal_bias: goal_bias.clamp(0.0, 0.5) }
    }
}
impl SamplingDistribution<f32, 2> for GoalBiasedDistribution {
    fn sample(&mut self) -> RealVectorState<f32, 2> {
        let mut rng = rand::thread_rng();
        if rng.gen::<f64>() < self.goal_bias {
            let mut vals = self.bias_target.values().clone();
            for v in vals.iter_mut() { *v += (rng.gen::<f64>() * 1.0 - 0.5) as f32; }
            RealVectorState::new(vals)
        } else { self.uniform.sample() }
    }
}

// ==================== CLI ====================
#[derive(Parser, Debug)]
#[command(version, about = "SMART-3D Multi-Agent — Shared Map + Mutual Avoidance")]
struct CliArgs {
    #[arg(short, long, default_value = "mega_airport")]
    scenario: String,

    /// Number of agents (2..8)
    #[arg(long, default_value = "4")]
    num_agents: usize,

    /// Agent placement mode: cross, converge, scatter
    #[arg(long, default_value = "cross")]
    agent_mode: String,

    /// Robot speed (m/s)
    #[arg(long, default_value = "3.0")]
    robot_speed: f32,

    #[arg(long, default_value = "0.05")]
    dt: f32,

    #[arg(long, default_value = "3.0")]
    playback_speed: f32,

    #[arg(long)]
    include_tree: bool,

    #[arg(long, default_value = "10.0")]
    sensor_range: f32,

    #[arg(long, default_value = "0.5")]
    robot_radius: f32,

    #[arg(long, default_value = "5000")]
    rrt_iterations: usize,

    #[arg(long, default_value = "0.0")]
    goal_bias: f64,

    #[arg(long, default_value = "150")]
    shortcut_iters: usize,

    #[arg(long, default_value = "1.5")]
    detour_threshold: f32,

    #[arg(long, default_value = "1.0")]
    lrz_time: f32,

    #[arg(long, default_value = "30.0")]
    lsr_max: f32,

    /// Number of dynamic people obstacles per agent (0 to disable)
    #[arg(short, long, default_value = "0")]
    num_obstacles: usize,

    #[arg(short, long, default_value = "1.5")]
    obstacle_speed: f32,

    /// Max simulation time before timeout (seconds)
    #[arg(long, default_value = "600.0")]
    max_time: f32,

    /// Agent priorities (comma-separated, e.g. "0,0,0,0" = all equal, "1,0,0,0" = A0 highest)
    /// Higher number = higher priority. Same priority = random tiebreak per run.
    #[arg(long, default_value = "")]
    agent_priorities: String,

    /// Senior baseline: path-only repair (original SMART author's approach).
    /// Only checks edges ON the current path, not full tree scan.
    /// For benchmark comparison against our full-tree method.
    #[arg(long)]
    senior_mode: bool,

    /// Disable wall sharing between agents. Each agent only knows walls
    /// discovered by its own LiDAR. Agents still avoid each other physically.
    /// Used for ablation: measure the value of shared wall discovery.
    #[arg(long)]
    no_wall_sharing: bool,
}

// ==================== Agent ====================
struct Agent {
    name: String,
    color: [f32; 3],
    priority: u32,      // higher = go first. 0 = default
    tiebreak: f32,      // random, for same-priority encounters (stable per run)
    smart: SMART<f32, 2, KdTreeNearestNeighbors<f32, 2>>,
    current_path: Vec<RealVectorState<f32, 2>>,
    path_idx: usize,
    robot: RealVectorState<f32, 2>,
    goal: RealVectorState<f32, 2>,
    reached_goal: bool,
    rebuild_count: u32,
    frames_since_progress: u64,
    last_progress_pos: RealVectorState<f32, 2>,
    // Per-agent wall tracking (used only in --no-wall-sharing mode)
    own_known_walls: Vec<StaticRectangularObstacle<f32, 2>>,
    own_walls_discovered: Vec<bool>,
    own_cached_inflated: Vec<StaticRectangularObstacle<f32, 2>>,
}

// SAFETY: Each agent is accessed exclusively by one thread in par_iter_mut.
// SMART contains Box<dyn ValidityChecker> which isn't auto-Send, but we never
// share agents across threads — rayon guarantees disjoint mutable access.
unsafe impl Send for Agent {}
unsafe impl Sync for Agent {}

const AGENT_COLORS: [[f32; 3]; 8] = [
    [1.0, 0.2, 0.2],   // red
    [0.2, 0.4, 1.0],   // blue
    [0.2, 0.9, 0.3],   // green
    [1.0, 0.8, 0.0],   // yellow
    [0.8, 0.2, 0.8],   // purple
    [0.0, 0.8, 0.8],   // cyan
    [1.0, 0.5, 0.0],   // orange
    [0.6, 0.6, 0.6],   // gray
];

// ==================== Wall helpers ====================
fn wall(x1: f32, y1: f32, x2: f32, y2: f32) -> StaticRectangularObstacle<f32, 2> {
    StaticRectangularObstacle::new(
        RealVectorState::new([x1.min(x2), y1.min(y2)]),
        RealVectorState::new([x1.max(x2), y1.max(y2)]),
    )
}

fn inflate_walls(
    walls: &[StaticRectangularObstacle<f32, 2>],
    robot_r: f32,
) -> Vec<StaticRectangularObstacle<f32, 2>> {
    walls.iter().map(|w| {
        let min = w.min_corner().values();
        let max = w.max_corner().values();
        StaticRectangularObstacle::new(
            RealVectorState::new([min[0] - robot_r, min[1] - robot_r]),
            RealVectorState::new([max[0] + robot_r, max[1] + robot_r]),
        )
    }).collect()
}

// ==================== LiDAR ray-casting ====================
const GRID_RES: f32 = 0.5;
const NUM_RAYS: usize = 180;

fn ray_rect_hit(
    ox: f32, oy: f32, dx: f32, dy: f32,
    x1: f32, y1: f32, x2: f32, y2: f32,
) -> Option<f32> {
    let mut best = f32::MAX;
    if dx.abs() > 1e-8 {
        let t = (x1 - ox) / dx;
        if t > 0.001 { let y = oy + t * dy; if y >= y1 && y <= y2 && t < best { best = t; } }
        let t = (x2 - ox) / dx;
        if t > 0.001 { let y = oy + t * dy; if y >= y1 && y <= y2 && t < best { best = t; } }
    }
    if dy.abs() > 1e-8 {
        let t = (y1 - oy) / dy;
        if t > 0.001 { let x = ox + t * dx; if x >= x1 && x <= x2 && t < best { best = t; } }
        let t = (y2 - oy) / dy;
        if t > 0.001 { let x = ox + t * dx; if x >= x1 && x <= x2 && t < best { best = t; } }
    }
    if best < f32::MAX { Some(best) } else { None }
}

fn lidar_scan(
    rx: f32, ry: f32,
    sensor_range: f32,
    true_walls: &[StaticRectangularObstacle<f32, 2>],
    walls_discovered: &mut [bool],
    known_walls: &mut Vec<StaticRectangularObstacle<f32, 2>>,
    grid: &mut [i8],
    grid_w: usize, grid_h: usize,
) -> bool {
    let mut new_wall = false;
    let pi2 = std::f32::consts::PI * 2.0;
    for i in 0..NUM_RAYS {
        let ang = pi2 * i as f32 / NUM_RAYS as f32;
        let dx = ang.cos();
        let dy = ang.sin();
        let mut hit_t = sensor_range;
        let mut hit_wall: Option<usize> = None;
        for (wi, w) in true_walls.iter().enumerate() {
            let mn = w.min_corner().values();
            let mx = w.max_corner().values();
            if let Some(t) = ray_rect_hit(rx, ry, dx, dy, mn[0], mn[1], mx[0], mx[1]) {
                if t < hit_t { hit_t = t; hit_wall = Some(wi); }
            }
        }
        let step = GRID_RES * 0.4;
        let steps = (hit_t / step) as usize;
        for s in 0..=steps {
            let t = (s as f32) * step;
            if t > hit_t { break; }
            let gx = ((rx + t * dx) / GRID_RES) as i32;
            let gy = ((ry + t * dy) / GRID_RES) as i32;
            if gx >= 0 && gy >= 0 && (gx as usize) < grid_w && (gy as usize) < grid_h {
                let idx = (gy as usize) * grid_w + (gx as usize);
                if grid[idx] < 0 { grid[idx] = 0; }
            }
        }
        if let Some(wi) = hit_wall {
            let hx = rx + hit_t * dx;
            let hy = ry + hit_t * dy;
            let gx = (hx / GRID_RES) as i32;
            let gy = (hy / GRID_RES) as i32;
            if gx >= 0 && gy >= 0 && (gx as usize) < grid_w && (gy as usize) < grid_h {
                grid[(gy as usize) * grid_w + (gx as usize)] = 1;
            }
            if !walls_discovered[wi] {
                walls_discovered[wi] = true;
                known_walls.push(true_walls[wi].clone());
                new_wall = true;
            }
        }
    }
    new_wall
}

fn grid_to_string(grid: &[i8]) -> String {
    grid.iter().map(|&v| match v { 0 => 'F', 1 => 'W', _ => 'U' }).collect()
}

// ==================== Wall visibility check ====================
fn is_visible_through_walls(
    rx: f32, ry: f32, tx: f32, ty: f32,
    known_walls: &[StaticRectangularObstacle<f32, 2>],
) -> bool {
    let dx = tx - rx;
    let dy = ty - ry;
    let dist = (dx * dx + dy * dy).sqrt();
    if dist < 1e-6 { return true; }
    let ndx = dx / dist;
    let ndy = dy / dist;
    for w in known_walls {
        let mn = w.min_corner().values();
        let mx = w.max_corner().values();
        if let Some(t) = ray_rect_hit(rx, ry, ndx, ndy, mn[0], mn[1], mx[0], mx[1]) {
            if t < dist { return false; }
        }
    }
    true
}

// ==================== OHZ helper ====================
fn make_spherical_obstacles_with_ohzs(
    obstacles: &[Box<dyn DynamicSphericalObstacle<f32, 2>>],
    obs_speed: f32,
    ohz_time: f32,
) -> Vec<SphericalObstacleWithOhz<f32, 2>> {
    obstacles.iter().map(|o| {
        let inner = StaticSphericalObstacle::new(o.center().clone(), o.radius());
        let ohz_r = o.radius() + obs_speed * ohz_time;
        let ohz = StaticSphericalObstacle::new(o.center().clone(), ohz_r);
        SphericalObstacleWithOhz::new(inner, ohz)
    }).collect()
}

// ==================== Move along path ====================
fn holonomic_move_along_path(
    robot: &RealVectorState<f32, 2>,
    path: &[RealVectorState<f32, 2>],
    mut idx: usize,
    step: f32,
) -> (RealVectorState<f32, 2>, usize) {
    if path.is_empty() || idx >= path.len() {
        return (robot.clone(), idx);
    }
    let mut remaining = step;
    let mut pos = robot.clone();
    while remaining > 1e-6 && idx < path.len() {
        let target = &path[idx];
        let d = pos.euclidean_distance(target);
        if d < 1e-6 {
            idx += 1;
            continue;
        }
        if d <= remaining {
            pos = target.clone();
            remaining -= d;
            idx += 1;
        } else {
            let pv = pos.values();
            let tv = target.values();
            let ratio = remaining / d;
            pos = RealVectorState::new([
                pv[0] + (tv[0] - pv[0]) * ratio,
                pv[1] + (tv[1] - pv[1]) * ratio,
            ]);
            remaining = 0.0;
        }
    }
    (pos, idx)
}

// ==================== Build SMART ====================
fn build_smart(
    from: &RealVectorState<f32, 2>,
    goal: &RealVectorState<f32, 2>,
    walls: &[StaticRectangularObstacle<f32, 2>],
    ranges: [(f32, f32); 2],
    robot_speed: f32,
    robot_radius: f32,
    iters: usize,
    goal_bias: f64,
    shortcut_iters: usize,
    lrz_time: f32,
    lsr_max: f32,
) -> Option<(SMART<f32, 2, KdTreeNearestNeighbors<f32, 2>>, Vec<RealVectorState<f32, 2>>)> {
    // Try up to 3 times with increasing iterations: 1x, 2x, 3x
    for attempt in 0..3 {
        let try_iters = iters * (attempt + 1);
        let result = build_smart_once(from, goal, walls, ranges, robot_speed, robot_radius,
                                       try_iters, goal_bias, shortcut_iters, lrz_time, lsr_max);
        if result.is_some() {
            if attempt > 0 {
                eprintln!("      build_smart succeeded on attempt {} ({} iters)", attempt + 1, try_iters);
            }
            return result;
        }
    }
    None
}

fn build_smart_once(
    from: &RealVectorState<f32, 2>,
    goal: &RealVectorState<f32, 2>,
    walls: &[StaticRectangularObstacle<f32, 2>],
    ranges: [(f32, f32); 2],
    robot_speed: f32,
    robot_radius: f32,
    iters: usize,
    goal_bias: f64,
    shortcut_iters: usize,
    lrz_time: f32,
    lsr_max: f32,
) -> Option<(SMART<f32, 2, KdTreeNearestNeighbors<f32, 2>>, Vec<RealVectorState<f32, 2>>)> {
    let inflated = inflate_walls(walls, robot_radius);
    let static_vc: Box<dyn ValidityChecker<f32, 2>> = if inflated.is_empty() {
        Box::new(AlwaysValid::new())
    } else {
        Box::new(SpatialRectValidityChecker::new(inflated, 4.0))
    };
    let area = (ranges[0].1 - ranges[0].0) * (ranges[1].1 - ranges[1].0);
    let gamma = optimal_gamma(area, 2);
    let sd: Box<dyn SamplingDistribution<f32, 2>> = if goal_bias > 0.001 {
        Box::new(GoalBiasedDistribution::new(ranges, from.clone(), goal_bias))
    } else {
        Box::new(UniformDistribution::new(ranges))
    };
    let mut rrt_star: RRTstar<f32, 2, KdTreeNearestNeighbors<f32, 2>> = RRTstar::new(
        goal.clone(), from.clone(), 1.0, static_vc, sd, 1.0, 1.7, gamma,
    );
    let mut term = MaxIterationsTermination::new(iters);
    rrt_star.plan_until(&mut term);
    let (nodes, nn, svc, sd) = rrt_star.deconstruct_into_components();
    let lrz_radius = robot_speed * lrz_time;
    let hot_node_r = (lsr_max * 0.12).max(1.7);
    let path_node_r = hot_node_r;
    let mut smart = SMART::new_from_initial_rrt_star_tree(
        &nodes, nn, from.clone(), goal.clone(), robot_radius, svc, sd, lrz_radius,
        1.0, 1.5, lsr_max, hot_node_r, path_node_r, false, 0.01,
    );
    smart.initial_plan_add_node_at_start();
    smart.initial_plan_build_path();
    if shortcut_iters > 0 { smart.shortcut_current_path(shortcut_iters); }
    let path = smart.get_path()?;
    Some((smart, path))
}

// ==================== Scenario definitions ====================
struct ScenarioDef {
    name: String,
    world_size: f32,
    walls: Vec<StaticRectangularObstacle<f32, 2>>,
}

fn make_building(ws: f32) -> ScenarioDef {
    let t = 0.3; let door = 2.5; let mut w = Vec::new();
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));
    w.push(wall(t, 16.8, 7.0, 16.8 + t));
    w.push(wall(7.0 + door, 16.8, 16.0, 16.8 + t));
    w.push(wall(16.0 + door, 16.8, 25.0, 16.8 + t));
    w.push(wall(25.0 + door, 16.8, ws - t, 16.8 + t));
    w.push(wall(t, 14.5, 7.0, 14.5 + t));
    w.push(wall(7.0 + door, 14.5, 16.0, 14.5 + t));
    w.push(wall(16.0 + door, 14.5, 25.0, 14.5 + t));
    w.push(wall(25.0 + door, 14.5, ws - t, 14.5 + t));
    w.push(wall(10.5, t, 10.5 + t, 5.5));
    w.push(wall(10.5, 5.5 + door, 10.5 + t, 14.5));
    w.push(wall(21.5, t, 21.5 + t, 8.5));
    w.push(wall(21.5, 8.5 + door, 21.5 + t, 14.5));
    w.push(wall(21.5 + t, 7.5, 24.5, 7.5 + t));
    w.push(wall(24.5 + door, 7.5, ws - t, 7.5 + t));
    w.push(wall(8.5, 16.8 + t, 8.5 + t, 22.5));
    w.push(wall(8.5, 22.5 + door, 8.5 + t, ws - t));
    w.push(wall(20.0, 16.8 + t, 20.0 + t, 20.5));
    w.push(wall(20.0, 20.5 + door, 20.0 + t, ws - t));
    w.push(wall(t, 25.0, 8.5, 25.0 + t));
    w.push(wall(20.0 + t, 25.0, 25.5, 25.0 + t));
    w.push(wall(25.5 + door, 25.0, ws - t, 25.0 + t));
    w.push(wall(3.0, 3.0, 6.0, 3.5));
    w.push(wall(3.0, 6.0, 8.0, 6.5));
    w.push(wall(13.0, 4.0, 14.0, 9.0));
    w.push(wall(17.0, 2.0, 18.0, 8.0));
    w.push(wall(24.0, 1.5, 24.5, 6.0));
    w.push(wall(28.0, 1.5, 28.5, 6.0));
    w.push(wall(24.0, 9.0, 29.0, 9.5));
    w.push(wall(2.5, 19.0, 6.0, 20.0));
    w.push(wall(11.0, 19.5, 11.5, 23.0));
    w.push(wall(15.0, 19.5, 15.5, 23.0));
    w.push(wall(11.0, 21.0, 15.5, 21.5));
    w.push(wall(23.0, 19.0, 23.5, 23.5));
    w.push(wall(27.0, 19.0, 27.5, 23.5));
    w.push(wall(3.0, 27.5, 6.0, 28.0));
    w.push(wall(23.0, 27.0, 28.0, 27.5));
    w.push(wall(25.5, 29.0, 26.5, 30.0));
    ScenarioDef { name: "building".into(), world_size: ws, walls: w }
}

fn make_office(ws: f32) -> ScenarioDef {
    let t = 0.3; let door = 2.5; let mut w = Vec::new();
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));
    w.push(wall(10.5, t, 10.5 + t, 3.5));
    w.push(wall(10.5, 3.5 + door, 10.5 + t, 14.5));
    w.push(wall(10.5, 14.5 + door, 10.5 + t, 25.5));
    w.push(wall(10.5, 25.5 + door, 10.5 + t, ws - t));
    w.push(wall(21.5, t, 21.5 + t, 6.5));
    w.push(wall(21.5, 6.5 + door, 21.5 + t, 18.5));
    w.push(wall(21.5, 18.5 + door, 21.5 + t, 26.5));
    w.push(wall(21.5, 26.5 + door, 21.5 + t, ws - t));
    w.push(wall(t, 10.5, 3.5, 10.5 + t));
    w.push(wall(3.5 + door, 10.5, 10.5, 10.5 + t));
    w.push(wall(10.5 + t, 10.5, 14.5, 10.5 + t));
    w.push(wall(14.5 + door, 10.5, 21.5, 10.5 + t));
    w.push(wall(21.5 + t, 10.5, 25.5, 10.5 + t));
    w.push(wall(25.5 + door, 10.5, ws - t, 10.5 + t));
    w.push(wall(t, 21.5, 2.5, 21.5 + t));
    w.push(wall(2.5 + door, 21.5, 10.5, 21.5 + t));
    w.push(wall(10.5 + t, 21.5, 13.5, 21.5 + t));
    w.push(wall(13.5 + door, 21.5, 21.5, 21.5 + t));
    w.push(wall(21.5 + t, 21.5, 24.5, 21.5 + t));
    w.push(wall(24.5 + door, 21.5, ws - t, 21.5 + t));
    w.push(wall(1.5, 1.5, 4.5, 2.0));
    w.push(wall(1.5, 2.0, 2.0, 5.5));
    w.push(wall(13.0, 3.5, 19.0, 4.0));
    w.push(wall(13.0, 7.5, 19.0, 8.0));
    w.push(wall(24.0, 2.0, 29.5, 2.5));
    w.push(wall(24.0, 2.5, 24.5, 7.5));
    w.push(wall(2.0, 13.0, 2.5, 19.5));
    w.push(wall(5.5, 14.0, 8.5, 14.5));
    w.push(wall(13.0, 13.5, 14.0, 19.0));
    w.push(wall(17.5, 13.5, 18.5, 19.0));
    w.push(wall(24.0, 12.5, 26.5, 14.5));
    w.push(wall(28.0, 17.0, 30.5, 19.5));
    w.push(wall(3.0, 24.5, 7.5, 25.0));
    w.push(wall(4.0, 23.5, 6.5, 26.5));
    w.push(wall(12.5, 24.0, 13.0, 28.5));
    w.push(wall(16.5, 24.0, 17.0, 28.5));
    w.push(wall(19.5, 26.5, 20.5, 30.0));
    w.push(wall(24.5, 24.0, 29.5, 25.0));
    w.push(wall(26.0, 28.0, 27.0, 29.0));
    ScenarioDef { name: "office".into(), world_size: ws, walls: w }
}

fn make_warehouse(ws: f32) -> ScenarioDef {
    let t = 0.3; let mut w = Vec::new();
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));
    for &rx in &[5.0f32, 10.0, 15.0, 20.0, 25.0] {
        w.push(wall(rx, 2.0, rx + 0.8, 6.5));
        w.push(wall(rx, 9.5, rx + 0.8, 14.5));
        w.push(wall(rx, 17.5, rx + 0.8, 22.5));
        w.push(wall(rx, 25.5, rx + 0.8, 30.0));
    }
    w.push(wall(6.0, 12.0, 14.5, 12.0 + t));
    w.push(wall(16.0, 20.0, 24.5, 20.0 + t));
    w.push(wall(27.0, 27.0, ws - t, 27.0 + t));
    w.push(wall(27.0, 27.0 + t, 27.0 + t, 28.5));
    w.push(wall(27.0, 31.0, 27.0 + t, ws - t));
    ScenarioDef { name: "warehouse".into(), world_size: ws, walls: w }
}

fn make_megacampus(ws: f32) -> ScenarioDef {
    let t = 0.3; let door = 2.5; let mut w = Vec::new();
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));
    for &cy in &[32.0f32, 64.0, 96.0] {
        let mut x = t;
        while x < ws - t {
            let gap = x + 8.0;
            if gap > ws - t { break; }
            w.push(wall(x, cy, gap.min(ws - t), cy + t));
            w.push(wall(x, cy - 2.5, gap.min(ws - t), cy - 2.5 + t));
            x = gap + door;
        }
    }
    for &cx in &[32.0f32, 64.0, 96.0] {
        let mut y = t;
        while y < ws - t {
            let gap = y + 8.0;
            if gap > ws - t { break; }
            w.push(wall(cx, y, cx + t, gap.min(ws - t)));
            w.push(wall(cx - 2.5, y, cx - 2.5 + t, gap.min(ws - t)));
            y = gap + door;
        }
    }
    let quads = [
        (2.0, 2.0), (2.0, 34.0), (2.0, 66.0), (2.0, 98.0),
        (34.0, 2.0), (34.0, 34.0), (34.0, 66.0), (34.0, 98.0),
        (66.0, 2.0), (66.0, 34.0), (66.0, 66.0), (66.0, 98.0),
        (98.0, 2.0), (98.0, 34.0), (98.0, 66.0), (98.0, 98.0),
    ];
    for &(qx, qy) in &quads {
        for &rx in &[qx + 8.0, qx + 18.0] {
            if rx + t > ws - 2.0 { continue; }
            w.push(wall(rx, qy, rx + t, qy + 5.0));
            w.push(wall(rx, qy + 5.0 + door, rx + t, qy + 14.0));
            w.push(wall(rx, qy + 14.0 + door, rx + t, qy + 26.0));
        }
        for &ry in &[qy + 10.0, qy + 20.0] {
            if ry + t > ws - 2.0 { continue; }
            w.push(wall(qx, ry, qx + 5.0, ry + t));
            w.push(wall(qx + 5.0 + door, ry, qx + 13.0, ry + t));
            w.push(wall(qx + 13.0 + door, ry, qx + 26.0, ry + t));
        }
        w.push(wall(qx + 2.0, qy + 2.0, qx + 5.0, qy + 2.5));
        w.push(wall(qx + 12.0, qy + 4.0, qx + 13.0, qy + 8.0));
        w.push(wall(qx + 3.0, qy + 14.0, qx + 6.0, qy + 14.5));
        w.push(wall(qx + 22.0, qy + 22.0, qx + 24.0, qy + 22.5));
    }
    for &px in &[48.0f32, 56.0, 64.0, 72.0, 80.0] {
        for &py in &[48.0f32, 56.0, 64.0, 72.0, 80.0] {
            w.push(wall(px, py, px + 1.0, py + 1.0));
        }
    }
    for &(sx, sy) in &[(2.0, 2.0), (2.0, 120.0), (120.0, 2.0), (120.0, 120.0)] {
        w.push(wall(sx, sy, sx + 6.0, sy + t));
        w.push(wall(sx, sy, sx + t, sy + 6.0));
        w.push(wall(sx + 6.0, sy, sx + 6.0 + t, sy + 3.0));
        w.push(wall(sx, sy + 6.0, sx + 3.0, sy + 6.0 + t));
    }
    for i in 0..8 {
        let bx = 8.0 + (i as f32) * 15.0;
        if bx + 10.0 > ws - 2.0 { continue; }
        w.push(wall(bx, 42.0, bx + 10.0, 42.5));
        w.push(wall(bx, 86.0, bx + 10.0, 86.5));
    }
    for i in 0..8 {
        let by = 8.0 + (i as f32) * 15.0;
        if by + 10.0 > ws - 2.0 { continue; }
        w.push(wall(42.0, by, 42.5, by + 10.0));
        w.push(wall(86.0, by, 86.5, by + 10.0));
    }
    ScenarioDef { name: "megacampus".into(), world_size: ws, walls: w }
}

fn make_mega_hospital(ws: f32) -> ScenarioDef {
    let t = 0.3; let door = 2.0; let mut w = Vec::new();
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));
    { let mut x = t; while x < ws - t { let seg_end = (x + 8.0).min(ws - t); w.push(wall(x, 44.0, seg_end, 44.0 + t)); x = seg_end + door; } }
    { let mut x = t; while x < ws - t { let seg_end = (x + 8.0).min(ws - t); w.push(wall(x, 49.0, seg_end, 49.0 + t)); x = seg_end + door; } }
    for &cx in &[24.0f32, 48.0, 72.0] {
        { let mut y = t; while y < ws - t { if y >= 43.0 && y <= 50.0 { y = 50.5; continue; } let seg_end = (y + 6.0).min(ws - t); if seg_end > 43.0 && y < 43.0 { w.push(wall(cx, y, cx + t, 43.0)); y = 50.0; continue; } w.push(wall(cx, y, cx + t, seg_end)); y = seg_end + door; } }
        { let mut y = t; while y < ws - t { if y >= 43.0 && y <= 50.0 { y = 50.5; continue; } let seg_end = (y + 6.0).min(ws - t); if seg_end > 43.0 && y < 43.0 { w.push(wall(cx + 3.0, y, cx + 3.0 + t, 43.0)); y = 50.0; continue; } w.push(wall(cx + 3.0, y, cx + 3.0 + t, seg_end)); y = seg_end + door; } }
    }
    let north_zones = [(1.5, 23.5), (27.5, 47.5), (51.5, 71.5), (75.5, 94.5)];
    for &(zx_start, zx_end) in &north_zones {
        let zone_w = zx_end - zx_start;
        if zone_w < 8.0 { continue; }
        for &ry in &[58.0f32, 68.0, 78.0, 88.0] {
            if ry + t > ws - 2.0 { continue; }
            let mut x = zx_start;
            while x < zx_end { let seg_end = (x + 5.0f32).min(zx_end); w.push(wall(x, ry, seg_end, ry + t)); x = seg_end + door; }
        }
        let mut rx = zx_start + 5.0;
        while rx < zx_end - 2.0 {
            for &(ry_lo, ry_hi) in &[(51.0, 57.5), (58.5, 67.5), (68.5, 77.5), (78.5, 87.5), (88.5, 94.5)] {
                if ry_hi > ws - 2.0 { continue; }
                let wall_hi = ry_hi.min(ws - 1.5);
                let mid = (ry_lo + wall_hi) / 2.0;
                if mid - 1.0 > ry_lo { w.push(wall(rx, ry_lo, rx + t, mid - 1.0)); }
                if mid + 1.0 < wall_hi { w.push(wall(rx, mid + 1.0, rx + t, wall_hi)); }
            }
            rx += 5.0;
        }
    }
    for &bx in &[10.0f32, 18.0, 26.0, 34.0, 42.0] {
        if bx + t > 46.0 { continue; }
        w.push(wall(bx, 2.5, bx + t, 10.0));
        w.push(wall(bx, 14.0, bx + t, 22.0));
    }
    w.push(wall(2.0, 11.5, 20.0, 11.5 + t));
    w.push(wall(22.0, 11.5, 46.0, 11.5 + t));
    w.push(wall(2.0, 23.0, 15.0, 23.0 + t));
    w.push(wall(17.0, 23.0, 23.0, 23.0 + t));
    w.push(wall(23.0, 15.0, 23.0 + t, 23.0));
    w.push(wall(50.0, 28.0, 60.0, 28.0 + t));
    w.push(wall(62.0, 28.0, 75.0, 28.0 + t));
    w.push(wall(77.0, 28.0, 94.0, 28.0 + t));
    w.push(wall(50.0, 31.0, 65.0, 31.0 + t));
    w.push(wall(67.0, 31.0, 80.0, 31.0 + t));
    w.push(wall(82.0, 31.0, 94.0, 31.0 + t));
    for &orx in &[54.0f32, 62.0, 70.0, 78.0, 86.0] {
        if orx + t > 94.0 { continue; }
        w.push(wall(orx, 2.5, orx + t, 12.0));
        w.push(wall(orx, 14.0, orx + t, 27.5));
    }
    w.push(wall(50.5, 13.0, 65.0, 13.0 + t));
    w.push(wall(67.0, 13.0, 80.0, 13.0 + t));
    w.push(wall(82.0, 13.0, 93.5, 13.0 + t));
    w.push(wall(50.0, 36.0, 62.0, 36.0 + t));
    w.push(wall(64.0, 36.0, 78.0, 36.0 + t));
    w.push(wall(80.0, 36.0, 94.0, 36.0 + t));
    w.push(wall(50.0, 40.0, 58.0, 40.0 + t));
    w.push(wall(60.0, 40.0, 75.0, 40.0 + t));
    w.push(wall(77.0, 40.0, 94.0, 40.0 + t));
    for &rx in &[58.0f32, 68.0, 80.0] {
        if rx + 0.6 > 94.0 { continue; }
        w.push(wall(rx, 33.0, rx + 0.6, 36.0));
        w.push(wall(rx, 36.5, rx + 0.6, 40.0));
        w.push(wall(rx, 40.5, rx + 0.6, 43.5));
    }
    w.push(wall(5.0, 30.0, 8.0, 30.5));
    w.push(wall(5.0, 34.0, 8.0, 34.5));
    w.push(wall(12.0, 28.0, 15.0, 28.5));
    w.push(wall(12.0, 32.0, 15.0, 32.5));
    w.push(wall(12.0, 36.0, 15.0, 36.5));
    w.push(wall(18.0, 30.0, 21.0, 30.5));
    w.push(wall(18.0, 38.0, 21.0, 38.5));
    for &(sx, sy) in &[(2.0, 2.0), (2.0, 88.0), (88.0, 2.0), (88.0, 88.0)] {
        w.push(wall(sx, sy, sx + 5.5, sy + t));
        w.push(wall(sx, sy, sx + t, sy + 5.5));
        w.push(wall(sx + 5.5, sy, sx + 5.5 + t, sy + 2.5));
        w.push(wall(sx, sy + 5.5, sx + 2.5, sy + 5.5 + t));
    }
    for &(nx, ny) in &[(25.5, 46.0), (49.5, 46.0), (73.5, 46.0)] {
        w.push(wall(nx, ny, nx + 1.5, ny + 0.5));
        w.push(wall(nx, ny + 2.0, nx + 1.5, ny + 2.5));
    }
    w.push(wall(46.0, 46.5, 46.5, 48.5));
    w.push(wall(49.5, 46.5, 50.0, 48.5));
    ScenarioDef { name: "mega_hospital".into(), world_size: ws, walls: w }
}

fn make_mega_airport(ws: f32) -> ScenarioDef {
    let t = 0.3; let door = 2.5; let mut w = Vec::new();
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));
    for &cy in &[6.0f32, 12.0, 18.0, 24.0] {
        for col in 0..6 {
            let cx = 15.0 + (col as f32) * 22.0;
            if cx + 12.0 > ws - 5.0 { continue; }
            w.push(wall(cx, cy, cx + 12.0, cy + 0.5));
        }
    }
    for &px in &[20.0f32, 50.0, 80.0, 110.0, 140.0] {
        for &py in &[8.0f32, 20.0] {
            if px + 1.0 > ws - 2.0 { continue; }
            w.push(wall(px, py, px + 1.0, py + 1.0));
        }
    }
    let sec_y = 28.0;
    let lane_positions = [35.0f32, 65.0, 95.0, 125.0];
    let mut prev_x = t;
    for &lx in &lane_positions {
        w.push(wall(prev_x, sec_y, lx, sec_y + 2.0));
        prev_x = lx + door;
    }
    w.push(wall(prev_x, sec_y, ws - t, sec_y + 2.0));
    { let mut x = t; while x < ws - t { let seg = (x + 10.0).min(ws - t); w.push(wall(x, 52.0, seg, 52.0 + t)); x = seg + door; } }
    for &sx in &[5.0f32, 25.0, 45.0, 65.0, 95.0, 115.0, 135.0] {
        if sx + 12.0 > ws - 3.0 { continue; }
        w.push(wall(sx, 33.0, sx + 12.0, 33.0 + t));
        w.push(wall(sx, 33.0, sx + t, 38.0));
        w.push(wall(sx + 12.0, 33.0, sx + 12.0 + t, 38.0));
        w.push(wall(sx, 38.0, sx + 4.0, 38.0 + t));
        w.push(wall(sx + 6.5, 38.0, sx + 12.0, 38.0 + t));
    }
    for &(fx, fy) in &[(30.0, 42.0), (60.0, 42.0), (90.0, 42.0), (120.0, 42.0)] {
        if fx + 8.0 > ws - 3.0 { continue; }
        w.push(wall(fx, fy, fx + 8.0, fy + 0.5));
        w.push(wall(fx, fy + 4.0, fx + 8.0, fy + 4.5));
    }
    { let mut y = 52.5; while y < 145.0 { let seg = (y + 12.0f32).min(145.0); w.push(wall(73.0, y, 73.0 + t, seg)); y = seg + door; } }
    { let mut y = 52.5; while y < 145.0 { let seg = (y + 12.0f32).min(145.0); w.push(wall(87.0, y, 87.0 + t, seg)); y = seg + door; } }
    for &py in &[60.0f32, 75.0, 90.0, 105.0, 120.0, 135.0] {
        w.push(wall(78.0, py, 79.0, py + 1.0));
        w.push(wall(81.0, py, 82.0, py + 1.0));
    }
    let gate_ys = [68.0f32, 93.0, 118.0];
    for &gy in &gate_ys {
        { let mut x = 5.0f32; while x < 72.5 { let seg = (x + 8.0).min(72.5); w.push(wall(x, gy, seg, gy + t)); x = seg + door; } }
        { let mut x = 5.0f32; while x < 72.5 { let seg = (x + 8.0).min(72.5); w.push(wall(x, gy + 3.0, seg, gy + 3.0 + t)); x = seg + door; } }
        for gate_i in 0..4 {
            let gx = 8.0 + (gate_i as f32) * 16.0;
            if gx + 10.0 > 70.0 { continue; }
            w.push(wall(gx, gy - 8.0, gx + t, gy - 0.5));
            w.push(wall(gx + 10.0, gy - 8.0, gx + 10.0 + t, gy - 0.5));
            w.push(wall(gx, gy - 8.0, gx + 3.5, gy - 8.0 + t));
            w.push(wall(gx + 6.0, gy - 8.0, gx + 10.0, gy - 8.0 + t));
            w.push(wall(gx + 3.5, gy - 14.0, gx + 3.5 + t, gy - 8.0));
            w.push(wall(gx + 6.0, gy - 14.0, gx + 6.0 + t, gy - 8.0));
            w.push(wall(gx + 3.5, gy - 14.0, gx + 6.0, gy - 14.0 + t));
        }
        for gate_i in 0..4 {
            let gx = 8.0 + (gate_i as f32) * 16.0;
            if gx + 10.0 > 70.0 { continue; }
            w.push(wall(gx, gy + 3.5, gx + t, gy + 11.5));
            w.push(wall(gx + 10.0, gy + 3.5, gx + 10.0 + t, gy + 11.5));
            w.push(wall(gx, gy + 11.5, gx + 3.5, gy + 11.5 + t));
            w.push(wall(gx + 6.0, gy + 11.5, gx + 10.0, gy + 11.5 + t));
        }
        { let mut x = 87.5f32; while x < ws - 5.0 { let seg = (x + 8.0).min(ws - 5.0); w.push(wall(x, gy, seg, gy + t)); x = seg + door; } }
        { let mut x = 87.5f32; while x < ws - 5.0 { let seg = (x + 8.0).min(ws - 5.0); w.push(wall(x, gy + 3.0, seg, gy + 3.0 + t)); x = seg + door; } }
        for gate_i in 0..4 {
            let gx = 90.0 + (gate_i as f32) * 16.0;
            if gx + 10.0 > ws - 3.0 { continue; }
            w.push(wall(gx, gy - 8.0, gx + t, gy - 0.5));
            w.push(wall(gx + 10.0, gy - 8.0, gx + 10.0 + t, gy - 0.5));
            w.push(wall(gx, gy - 8.0, gx + 3.5, gy - 8.0 + t));
            w.push(wall(gx + 6.0, gy - 8.0, gx + 10.0, gy - 8.0 + t));
            w.push(wall(gx + 3.5, gy - 14.0, gx + 3.5 + t, gy - 8.0));
            w.push(wall(gx + 6.0, gy - 14.0, gx + 6.0 + t, gy - 8.0));
            w.push(wall(gx + 3.5, gy - 14.0, gx + 6.0, gy - 14.0 + t));
        }
        for gate_i in 0..4 {
            let gx = 90.0 + (gate_i as f32) * 16.0;
            if gx + 10.0 > ws - 3.0 { continue; }
            w.push(wall(gx, gy + 3.5, gx + t, gy + 11.5));
            w.push(wall(gx + 10.0, gy + 3.5, gx + 10.0 + t, gy + 11.5));
            w.push(wall(gx, gy + 11.5, gx + 3.5, gy + 11.5 + t));
            w.push(wall(gx + 6.0, gy + 11.5, gx + 10.0, gy + 11.5 + t));
        }
    }
    w.push(wall(73.0, 145.0, 87.0, 145.0 + t));
    ScenarioDef { name: "mega_airport".into(), world_size: ws, walls: w }
}

fn make_mega_university(ws: f32) -> ScenarioDef {
    let t = 0.3; let door = 2.5; let mut w = Vec::new();
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));

    macro_rules! building_shell {
        ($bx:expr, $by:expr, $bw:expr, $bh:expr) => {{
            let (bx, by, bw, bh) = ($bx as f32, $by as f32, $bw as f32, $bh as f32);
            w.push(wall(bx, by, bx + bw * 0.4, by + t));
            w.push(wall(bx + bw * 0.4 + door, by, bx + bw, by + t));
            w.push(wall(bx, by + bh, bx + bw * 0.6, by + bh + t));
            w.push(wall(bx + bw * 0.6 + door, by + bh, bx + bw, by + bh + t));
            w.push(wall(bx, by, bx + t, by + bh * 0.45));
            w.push(wall(bx, by + bh * 0.45 + door, bx + t, by + bh));
            w.push(wall(bx + bw, by, bx + bw + t, by + bh * 0.55));
            w.push(wall(bx + bw, by + bh * 0.55 + door, bx + bw + t, by + bh));
        }};
    }

    // LIBRARY (NW)
    building_shell!(8.0, 130.0, 50.0, 50.0);
    { let mut sx = 12.0f32; while sx < 54.0 { w.push(wall(sx, 135.0, sx + t, 155.0)); w.push(wall(sx, 158.0, sx + t, 176.0)); sx += 3.5; } }
    w.push(wall(12.0, 155.0, 30.0, 155.0 + t));
    w.push(wall(32.5, 155.0, 55.0, 155.0 + t));
    w.push(wall(12.0, 165.0, 25.0, 165.0 + t));
    w.push(wall(27.5, 165.0, 55.0, 165.0 + t));

    // ENGINEERING (NE)
    building_shell!(130.0, 130.0, 55.0, 50.0);
    for &lx in &[148.0f32, 165.0] {
        w.push(wall(lx, 132.0, lx + t, 148.0));
        w.push(wall(lx, 150.5, lx + t, 165.0));
        w.push(wall(lx, 167.5, lx + t, 178.0));
    }
    for &ly in &[148.0f32, 162.0] {
        w.push(wall(132.0, ly, 147.5, ly + t));
        w.push(wall(150.0, ly, 164.5, ly + t));
        w.push(wall(167.0, ly, 183.0, ly + t));
    }
    for &(ex, ey) in &[(135.0,136.0),(135.0,142.0),(152.0,136.0),(152.0,142.0),(170.0,136.0),(170.0,142.0),(135.0,152.0),(152.0,152.0),(170.0,152.0),(135.0,170.0),(152.0,170.0),(170.0,170.0)] {
        w.push(wall(ex, ey, ex + 8.0, ey + 0.5));
    }

    // ADMIN (SW)
    building_shell!(8.0, 8.0, 47.0, 47.0);
    { let mut cx = 13.0f32; while cx < 52.0 { w.push(wall(cx, 10.0, cx + t, 22.0)); w.push(wall(cx, 24.5, cx + t, 36.0)); w.push(wall(cx, 38.5, cx + t, 52.0)); cx += 4.0; } }
    for &cy in &[22.0f32, 36.0] {
        let mut cx2 = 10.0f32;
        while cx2 < 52.0 { let seg = (cx2 + 3.0).min(52.0); w.push(wall(cx2, cy, seg, cy + t)); cx2 = seg + 1.5; }
    }
    w.push(wall(10.0, 10.0, 18.0, 10.0 + t));
    w.push(wall(18.0, 10.0, 18.0 + t, 18.0));
    w.push(wall(10.0, 18.0, 18.0, 18.0 + t));
    w.push(wall(40.0, 42.0, 52.0, 42.0 + t));
    w.push(wall(40.0, 42.0, 40.0 + t, 52.0));

    // SCIENCE (SE)
    building_shell!(130.0, 8.0, 55.0, 47.0);
    for row in 0..5 { let ly = 12.0 + (row as f32) * 8.0; if ly + 6.0 > 52.0 { continue; } for col in 0..4 { let lx = 134.0 + (col as f32) * 13.0; if lx + 10.0 > 183.0 { continue; } w.push(wall(lx, ly, lx + 10.0, ly + 0.5)); } }
    for &fy in &[18.0f32, 34.0] { w.push(wall(132.0, fy, 155.0, fy + 0.6)); w.push(wall(158.0, fy, 183.0, fy + 0.6)); }
    w.push(wall(160.0, 10.0, 160.0 + t, 25.0));
    w.push(wall(160.0, 27.5, 160.0 + t, 42.0));

    // GYM (center-south)
    building_shell!(68.0, 10.0, 57.0, 45.0);
    w.push(wall(75.0, 20.0, 85.0, 20.5)); w.push(wall(75.0, 30.0, 85.0, 30.5)); w.push(wall(75.0, 40.0, 85.0, 40.5));
    w.push(wall(95.0, 12.0, 95.0 + t, 28.0)); w.push(wall(95.0, 30.5, 95.0 + t, 52.0));
    for &(cx, cy) in &[(100.0,18.0),(100.0,26.0),(100.0,34.0),(100.0,42.0),(110.0,18.0),(110.0,26.0),(110.0,34.0),(110.0,42.0)] {
        w.push(wall(cx, cy, cx + 6.0, cy + 0.5));
    }

    // LECTURE HALL (center-north)
    building_shell!(68.0, 130.0, 57.0, 45.0);
    for row in 0..6 { let ry = 135.0 + (row as f32) * 6.0; if ry + t > 170.0 { continue; } w.push(wall(73.0, ry, 90.0, ry + t)); w.push(wall(93.0, ry, 120.0, ry + t)); }
    w.push(wall(72.0, 168.0, 122.0, 168.0 + 0.5));
    w.push(wall(90.0 + t, 132.0, 93.0, 132.0 + t));

    // CENTRAL QUAD
    for &(px, py) in &[(82.0,75.0),(82.0,85.0),(82.0,95.0),(82.0,105.0),(100.0,75.0),(100.0,85.0),(100.0,95.0),(100.0,105.0),(113.0,80.0),(113.0,100.0)] {
        w.push(wall(px, py, px + 3.0, py + 0.5));
    }
    w.push(wall(93.0, 88.0, 95.0, 90.0)); w.push(wall(99.0, 88.0, 101.0, 90.0));
    w.push(wall(93.0, 94.0, 95.0, 96.0)); w.push(wall(99.0, 94.0, 101.0, 96.0));

    // COVERED WALKWAYS
    w.push(wall(60.0, 58.0, 60.0 + t, 128.0)); w.push(wall(63.0, 58.0, 63.0 + t, 128.0));
    w.push(wall(127.0, 58.0, 127.0 + t, 128.0)); w.push(wall(129.5, 58.0, 129.5 + t, 128.0));
    w.push(wall(58.0, 56.0, 65.0, 56.0 + t)); w.push(wall(58.0, 58.5, 65.0, 58.5 + t));
    w.push(wall(126.0, 56.0, 128.0, 56.0 + t)); w.push(wall(126.0, 58.5, 128.0, 58.5 + t));
    w.push(wall(60.0, 126.0, 66.0, 126.0 + t)); w.push(wall(60.0, 128.5, 66.0, 128.5 + t));
    w.push(wall(126.0, 126.0, 128.0, 126.0 + t)); w.push(wall(126.0, 128.5, 128.0, 128.5 + t));

    ScenarioDef { name: "mega_university".into(), world_size: ws, walls: w }
}

/// "bottleneck" — Two rooms connected by a narrow corridor (only 1 robot fits)
/// Designed to test priority-based yielding when agents meet head-on.
///
///   ┌──────────┐         ┌──────────┐
///   │          │         │          │
///   │  Room L  │ ═══════ │  Room R  │   corridor y=14.5~17.5, x=13~19
///   │          │         │          │
///   └──────────┘         └──────────┘
///
fn make_bottleneck(ws: f32) -> ScenarioDef {
    let t = 0.3;
    let mut w = Vec::new();

    // Outer walls
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));

    // Left partition (x=13): solid wall with gap at y=14.5~17.5
    w.push(wall(13.0, t, 13.0 + t, 14.5));        // bottom block
    w.push(wall(13.0, 17.5, 13.0 + t, ws - t));    // top block

    // Right partition (x=19): solid wall with gap at y=14.5~17.5
    w.push(wall(19.0, t, 19.0 + t, 14.5));         // bottom block
    w.push(wall(19.0, 17.5, 19.0 + t, ws - t));    // top block

    // Corridor walls (top and bottom boundaries of the corridor)
    w.push(wall(13.0 + t, 14.5, 19.0, 14.5 + t));  // bottom corridor wall
    w.push(wall(13.0 + t, 17.5 - t, 19.0, 17.5));  // top corridor wall

    // Some furniture in rooms for visual interest
    w.push(wall(3.0, 5.0, 6.0, 5.5));
    w.push(wall(3.0, 10.0, 6.0, 10.5));
    w.push(wall(3.0, 22.0, 6.0, 22.5));
    w.push(wall(8.0, 8.0, 8.5, 12.0));
    w.push(wall(24.0, 5.0, 27.0, 5.5));
    w.push(wall(24.0, 10.0, 27.0, 10.5));
    w.push(wall(24.0, 22.0, 27.0, 22.5));
    w.push(wall(22.0, 8.0, 22.5, 12.0));

    ScenarioDef { name: "bottleneck".into(), world_size: ws, walls: w }
}

fn make_scenario(name: &str) -> ScenarioDef {
    match name {
        "building" => make_building(32.0),
        "office" => make_office(32.0),
        "warehouse" => make_warehouse(32.0),
        "megacampus" => make_megacampus(128.0),
        "mega_hospital" => make_mega_hospital(96.0),
        "mega_airport" => make_mega_airport(160.0),
        "mega_university" => make_mega_university(192.0),
        "bottleneck" => make_bottleneck(32.0),
        _ => {
            eprintln!("Unknown scenario '{}', defaulting to 'mega_airport'", name);
            make_mega_airport(160.0)
        }
    }
}

// ==================== Multi-agent placement ====================

/// Check if a position is inside any inflated wall
fn is_position_blocked(
    pos: &[f32; 2],
    walls: &[StaticRectangularObstacle<f32, 2>],
    robot_radius: f32,
) -> bool {
    let inflated = inflate_walls(walls, robot_radius);
    inflated.iter().any(|w| {
        let mn = w.min_corner().values();
        let mx = w.max_corner().values();
        pos[0] >= mn[0] && pos[0] <= mx[0] && pos[1] >= mn[1] && pos[1] <= mx[1]
    })
}

/// Nudge a position out of inflated walls by searching nearby
fn safe_position(
    pos: [f32; 2],
    walls: &[StaticRectangularObstacle<f32, 2>],
    robot_radius: f32,
    ws: f32,
) -> [f32; 2] {
    if !is_position_blocked(&pos, walls, robot_radius) {
        return pos;
    }
    // Spiral search for a safe spot
    for r in 1..20 {
        let step = r as f32 * 0.5;
        for &(dx, dy) in &[(1.0,0.0),(0.0,1.0),(-1.0,0.0),(0.0,-1.0),
                           (0.7,0.7),(-0.7,0.7),(0.7,-0.7),(-0.7,-0.7)] {
            let candidate = [
                (pos[0] + dx * step).clamp(1.5, ws - 1.5),
                (pos[1] + dy * step).clamp(1.5, ws - 1.5),
            ];
            if !is_position_blocked(&candidate, walls, robot_radius) {
                eprintln!("    Nudged ({:.1},{:.1}) → ({:.1},{:.1})", pos[0], pos[1], candidate[0], candidate[1]);
                return candidate;
            }
        }
    }
    pos // give up, let build_smart handle it
}

fn generate_agent_configs(
    mode: &str, num_agents: usize, ws: f32, margin: f32,
    walls: &[StaticRectangularObstacle<f32, 2>], robot_radius: f32,
) -> Vec<([f32; 2], [f32; 2])> {
    let lo = margin;
    let hi = ws - margin;
    let cx = ws / 2.0;
    let cy = ws / 2.0;

    let corners: Vec<[f32; 2]> = vec![
        [lo, lo], [hi, lo], [lo, hi], [hi, hi],
        [cx, lo], [lo, cx], [hi, cx], [cx, hi],
    ];

    match mode {
        "cross" => {
            let pairs: Vec<([f32; 2], [f32; 2])> = vec![
                ([lo, lo], [hi, hi]),
                ([hi, lo], [lo, hi]),
                ([lo, hi], [hi, lo]),
                ([hi, hi], [lo, lo]),
                ([cx, lo], [cx, hi]),
                ([lo, cx], [hi, cx]),
                ([hi, cx], [lo, cx]),
                ([cx, hi], [cx, lo]),
            ];
            pairs.into_iter().take(num_agents).map(|(s, g)| {
                (safe_position(s, walls, robot_radius, ws),
                 safe_position(g, walls, robot_radius, ws))
            }).collect()
        }
        "converge" => {
            let goal = safe_position([cx, cy], walls, robot_radius, ws);
            corners.iter().take(num_agents).map(|&c| {
                (safe_position(c, walls, robot_radius, ws), goal)
            }).collect()
        }
        "scatter" => {
            let offsets: Vec<[f32; 2]> = vec![
                [cx - 3.0, cy - 3.0], [cx + 3.0, cy - 3.0],
                [cx - 3.0, cy + 3.0], [cx + 3.0, cy + 3.0],
                [cx, cy - 3.0], [cx - 3.0, cy],
                [cx + 3.0, cy], [cx, cy + 3.0],
            ];
            offsets.into_iter().zip(corners.into_iter())
                .take(num_agents).map(|(s, g)| {
                    (safe_position(s, walls, robot_radius, ws),
                     safe_position(g, walls, robot_radius, ws))
                }).collect()
        }
        "bottleneck" => {
            // Agents on opposite sides of corridor, facing each other head-on
            // Corridor is at y=16 (center), left room x<13, right room x>19
            let pairs: Vec<([f32; 2], [f32; 2])> = vec![
                ([6.0, 16.0], [26.0, 16.0]),   // A0: left→right corridor
                ([26.0, 16.0], [6.0, 16.0]),   // A1: right→left corridor
                ([6.0, 10.0], [26.0, 22.0]),   // A2: left-low→right-high
                ([26.0, 22.0], [6.0, 10.0]),   // A3: right-high→left-low
                ([4.0, 16.0], [28.0, 16.0]),   // A4: left→right corridor (offset)
                ([28.0, 16.0], [4.0, 16.0]),   // A5: right→left corridor (offset)
                ([6.0, 22.0], [26.0, 10.0]),   // A6: left-high→right-low
                ([26.0, 10.0], [6.0, 22.0]),   // A7: right-low→left-high
            ];
            pairs.into_iter().take(num_agents).map(|(s, g)| {
                (safe_position(s, walls, robot_radius, ws),
                 safe_position(g, walls, robot_radius, ws))
            }).collect()
        }
        _ => {
            eprintln!("Unknown mode '{}', defaulting to 'cross'", mode);
            generate_agent_configs("cross", num_agents, ws, margin, walls, robot_radius)
        }
    }
}

// ==================== JSON helpers ====================
fn walls_to_json(walls: &[StaticRectangularObstacle<f32, 2>]) -> JsonValue {
    let mut arr = JsonValue::new_array();
    for w in walls {
        let min = w.min_corner().values();
        let max = w.max_corner().values();
        arr.push(json::array![min[0], min[1], max[0], max[1]]).unwrap();
    }
    arr
}

// ==================== Forward-skip helper ====================
fn forward_skip(
    robot: &RealVectorState<f32, 2>,
    goal: &RealVectorState<f32, 2>,
    path: &[RealVectorState<f32, 2>],
    cached_inflated: &[StaticRectangularObstacle<f32, 2>],
) -> usize {
    let rv = robot.values();
    let gv = goal.values();
    let to_goal = [gv[0] - rv[0], gv[1] - rv[1]];
    let max_skip_dist = 5.0f32;
    let scan = path.len().min(15);
    for i in 1..scan {
        let pv = path[i].values();
        let to_pt = [pv[0] - rv[0], pv[1] - rv[1]];
        let d = (to_pt[0] * to_pt[0] + to_pt[1] * to_pt[1]).sqrt();
        if d > max_skip_dist { break; }
        let dot_fwd = to_pt[0] * to_goal[0] + to_pt[1] * to_goal[1];
        if dot_fwd <= 0.0 { continue; }
        let blocked = cached_inflated.iter().any(|w| w.intersects_edge(robot, &path[i]));
        if !blocked { return i; }
    }
    0
}


// ==================== MAIN SIMULATION ====================
fn run_multi_agent(scenario: &ScenarioDef, args: &CliArgs, out: &mut io::StdoutLock) {
    let ws = scenario.world_size;
    let ranges = [(0.0, ws), (0.0, ws)];
    let iters = args.rrt_iterations.max(5000);
    let dt = args.dt;
    let ohz_time = 0.4;

    eprintln!("\n{}", "=".repeat(80));
    eprintln!("  SMART-3D Multi-Agent | {} | {}x{} | {} agents | mode={}",
        scenario.name, ws, ws, args.num_agents, args.agent_mode);
    eprintln!("  TRUE walls: {} | Sensor: {}m | Robot radius: {}m | RRT* iters: {}{}",
        scenario.walls.len(), args.sensor_range, args.robot_radius, iters,
        if args.senior_mode { " | SENIOR MODE" } else { "" });
    eprintln!("{}", "=".repeat(80));

    // Generate agent start/goal pairs
    // Margin scales with world size: ~2.5m for 32m maps, ~8m for 160m maps
    let margin = (ws * 0.05).max(2.0).min(10.0);
    let configs = generate_agent_configs(&args.agent_mode, args.num_agents, ws, margin,
                                          &scenario.walls, args.robot_radius);

    // Global shared state
    let mut known_walls: Vec<StaticRectangularObstacle<f32, 2>> = Vec::new();
    let mut walls_discovered: Vec<bool> = vec![false; scenario.walls.len()];
    let grid_w = (ws / GRID_RES) as usize;
    let grid_h = (ws / GRID_RES) as usize;
    let mut scanned_grid: Vec<i8> = vec![-1; grid_w * grid_h];

    // Per-agent wall state (only used in --no-wall-sharing mode)
    let num_agents_actual = configs.len();
    let mut per_agent_known_walls: Vec<Vec<StaticRectangularObstacle<f32, 2>>> = vec![Vec::new(); num_agents_actual];
    let mut per_agent_walls_discovered: Vec<Vec<bool>> = vec![vec![false; scenario.walls.len()]; num_agents_actual];

    // Initial LiDAR scan from all agent start positions
    for (idx, (start, _)) in configs.iter().enumerate() {
        if args.no_wall_sharing {
            // Each agent only discovers walls visible from its own spawn
            lidar_scan(
                start[0], start[1], args.sensor_range, &scenario.walls,
                &mut per_agent_walls_discovered[idx], &mut per_agent_known_walls[idx],
                &mut scanned_grid, grid_w, grid_h,
            );
        } else {
            lidar_scan(
                start[0], start[1], args.sensor_range, &scenario.walls,
                &mut walls_discovered, &mut known_walls, &mut scanned_grid, grid_w, grid_h,
            );
        }
    }
    if args.no_wall_sharing {
        eprintln!("  [NO WALL SHARING] Per-agent initial scans: {:?}",
            per_agent_known_walls.iter().map(|w| w.len()).collect::<Vec<_>>());
        // Also populate global for viz/grid (doesn't affect planning)
        for i in 0..num_agents_actual {
            for w in &per_agent_known_walls[i] {
                if !known_walls.iter().any(|kw| {
                    let a = kw.min_corner().values(); let b = kw.max_corner().values();
                    let c = w.min_corner().values(); let d = w.max_corner().values();
                    (a[0] - c[0]).abs() < 0.01 && (a[1] - c[1]).abs() < 0.01 &&
                    (b[0] - d[0]).abs() < 0.01 && (b[1] - d[1]).abs() < 0.01
                }) {
                    known_walls.push(w.clone());
                }
            }
        }
    }
    eprintln!("  Initial combined scan: discovered {}/{} walls", known_walls.len(), scenario.walls.len());

    // Build SMART for each agent
    // Parse priorities (comma-separated, e.g. "1,0,0,0")
    let priorities: Vec<u32> = if args.agent_priorities.is_empty() {
        vec![0; configs.len()]
    } else {
        args.agent_priorities.split(',')
            .map(|s| s.trim().parse::<u32>().unwrap_or(0))
            .collect()
    };

    eprintln!("  Building {} SMART trees ({} iters each)...", configs.len(), iters);
    let build_start = Instant::now();
    let mut agents: Vec<Agent> = Vec::new();
    let mut rng = rand::thread_rng();
    for (i, (start, goal)) in configs.iter().enumerate() {
        let s = RealVectorState::new(*start);
        let g = RealVectorState::new(*goal);
        let name = format!("A{}", i);
        let pri = priorities.get(i).copied().unwrap_or(0);
        let tb: f32 = rng.gen(); // random tiebreak, stable for this run
        eprintln!("    {} ({:.0},{:.0}) → ({:.0},{:.0}) pri={} tb={:.2}",
            name, start[0], start[1], goal[0], goal[1], pri, tb);
        // In no-sharing mode, each agent builds with its own walls only
        let build_walls = if args.no_wall_sharing {
            &per_agent_known_walls[i]
        } else {
            &known_walls
        };
        let own_kw = if args.no_wall_sharing { per_agent_known_walls[i].clone() } else { Vec::new() };
        let own_wd = if args.no_wall_sharing { per_agent_walls_discovered[i].clone() } else { Vec::new() };
        let own_ci = if args.no_wall_sharing { inflate_walls(&own_kw, args.robot_radius) } else { Vec::new() };
        match build_smart(&s, &g, build_walls, ranges, args.robot_speed, args.robot_radius,
                          iters, args.goal_bias, args.shortcut_iters, args.lrz_time, args.lsr_max) {
            Some((smart, path)) => {
                agents.push(Agent {
                    name, color: AGENT_COLORS[i % 8],
                    priority: pri, tiebreak: tb,
                    smart, current_path: path, path_idx: 1,
                    robot: s.clone(), goal: g, reached_goal: false,
                    rebuild_count: 0, frames_since_progress: 0,
                    last_progress_pos: s,
                    own_known_walls: own_kw, own_walls_discovered: own_wd,
                    own_cached_inflated: own_ci,
                });
            }
            None => {
                // Retry up to 5 times (different RNG seed each attempt)
                let mut ok = false;
                for attempt in 1..=5 {
                    eprintln!("    ⚠ {} initial path failed, retry {}/5...", name, attempt);
                    if let Some((smart, path)) = build_smart(&s, &g, build_walls, ranges,
                            args.robot_speed, args.robot_radius, iters,
                            args.goal_bias, args.shortcut_iters, args.lrz_time, args.lsr_max) {
                        agents.push(Agent {
                            name: name.clone(), color: AGENT_COLORS[i % 8],
                            priority: pri, tiebreak: tb,
                            smart, current_path: path, path_idx: 1,
                            robot: s.clone(), goal: g.clone(), reached_goal: false,
                            rebuild_count: 0, frames_since_progress: 0,
                            last_progress_pos: s.clone(),
                            own_known_walls: if args.no_wall_sharing { per_agent_known_walls[i].clone() } else { Vec::new() },
                            own_walls_discovered: if args.no_wall_sharing { per_agent_walls_discovered[i].clone() } else { Vec::new() },
                            own_cached_inflated: if args.no_wall_sharing { inflate_walls(&per_agent_known_walls[i], args.robot_radius) } else { Vec::new() },
                        });
                        eprintln!("    ✓ {} retry {} succeeded", name, attempt);
                        ok = true;
                        break;
                    }
                }
                if !ok {
                    eprintln!("    ❌ {} failed after 5 retries! Skipping.", name);
                }
            }
        }
    }
    let build_elapsed = build_start.elapsed();
    eprintln!("  {} agents initialized in {:.1}s", agents.len(), build_elapsed.as_secs_f32());
    if agents.is_empty() {
        eprintln!("  No agents initialized! Exiting.");
        return;
    }

    let mut cached_inflated = inflate_walls(&known_walls, args.robot_radius);
    let mut total_walls_known = known_walls.len();

    // ---- Spawn dynamic obstacles (people) ----
    let obstacle_radius = 0.6f32;
    let ohz_time_people = 0.5f32;
    let spawn_ranges = [(5.0f32, ws - 5.0), (5.0f32, ws - 5.0)];
    let mut obs_sampling = UniformDistribution::new(spawn_ranges);
    let mut obstacles: Vec<Box<dyn DynamicSphericalObstacle<f32, 2>>> = Vec::new();
    if args.num_obstacles > 0 {
        let margin_obs = obstacle_radius + 0.3;
        let mut spawned = 0usize;
        let mut attempts = 0usize;
        while spawned < args.num_obstacles && attempts < args.num_obstacles * 50 {
            let pos = obs_sampling.sample();
            attempts += 1;
            let pv = pos.values();
            let in_wall = scenario.walls.iter().any(|w| {
                let min = w.min_corner().values();
                let max = w.max_corner().values();
                pv[0] >= min[0] - margin_obs && pv[0] <= max[0] + margin_obs &&
                pv[1] >= min[1] - margin_obs && pv[1] <= max[1] + margin_obs
            });
            // Don't spawn near any agent start or goal
            let near_agent = agents.iter().any(|a| {
                let sv = a.robot.values();
                let gv = a.goal.values();
                let ds = (pv[0]-sv[0]).powi(2) + (pv[1]-sv[1]).powi(2);
                let dg = (pv[0]-gv[0]).powi(2) + (pv[1]-gv[1]).powi(2);
                ds < 9.0 || dg < 9.0
            });
            if !in_wall && !near_agent {
                let c = StaticSphericalObstacle::new(pos, obstacle_radius);
                obstacles.push(Box::new(RandomDirectionCircle::new(
                    c, args.obstacle_speed, 0.5, 5.0, spawn_ranges, scenario.walls.clone(),
                )));
                spawned += 1;
            }
        }
        eprintln!("  Spawned {} people", spawned);
    }

    // Metadata JSON
    let mut agents_meta = JsonValue::new_array();
    for a in &agents {
        let sv = a.robot.values();
        let gv = a.goal.values();
        agents_meta.push(object! {
            "name": a.name.as_str(), "start": [sv[0], sv[1]], "goal": [gv[0], gv[1]],
            "color": [a.color[0], a.color[1], a.color[2]]
        }).unwrap();
    }
    let meta = object! {
        "type": "metadata", "scenario": scenario.name.as_str(), "world_size": ws,
        "num_agents": agents.len() as u32, "agent_mode": args.agent_mode.as_str(),
        "num_obstacles": obstacles.len() as u32,
        "obstacle_radius": obstacle_radius, "obstacle_speed": args.obstacle_speed,
        "robot_speed": args.robot_speed, "robot_radius": args.robot_radius,
        "sensor_range": args.sensor_range, "dt": dt,
        "walls": walls_to_json(&scenario.walls),
        "agents": agents_meta,
        "grid_res": GRID_RES, "grid_w": grid_w as u32, "grid_h": grid_h as u32,
        "grid": grid_to_string(&scanned_grid),
    };
    if writeln!(out, "{}", meta.dump()).is_err() { return; }

    // ==================== Main loop ====================
    let mut frame: u64 = 0;
    let mut sim_time = 0.0f32;
    let delay = Duration::from_secs_f32(dt / args.playback_speed);

    loop {
        let t0 = Instant::now();
        sim_time += dt;

        // ---- Step 0: Move people ----
        for o in &mut obstacles { o.update(dt); }
        let all_people_ohz = make_spherical_obstacles_with_ohzs(&obstacles, args.obstacle_speed, ohz_time_people);

        // ---- Phase 1+2: Wall discovery and tree repair ----
        if args.no_wall_sharing {
            // ======== NO WALL SHARING MODE ========
            // Each agent scans into its own wall list, repairs its own tree
            let mut any_new_wall_global = false;
            for i in 0..agents.len() {
                if agents[i].reached_goal { continue; }
                let rx = agents[i].robot.values()[0];
                let ry = agents[i].robot.values()[1];
                // Take fields out to satisfy borrow checker
                let mut tmp_disc = std::mem::take(&mut agents[i].own_walls_discovered);
                let mut tmp_walls = std::mem::take(&mut agents[i].own_known_walls);
                let found = lidar_scan(
                    rx, ry, args.sensor_range, &scenario.walls,
                    &mut tmp_disc, &mut tmp_walls,
                    &mut scanned_grid, grid_w, grid_h,
                );
                agents[i].own_walls_discovered = tmp_disc;
                agents[i].own_known_walls = tmp_walls;
                if !found { continue; }
                any_new_wall_global = true;

                // Rebuild this agent's own inflated walls
                agents[i].own_cached_inflated = inflate_walls(&agents[i].own_known_walls, args.robot_radius);
                let my_inflated = agents[i].own_cached_inflated.clone();
                let my_known_walls = agents[i].own_known_walls.clone();

                // Check goal unreachable
                let gv = agents[i].goal.values();
                let goal_blocked = my_inflated.iter().any(|w| {
                    let mn = w.min_corner().values(); let mx = w.max_corner().values();
                    gv[0] >= mn[0] && gv[0] <= mx[0] && gv[1] >= mn[1] && gv[1] <= mx[1]
                });
                if goal_blocked {
                    eprintln!("  ❌ {} UNREACHABLE: goal inside wall", agents[i].name);
                    agents[i].reached_goal = true;
                    continue;
                }

                // Check path affected
                let path_affected = agents[i].current_path.windows(2).any(|pair| {
                    my_inflated.iter().any(|w| w.intersects_edge(&pair[0], &pair[1]))
                });

                if !path_affected {
                    let vc: Box<dyn ValidityChecker<f32, 2>> = if my_inflated.is_empty() {
                        Box::new(AlwaysValid::new())
                    } else {
                        Box::new(SpatialRectValidityChecker::new(my_inflated.clone(), 4.0))
                    };
                    agents[i].smart.set_static_validity_checker(vc);
                } else {
                    let vc: Box<dyn ValidityChecker<f32, 2>> =
                        Box::new(SpatialRectValidityChecker::new(my_inflated.clone(), 4.0));
                    let robot_clone = agents[i].robot.clone();
                    let mut rt = MaxTimeTermination::new(Duration::from_secs_f32(dt * 3.0));
                    let inc_result = panic::catch_unwind(panic::AssertUnwindSafe(|| {
                        agents[i].smart.update_static_obstacles(vc, Some(&mut rt), robot_clone)
                    }));
                    match inc_result {
                        Ok(SmartUpdateResult::ReplanningSuccessful) => {
                            if let Some(new_path) = agents[i].smart.get_path() {
                                let skip = forward_skip(&agents[i].robot, &agents[i].goal, &new_path, &my_inflated);
                                agents[i].current_path = new_path;
                                agents[i].path_idx = skip;
                            } else {
                                agents[i].rebuild_count += 1;
                                if let Some((ns, np)) = build_smart(&agents[i].robot, &agents[i].goal,
                                        &my_known_walls, ranges, args.robot_speed, args.robot_radius,
                                        iters, args.goal_bias, args.shortcut_iters, args.lrz_time, args.lsr_max) {
                                    agents[i].smart = ns;
                                    agents[i].current_path = np;
                                    agents[i].path_idx = 1;
                                }
                            }
                        }
                        Ok(SmartUpdateResult::NoReplanningNeeded) => {}
                        _ => {
                            agents[i].rebuild_count += 1;
                            if let Some((ns, np)) = build_smart(&agents[i].robot, &agents[i].goal,
                                    &my_known_walls, ranges, args.robot_speed, args.robot_radius,
                                    iters, args.goal_bias, args.shortcut_iters, args.lrz_time, args.lsr_max) {
                                agents[i].smart = ns;
                                agents[i].current_path = np;
                                agents[i].path_idx = 1;
                            }
                        }
                    }
                }
            }
            // Update global known_walls for viz only (doesn't affect planning in no-sharing)
            if any_new_wall_global {
                known_walls.clear();
                walls_discovered.iter_mut().for_each(|w| *w = false);
                for agent in &agents {
                    for (wi, &disc) in agent.own_walls_discovered.iter().enumerate() {
                        if disc && !walls_discovered[wi] {
                            walls_discovered[wi] = true;
                            known_walls.push(scenario.walls[wi].clone());
                        }
                    }
                }
                total_walls_known = known_walls.len();
                cached_inflated = inflate_walls(&known_walls, args.robot_radius);
            }
        } else {
            // ======== SHARED WALL MODE (default) ========
            // Phase 1: LiDAR scan from ALL agents, merge into global walls
            let mut any_new_wall = false;
            for i in 0..agents.len() {
                if agents[i].reached_goal { continue; }
                let rv = agents[i].robot.values();
                let found = lidar_scan(
                    rv[0], rv[1], args.sensor_range, &scenario.walls,
                    &mut walls_discovered, &mut known_walls, &mut scanned_grid, grid_w, grid_h,
                );
                if found { any_new_wall = true; }
            }

            // Phase 2: If new walls, update ALL agents' trees
            if any_new_wall {
                let new_count = known_walls.len() - total_walls_known;
                total_walls_known = known_walls.len();
                eprintln!("  [t={:.1}s] 🔍 +{} wall(s), total {}/{}", sim_time, new_count, known_walls.len(), scenario.walls.len());

                // Rebuild inflated walls from scratch (all known walls)
                cached_inflated = inflate_walls(&known_walls, args.robot_radius);

                // Check if any agent's goal just became unreachable (inside lethal wall)
                for i in 0..agents.len() {
                    if agents[i].reached_goal { continue; }
                    let gv = agents[i].goal.values();
                    let goal_blocked = cached_inflated.iter().any(|w| {
                        let mn = w.min_corner().values();
                        let mx = w.max_corner().values();
                        gv[0] >= mn[0] && gv[0] <= mx[0] && gv[1] >= mn[1] && gv[1] <= mx[1]
                    });
                    if goal_blocked {
                        eprintln!("  ❌ {} UNREACHABLE: goal ({:.1},{:.1}) inside wall", agents[i].name, gv[0], gv[1]);
                        agents[i].reached_goal = true;
                    }
                }

                // Parallel repair: each agent's tree is independent
                agents.par_iter_mut().for_each(|agent| {
                    if agent.reached_goal { return; }

                    // Check if this agent's path is affected by any known inflated wall
                    let path_affected = agent.current_path.windows(2).any(|pair| {
                        cached_inflated.iter().any(|w| w.intersects_edge(&pair[0], &pair[1]))
                    });

                    if !path_affected && !args.senior_mode {
                        // Lazy checker swap (not used in senior mode)
                        let vc: Box<dyn ValidityChecker<f32, 2>> = if cached_inflated.is_empty() {
                            Box::new(AlwaysValid::new())
                        } else {
                            Box::new(SpatialRectValidityChecker::new(cached_inflated.clone(), 4.0))
                        };
                        agent.smart.set_static_validity_checker(vc);
                    } else {
                        // Incremental tree repair
                        let vc: Box<dyn ValidityChecker<f32, 2>> =
                            Box::new(SpatialRectValidityChecker::new(cached_inflated.clone(), 4.0));
                        let robot_clone = agent.robot.clone();
                        let mut rt = MaxTimeTermination::new(Duration::from_secs_f32(dt * 3.0));

                        let inc_result = if args.senior_mode {
                            // Senior baseline: path-only repair
                            panic::catch_unwind(panic::AssertUnwindSafe(|| {
                                agent.smart.update_static_obstacles_path_only(vc, Some(&mut rt), robot_clone)
                            }))
                        } else {
                            // Our method: full-tree repair
                            panic::catch_unwind(panic::AssertUnwindSafe(|| {
                                agent.smart.update_static_obstacles(vc, Some(&mut rt), robot_clone)
                            }))
                        };

                        match inc_result {
                            Ok(SmartUpdateResult::ReplanningSuccessful) => {
                                if let Some(new_path) = agent.smart.get_path() {
                                    // Senior mode: validate path (path_only leaves dirty edges in tree)
                                    let path_dirty = args.senior_mode && new_path.windows(2).any(|pair| {
                                        cached_inflated.iter().any(|w| w.intersects_edge(&pair[0], &pair[1]))
                                    });
                                    if path_dirty {
                                        // Path crosses walls after path-only repair → full rebuild
                                        agent.rebuild_count += 1;
                                        eprintln!("    {} ⚠ senior path dirty, rebuild #{}", agent.name, agent.rebuild_count);
                                        if let Some((ns, np)) = build_smart(&agent.robot, &agent.goal, &known_walls,
                                                ranges, args.robot_speed, args.robot_radius, iters, args.goal_bias, args.shortcut_iters, args.lrz_time, args.lsr_max) {
                                            agent.smart = ns;
                                            agent.current_path = np;
                                            agent.path_idx = 1;
                                        }
                                    } else {
                                        let skip = forward_skip(&agent.robot, &agent.goal, &new_path, &cached_inflated);
                                        agent.current_path = new_path;
                                        agent.path_idx = skip;
                                        eprintln!("    {} ✓ incremental OK, skip={}", agent.name, skip);
                                    }
                                } else {
                                    // Path=None after repair, rebuild
                                    agent.rebuild_count += 1;
                                    eprintln!("    {} ⚠ path=None, rebuild #{}", agent.name, agent.rebuild_count);
                                    if let Some((ns, np)) = build_smart(&agent.robot, &agent.goal, &known_walls,
                                            ranges, args.robot_speed, args.robot_radius, iters, args.goal_bias, args.shortcut_iters, args.lrz_time, args.lsr_max) {
                                        agent.smart = ns;
                                        agent.current_path = np;
                                        agent.path_idx = 1;
                                    }
                                }
                            }
                            Ok(SmartUpdateResult::NoReplanningNeeded) => {
                                // Tree was not affected internally
                            }
                            _ => {
                                // Failed or panicked, rebuild
                                agent.rebuild_count += 1;
                                eprintln!("    {} ⚠ incremental failed, rebuild #{}", agent.name, agent.rebuild_count);
                                if let Some((ns, np)) = build_smart(&agent.robot, &agent.goal, &known_walls,
                                        ranges, args.robot_speed, args.robot_radius, iters, args.goal_bias, args.shortcut_iters, args.lrz_time, args.lsr_max) {
                                    agent.smart = ns;
                                    agent.current_path = np;
                                    agent.path_idx = 1;
                                }
                            }
                        }
                    }
                });
            }
        }

        // ---- Phase 3: Dynamic avoidance (agents + SHARED people observations) ----
        // Build shared visible people: union of all agents' observations this frame
        let shared_visible_people: Vec<SphericalObstacleWithOhz<f32, 2>> = {
            let mut seen = vec![false; all_people_ohz.len()];
            for agent in agents.iter() {
                if agent.reached_goal { continue; }
                let rv = agent.robot.values();
                for (pi, pobs) in all_people_ohz.iter().enumerate() {
                    if seen[pi] { continue; }
                    let oc = pobs.obstacle().center().values();
                    let d = ((rv[0] - oc[0]).powi(2) + (rv[1] - oc[1]).powi(2)).sqrt();
                    if d <= args.sensor_range + pobs.obstacle().radius()
                        && is_visible_through_walls(rv[0], rv[1], oc[0], oc[1], &known_walls) {
                        seen[pi] = true;
                    }
                }
            }
            all_people_ohz.iter().enumerate()
                .filter(|(pi, _)| seen[*pi])
                .map(|(_, p)| p.clone())
                .collect()
        };

        // Determine the ONE current global king: highest priority/tiebreak among active agents
        let king_idx: Option<usize> = (0..agents.len())
            .filter(|&i| !agents[i].reached_goal)
            .max_by(|&a, &b| {
                agents[a].priority.cmp(&agents[b].priority)
                    .then(agents[a].tiebreak.partial_cmp(&agents[b].tiebreak).unwrap_or(std::cmp::Ordering::Equal))
            });

        for i in 0..agents.len() {
            if agents[i].reached_goal { continue; }
            let is_king = king_idx == Some(i);

            // King ignores all other agents in SMART planning (plans straight through)
            // Non-king sees everyone normally
            let mut dyn_obs: Vec<SphericalObstacleWithOhz<f32, 2>> = Vec::new();
            if !is_king {
                for j in 0..agents.len() {
                    if i == j || agents[j].reached_goal { continue; }
                    let r = agents[j].robot.clone();
                    let obs = StaticSphericalObstacle::new(r.clone(), args.robot_radius);
                    let ohz = StaticSphericalObstacle::new(r, args.robot_radius + args.robot_speed * ohz_time);
                    dyn_obs.push(SphericalObstacleWithOhz::new(obs, ohz));
                }
            }

            // ALL shared visible people
            dyn_obs.extend(shared_visible_people.iter().cloned());

            if !dyn_obs.is_empty() {
                let robot_clone = agents[i].robot.clone();
                let pidx = agents[i].path_idx;
                let mut rt = MaxTimeTermination::new(Duration::from_secs_f32(dt * 0.5));

                let update_result = panic::catch_unwind(panic::AssertUnwindSafe(|| {
                    agents[i].smart.update(robot_clone, pidx, &dyn_obs, Some(&mut rt))
                }));

                match update_result {
                    Ok(SmartUpdateResult::ReplanningSuccessful) => {
                        if let Some(new_path) = agents[i].smart.get_path() {
                            agents[i].current_path = new_path;
                            agents[i].path_idx = 0;
                        }
                    }
                    Ok(_) => {}
                    Err(_) => {
                        agents[i].rebuild_count += 1;
                        eprintln!("    {} SMART panic, rebuild #{}", agents[i].name, agents[i].rebuild_count);
                        if let Some((ns, np)) = build_smart(&agents[i].robot, &agents[i].goal, &known_walls,
                                ranges, args.robot_speed, args.robot_radius, iters, args.goal_bias, args.shortcut_iters, args.lrz_time, args.lsr_max) {
                            agents[i].smart = ns;
                            agents[i].current_path = np;
                            agents[i].path_idx = 1;
                        }
                    }
                }
            }
        }

        // ---- Phase 4: King moves first, pushes everyone. Others flee king. ----
        let mut move_order: Vec<usize> = (0..agents.len()).collect();
        if let Some(ki) = king_idx {
            move_order.retain(|&x| x != ki);
            move_order.insert(0, ki);
        }

        for &i in &move_order {
            if agents[i].reached_goal { continue; }
            let is_king = king_idx == Some(i);

            let (new_robot, new_idx) = holonomic_move_along_path(
                &agents[i].robot, &agents[i].current_path, agents[i].path_idx,
                args.robot_speed * dt,
            );

            // Wall block check
            let wall_blocked = cached_inflated.iter().any(|w| {
                w.intersects_edge(&agents[i].robot, &new_robot)
            });

            // Person block check
            let collision_margin = args.robot_radius + obstacle_radius + 0.15;
            let rv_pre = agents[i].robot.values();
            let nv = new_robot.values();
            let people_blocked = shared_visible_people.iter().any(|obs| {
                let oc = obs.obstacle().center().values();
                let d_cur = ((rv_pre[0]-oc[0]).powi(2) + (rv_pre[1]-oc[1]).powi(2)).sqrt();
                let d_next = ((nv[0]-oc[0]).powi(2) + (nv[1]-oc[1]).powi(2)).sqrt();
                d_cur < collision_margin || d_next < collision_margin
            });

            // Agent blocking my next position
            let agent_blocker: Option<usize> = (0..agents.len()).find(|&j| {
                if i == j || agents[j].reached_goal { return false; }
                new_robot.euclidean_distance(&agents[j].robot) < args.robot_radius * 2.0
            });

            if people_blocked || wall_blocked {
                // Stop
            } else if is_king {
                if let Some(blocked_j) = agent_blocker {
                    // KING: chain push along movement direction
                    let push_dx = nv[0] - rv_pre[0];
                    let push_dy = nv[1] - rv_pre[1];
                    let push_dist = (push_dx*push_dx + push_dy*push_dy).sqrt();
                    if push_dist > 0.001 {
                        let dir = [push_dx / push_dist, push_dy / push_dist];
                        let push_step = args.robot_speed * dt;

                        // Build chain
                        let mut chain: Vec<usize> = vec![blocked_j];
                        for _depth in 0..8 {
                            let last = *chain.last().unwrap();
                            let lv = agents[last].robot.values();
                            let pushed = [lv[0] + dir[0] * push_step, lv[1] + dir[1] * push_step];
                            let next = (0..agents.len()).find(|&k| {
                                if k == i || chain.contains(&k) || agents[k].reached_goal { return false; }
                                let kv = agents[k].robot.values();
                                ((pushed[0]-kv[0]).powi(2) + (pushed[1]-kv[1]).powi(2)).sqrt() < args.robot_radius * 2.0
                            });
                            match next { Some(k) => chain.push(k), None => break }
                        }

                        // Can the end of chain move forward?
                        let last = *chain.last().unwrap();
                        let last_v = agents[last].robot.values();
                        let end_pos = RealVectorState::new([
                            (last_v[0] + dir[0] * push_step).clamp(1.5, ws - 1.5),
                            (last_v[1] + dir[1] * push_step).clamp(1.5, ws - 1.5),
                        ]);
                        let end_wall = cached_inflated.iter().any(|w| w.intersects_edge(&agents[last].robot, &end_pos));

                        if !end_wall {
                            for &cj in chain.iter().rev() {
                                let cv = agents[cj].robot.values();
                                agents[cj].robot = RealVectorState::new([
                                    (cv[0] + dir[0] * push_step).clamp(1.5, ws - 1.5),
                                    (cv[1] + dir[1] * push_step).clamp(1.5, ws - 1.5),
                                ]);
                                agents[cj].path_idx = 0;
                            }
                            agents[i].robot = new_robot;
                            agents[i].path_idx = new_idx;
                        } else {
                            // Can't push forward: scatter first blocker sideways
                            let bv = agents[blocked_j].robot.values();
                            let sdirs: Vec<[f32; 2]> = vec![
                                [-dir[1], dir[0]], [dir[1], -dir[0]], [-dir[0], -dir[1]],
                            ];
                            for sd in &sdirs {
                                let sp = RealVectorState::new([
                                    (bv[0] + sd[0] * push_step * 1.5).clamp(1.5, ws - 1.5),
                                    (bv[1] + sd[1] * push_step * 1.5).clamp(1.5, ws - 1.5),
                                ]);
                                let sw = cached_inflated.iter().any(|w| w.intersects_edge(&agents[blocked_j].robot, &sp));
                                let sa = (0..agents.len()).any(|k| {
                                    k != i && k != blocked_j && !agents[k].reached_goal
                                        && sp.euclidean_distance(&agents[k].robot) < args.robot_radius * 2.0
                                });
                                if !sw && !sa {
                                    agents[blocked_j].robot = sp;
                                    agents[blocked_j].path_idx = 0;
                                    if new_robot.euclidean_distance(&agents[blocked_j].robot) >= args.robot_radius * 2.0 {
                                        agents[i].robot = new_robot;
                                        agents[i].path_idx = new_idx;
                                    }
                                    break;
                                }
                            }
                        }
                    }
                } else {
                    // King, no blocker: move freely
                    agents[i].robot = new_robot;
                    agents[i].path_idx = new_idx;
                }
            } else {
                // NON-KING: actively flee from king if king is nearby and heading toward me
                let flee_range = 5.0f32;
                let should_flee = if let Some(ki) = king_idx {
                    let d = agents[i].robot.euclidean_distance(&agents[ki].robot);
                    d < flee_range && !agents[ki].reached_goal
                } else { false };

                if should_flee {
                    let ki = king_idx.unwrap();
                    let rv = agents[i].robot.values();
                    let kv = agents[ki].robot.values();
                    let away = [rv[0] - kv[0], rv[1] - kv[1]];
                    let d = (away[0]*away[0] + away[1]*away[1]).sqrt().max(0.01);
                    let step = args.robot_speed * dt;
                    // Try: away from king, perpendicular left/right
                    let dirs: Vec<[f32; 2]> = vec![
                        [away[0]/d, away[1]/d],
                        [-away[1]/d, away[0]/d],
                        [away[1]/d, -away[0]/d],
                    ];
                    let mut fled = false;
                    for dir in &dirs {
                        let rp = RealVectorState::new([
                            (rv[0] + dir[0] * step).clamp(1.5, ws - 1.5),
                            (rv[1] + dir[1] * step).clamp(1.5, ws - 1.5),
                        ]);
                        let wall_hit = cached_inflated.iter().any(|w| w.intersects_edge(&agents[i].robot, &rp));
                        let agent_hit = (0..agents.len()).any(|k| {
                            k != i && !agents[k].reached_goal && rp.euclidean_distance(&agents[k].robot) < args.robot_radius * 2.0
                        });
                        if !wall_hit && !agent_hit {
                            agents[i].robot = rp;
                            agents[i].path_idx = 0;
                            fled = true;
                            break;
                        }
                    }
                    if !fled {
                        // Can't flee, just stay
                    }
                } else if agent_blocker.is_none() {
                    // Not near king, no collision: follow SMART path normally
                    agents[i].robot = new_robot;
                    agents[i].path_idx = new_idx;
                }
                // else: blocked by another non-king agent, just stop
            }

            // Goal detection
            if agents[i].robot.euclidean_distance(&agents[i].goal) <= 1.5 {
                if !agents[i].reached_goal {
                    agents[i].reached_goal = true;
                    let rv = agents[i].robot.values();
                    eprintln!("  ✓ {} GOAL REACHED at ({:.1},{:.1}), t={:.1}s, rebuilds={}{}",
                        agents[i].name, rv[0], rv[1], sim_time, agents[i].rebuild_count,
                        if is_king { " 👑" } else { "" });
                }
            }

            // Progress tracking
            if agents[i].robot.euclidean_distance(&agents[i].last_progress_pos) > 2.0 {
                agents[i].last_progress_pos = agents[i].robot.clone();
                agents[i].frames_since_progress = 0;
            } else {
                agents[i].frames_since_progress += 1;
            }

            // Stuck: try rebuild every 40 frames of no progress
            // Don't reset counter on failure — let it accumulate to 600 for UNREACHABLE
            let stuck_threshold: u64 = 40;
            if agents[i].frames_since_progress > stuck_threshold
                && agents[i].frames_since_progress % stuck_threshold == 1
                && !agents[i].reached_goal
            {
                agents[i].rebuild_count += 1;
                if is_king {
                    // King: rebuild ignoring all other agents
                    eprintln!("    {} 👑 king stuck, rebuild #{} (ignoring others)",
                        agents[i].name, agents[i].rebuild_count);
                    if let Some((ns, np)) = build_smart(&agents[i].robot, &agents[i].goal, &known_walls,
                            ranges, args.robot_speed, args.robot_radius, iters,
                            args.goal_bias, args.shortcut_iters, args.lrz_time, args.lsr_max) {
                        agents[i].smart = ns;
                        agents[i].current_path = np;
                        agents[i].path_idx = 1;
                        agents[i].frames_since_progress = 0;
                        agents[i].last_progress_pos = agents[i].robot.clone();
                    }
                } else {
                    // Non-king: rebuild with ALL higher-priority agents as walls
                    // This creates a full priority chain: king > 2nd > 3rd > 4th
                    // Previously only yielded to king, so two non-kings could deadlock
                    let my_tb = agents[i].tiebreak;
                    let my_pri = agents[i].priority;
                    let mut walls_plus = known_walls.clone();
                    let mut yielded_to: Vec<String> = Vec::new();
                    for j in 0..agents.len() {
                        if i == j || agents[j].reached_goal { continue; }
                        let dominated = agents[j].priority > my_pri
                            || (agents[j].priority == my_pri && agents[j].tiebreak > my_tb);
                        if dominated {
                            let jv = agents[j].robot.values();
                            walls_plus.push(StaticRectangularObstacle::new(
                                RealVectorState::new([jv[0] - 3.0, jv[1] - 3.0]),
                                RealVectorState::new([jv[0] + 3.0, jv[1] + 3.0]),
                            ));
                            yielded_to.push(agents[j].name.clone());
                        }
                    }
                    if yielded_to.is_empty() {
                        eprintln!("    {} stuck (lowest priority), rebuild #{}",
                            agents[i].name, agents[i].rebuild_count);
                    } else {
                        eprintln!("    {} yields to [{}], rebuild #{}",
                            agents[i].name, yielded_to.join(","), agents[i].rebuild_count);
                    }
                    match build_smart(&agents[i].robot, &agents[i].goal, &walls_plus,
                            ranges, args.robot_speed, args.robot_radius, iters,
                            args.goal_bias, args.shortcut_iters, args.lrz_time, args.lsr_max) {
                        Some((ns, np)) => {
                            agents[i].smart = ns;
                            agents[i].current_path = np;
                            agents[i].path_idx = 1;
                            agents[i].frames_since_progress = 0;
                            agents[i].last_progress_pos = agents[i].robot.clone();
                        }
                        None => {} // no route, just wait (don't reset progress counter)
                    }
                }
            }

            // Unreachable timeout
            if agents[i].frames_since_progress > 600 && !agents[i].reached_goal {
                let rv = agents[i].robot.values();
                eprintln!("  ❌ {} UNREACHABLE at ({:.1},{:.1}), marking done", agents[i].name, rv[0], rv[1]);
                agents[i].reached_goal = true; // give up
            }
        }

        // ---- Phase 4.5: Post-move overlap resolution ----
        // Separate any agents that ended up overlapping after pushes/movement
        for _pass in 0..3 {
            for i in 0..agents.len() {
                if agents[i].reached_goal { continue; }
                for j in (i+1)..agents.len() {
                    if agents[j].reached_goal { continue; }
                    let d = agents[i].robot.euclidean_distance(&agents[j].robot);
                    let min_d = args.robot_radius * 2.0;
                    if d < min_d && d > 0.01 {
                        let iv = agents[i].robot.values();
                        let jv = agents[j].robot.values();
                        let dx = jv[0] - iv[0];
                        let dy = jv[1] - iv[1];
                        let overlap = (min_d - d) * 0.55; // push apart slightly more than half

                        let i_is_king = king_idx == Some(i);
                        let j_is_king = king_idx == Some(j);

                        if i_is_king {
                            // Push j away
                            let new_j = RealVectorState::new([
                                (jv[0] + dx / d * overlap).clamp(1.5, ws - 1.5),
                                (jv[1] + dy / d * overlap).clamp(1.5, ws - 1.5),
                            ]);
                            let wall_hit = cached_inflated.iter().any(|w| w.intersects_edge(&agents[j].robot, &new_j));
                            if !wall_hit { agents[j].robot = new_j; agents[j].path_idx = 0; }
                        } else if j_is_king {
                            // Push i away
                            let new_i = RealVectorState::new([
                                (iv[0] - dx / d * overlap).clamp(1.5, ws - 1.5),
                                (iv[1] - dy / d * overlap).clamp(1.5, ws - 1.5),
                            ]);
                            let wall_hit = cached_inflated.iter().any(|w| w.intersects_edge(&agents[i].robot, &new_i));
                            if !wall_hit { agents[i].robot = new_i; agents[i].path_idx = 0; }
                        } else {
                            // Both non-king: push both apart equally
                            let half = overlap * 0.5;
                            let new_i = RealVectorState::new([
                                (iv[0] - dx / d * half).clamp(1.5, ws - 1.5),
                                (iv[1] - dy / d * half).clamp(1.5, ws - 1.5),
                            ]);
                            let new_j = RealVectorState::new([
                                (jv[0] + dx / d * half).clamp(1.5, ws - 1.5),
                                (jv[1] + dy / d * half).clamp(1.5, ws - 1.5),
                            ]);
                            let wi = cached_inflated.iter().any(|w| w.intersects_edge(&agents[i].robot, &new_i));
                            let wj = cached_inflated.iter().any(|w| w.intersects_edge(&agents[j].robot, &new_j));
                            if !wi { agents[i].robot = new_i; agents[i].path_idx = 0; }
                            if !wj { agents[j].robot = new_j; agents[j].path_idx = 0; }
                        }
                    }
                }
            }
        }

        // ---- Phase 5: JSON output ----
        let mut agents_json = JsonValue::new_array();
        for a in &agents {
            let rv = a.robot.values();
            let gv = a.goal.values();

            let mut path_json = JsonValue::new_array();
            path_json.push(json::array![rv[0], rv[1]]).unwrap();
            let slice_start = a.path_idx.min(a.current_path.len().saturating_sub(1));
            for pt in &a.current_path[slice_start..] {
                let pv = pt.values();
                path_json.push(json::array![pv[0], pv[1]]).unwrap();
            }

            agents_json.push(object! {
                "name": a.name.as_str(),
                "robot": [rv[0], rv[1]],
                "goal": [gv[0], gv[1]],
                "path": path_json,
                "path_index": a.path_idx,
                "reached_goal": a.reached_goal,
                "rebuilds": a.rebuild_count,
                "color": [a.color[0], a.color[1], a.color[2]],
            }).unwrap();
        }

        // Optional tree edges (every 5 frames, only first agent to reduce bandwidth)
        let mut tree_json = JsonValue::Null;
        if args.include_tree && frame % 10 == 0 {
            let mut all_edges = JsonValue::new_array();
            for (ai, a) in agents.iter().enumerate() {
                if a.reached_goal { continue; }
                let sn = a.smart.get_nodes();
                for node in sn.iter() {
                    if let Some(pi) = node.parent {
                        let pn = sn[pi].state.values();
                        let cn = node.state.values();
                        all_edges.push(json::array![pn[0], pn[1], cn[0], cn[1], node.tree_id, ai]).unwrap();
                    }
                }
            }
            tree_json = all_edges;
        }

        let grid_str = if frame % 5 == 0 {
            JsonValue::from(grid_to_string(&scanned_grid))
        } else {
            JsonValue::Null
        };

        // People obstacles JSON
        let mut obs_json = JsonValue::new_array();
        for pobs in &all_people_ohz {
            let oc = pobs.obstacle().center().values();
            obs_json.push(object! {
                "x": oc[0], "y": oc[1],
                "radius": pobs.obstacle().radius(),
                "ohz_radius": pobs.ohz().radius(),
            }).unwrap();
        }

        let active_count = agents.iter().filter(|a| !a.reached_goal).count();
        let fj = object! {
            "type": "frame", "frame": frame, "time": sim_time,
            "walls_known": known_walls.len() as u32,
            "walls_total": scenario.walls.len() as u32,
            "active_agents": active_count as u32,
            "sensor_range": args.sensor_range,
            "robot_radius": args.robot_radius,
            "agents": agents_json,
            "obstacles": obs_json,
            "tree": tree_json,
            "grid": grid_str,
        };
        if writeln!(out, "{}", fj.dump()).is_err() { return; }
        out.flush().unwrap_or(());

        frame += 1;

        // Termination
        if agents.iter().all(|a| a.reached_goal) {
            eprintln!("\n  ✓ ALL AGENTS DONE at t={:.1}s, frame={}", sim_time, frame);
            break;
        }
        if sim_time >= args.max_time {
            eprintln!("\n  ⏰ TIMEOUT at {:.0}s", args.max_time);
            for a in &agents {
                if !a.reached_goal {
                    let rv = a.robot.values();
                    let d = a.robot.euclidean_distance(&a.goal);
                    eprintln!("    {} at ({:.1},{:.1}), dist_to_goal={:.1}m, rebuilds={}", a.name, rv[0], rv[1], d, a.rebuild_count);
                }
            }
            break;
        }

        let elapsed = t0.elapsed();
        if elapsed < delay { std::thread::sleep(delay - elapsed); }
    }

    // Summary
    eprintln!("\n{}", "=".repeat(60));
    eprintln!("  SUMMARY: {} | {} agents | mode={}", scenario.name, agents.len(), args.agent_mode);
    eprintln!("{}", "=".repeat(60));
    for a in &agents {
        let rv = a.robot.values();
        let d = a.robot.euclidean_distance(&a.goal);
        let status = if a.reached_goal && d <= 1.5 { "GOAL" } else { "FAIL" };
        eprintln!("  {} [{}] pos=({:.1},{:.1}) dist={:.1}m rebuilds={}",
            a.name, status, rv[0], rv[1], d, a.rebuild_count);
    }

    let ej = object! { "type": "end", "scenario": scenario.name.as_str(),
        "total_frames": frame, "total_time": sim_time,
        "agents_reached": agents.iter().filter(|a| a.reached_goal && a.robot.euclidean_distance(&a.goal) <= 1.5).count() as u32,
        "agents_total": agents.len() as u32,
    };
    let _ = writeln!(out, "{}", ej.dump());
}


fn main() {
    panic::set_hook(Box::new(|_| {}));
    let args = CliArgs::parse();
    let stdout = io::stdout();
    let mut out = stdout.lock();

    let scenario = make_scenario(&args.scenario);
    eprintln!("=== SMART-3D Multi-Agent Simulation ===");
    run_multi_agent(&scenario, &args, &mut out);
    eprintln!("=== Done ===");
}
