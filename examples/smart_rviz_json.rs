/// SMART-3D → RViz2 Bridge v3.1 — LiDAR Discovery + Wall Inflation + 3D
///
/// Core idea: Robot knows the STATIC walls (building floor plan) but does NOT
/// know where dynamic obstacles (people) are. As the robot moves, its LiDAR
/// sensor detects people within range. When a new obstacle enters the sensor
/// cone and threatens the planned path, SMART prunes the RRT* tree and
/// instantly repairs the route — this is the algorithm's key strength.
///
/// Usage:
///   cargo run --release --example smart_rviz_json -- --scenario building --include-tree
///   cargo run --release --example smart_rviz_json -- --scenario all -n 15 --sensor-range 8
use clap::Parser;
use json::{object, JsonValue};
use smart_3d::obstacles::random_dynamic::RandomDirectionCircle;
use smart_3d::obstacles::SphericalObstacle;
use smart_3d::obstacles::{
    AnalyticValidityChecker, DynamicSphericalObstacle, RectangularObstacle,
    StaticRectangularObstacle, StaticSphericalObstacle,
};
use smart_3d::rrt::KdTreeNearestNeighbors;
use smart_3d::rrt::{
    rrt_star::optimal_gamma,
    termination::{MaxIterationsTermination, MaxTimeTermination},
    validity_checker::AlwaysValid,
    RRTstar, RealVectorState, SamplingDistribution, UniformDistribution, ValidityChecker,
};
use smart_3d::smart::{SmartUpdateResult, SMART};
use rand::Rng;
use std::io::{self, Write};
use std::panic;
use std::thread;
use std::time::{Duration, Instant};

mod scenarios;
use scenarios::scenario_utils::{holonomic_move_along_path, make_spherical_obstacles_with_ohzs};

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
            for v in vals.iter_mut() { *v = *v + (rng.gen::<f64>() * 1.0 - 0.5) as f32; }
            RealVectorState::new(vals)
        } else { self.uniform.sample() }
    }
}

#[derive(Parser, Debug)]
#[command(version, about = "SMART-3D RViz2 v3.1 — LiDAR + WallInflation + 3D")]
struct CliArgs {
    /// Scenario: open, office, building, warehouse, all
    #[arg(short, long, default_value = "building")]
    scenario: String,

    /// Number of dynamic obstacles (people)
    #[arg(short, long, default_value = "12")]
    num_obstacles: usize,

    /// Obstacle speed (m/s)
    #[arg(short, long, default_value = "1.5")]
    obstacle_speed: f32,

    /// Robot speed (m/s)
    #[arg(long, default_value = "3.0")]
    robot_speed: f32,

    /// Simulation dt (seconds) — smaller = smoother
    #[arg(long, default_value = "0.05")]
    dt: f32,

    /// Playback speed (1.0=realtime, 3.0=3x, 5.0=5x)
    #[arg(long, default_value = "3.0")]
    playback_speed: f32,

    /// Include RRT* tree edges in output
    #[arg(long)]
    include_tree: bool,

    /// LiDAR sensor range in meters
    #[arg(long, default_value = "10.0")]
    sensor_range: f32,

    /// Robot body radius (m) — walls inflated by this amount
    #[arg(long, default_value = "0.5")]
    robot_radius: f32,

    /// RRT* iterations for initial tree
    #[arg(long, default_value = "5000")]
    rrt_iterations: usize,

    /// Pause between scenarios in 'all' mode (seconds)
    #[arg(long, default_value = "2.0")]
    pause_between: f32,

    /// Goal-bias sampling probability for RRT* (0=off, 0.10~0.15 recommended)
    #[arg(long, default_value = "0.0")]
    goal_bias: f64,

    /// Path shortcut iterations after build/rebuild (0=off)
    #[arg(long, default_value = "150")]
    shortcut_iters: usize,

    /// Detour detection threshold (0=off, 1.5=aggressive, 1.8=default, 2.5=conservative)
    #[arg(long, default_value = "1.5")]
    detour_threshold: f32,

    /// LRZ time range (seconds). LRZ radius = speed * time. Use 1.0 for 32m maps, 3.0~5.0 for mega maps.
    #[arg(long, default_value = "1.0")]
    lrz_time: f32,

    /// LSR max radius (meters). Hot-node repair search range. Larger = better repair, slower per-frame.
    #[arg(long, default_value = "30.0")]
    lsr_max: f32,

    /// Senior baseline mode: path-only repair, no checker swap, no shortcut, no detour.
    /// Replicates the original SMART-3D author's approach for fair comparison.
    #[arg(long, default_value = "false")]
    senior_mode: bool,
}

// ==================== Wall helpers ====================
fn wall(x1: f32, y1: f32, x2: f32, y2: f32) -> StaticRectangularObstacle<f32, 2> {
    StaticRectangularObstacle::new(
        RealVectorState::new([x1.min(x2), y1.min(y2)]),
        RealVectorState::new([x1.max(x2), y1.max(y2)]),
    )
}

fn point_to_rect_dist(px: f32, py: f32, w: &StaticRectangularObstacle<f32, 2>) -> f32 {
    let min = w.min_corner().values();
    let max = w.max_corner().values();
    let cx = px.clamp(min[0], max[0]);
    let cy = py.clamp(min[1], max[1]);
    ((px - cx).powi(2) + (py - cy).powi(2)).sqrt()
}

// ==================== LiDAR ray-casting ====================
const GRID_RES: f32 = 0.5;
const NUM_RAYS: usize = 180;

/// Ray-rectangle intersection: returns distance t along ray (ox+t*dx, oy+t*dy)
fn ray_rect_hit(
    ox: f32, oy: f32, dx: f32, dy: f32,
    x1: f32, y1: f32, x2: f32, y2: f32,
) -> Option<f32> {
    let mut best = f32::MAX;
    // Left edge x=x1
    if dx.abs() > 1e-8 {
        let t = (x1 - ox) / dx;
        if t > 0.001 { let y = oy + t * dy; if y >= y1 && y <= y2 && t < best { best = t; } }
    }
    // Right edge x=x2
    if dx.abs() > 1e-8 {
        let t = (x2 - ox) / dx;
        if t > 0.001 { let y = oy + t * dy; if y >= y1 && y <= y2 && t < best { best = t; } }
    }
    // Bottom edge y=y1
    if dy.abs() > 1e-8 {
        let t = (y1 - oy) / dy;
        if t > 0.001 { let x = ox + t * dx; if x >= x1 && x <= x2 && t < best { best = t; } }
    }
    // Top edge y=y2
    if dy.abs() > 1e-8 {
        let t = (y2 - oy) / dy;
        if t > 0.001 { let x = ox + t * dx; if x >= x1 && x <= x2 && t < best { best = t; } }
    }
    if best < f32::MAX { Some(best) } else { None }
}

/// Check if line of sight from (rx,ry) to (tx,ty) is blocked by any known wall.
/// Returns true if the target is visible (no wall blocks the ray).
fn is_visible_through_walls(
    rx: f32, ry: f32,
    tx: f32, ty: f32,
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
            if t < dist {
                return false; // Wall blocks line of sight
            }
        }
    }
    true
}

/// Cast LiDAR rays, update scanned_grid and discover walls.
/// Returns true if any new wall was discovered.
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

        // Find closest TRUE wall hit along this ray
        let mut hit_t = sensor_range;
        let mut hit_wall: Option<usize> = None;

        for (wi, w) in true_walls.iter().enumerate() {
            let mn = w.min_corner().values();
            let mx = w.max_corner().values();
            if let Some(t) = ray_rect_hit(rx, ry, dx, dy, mn[0], mn[1], mx[0], mx[1]) {
                if t < hit_t {
                    hit_t = t;
                    hit_wall = Some(wi);
                }
            }
        }

        // Mark cells along ray as FREE (up to hit point)
        let step = GRID_RES * 0.4; // step smaller than cell size for good coverage
        let steps = (hit_t / step) as usize;
        for s in 0..=steps {
            let t = (s as f32) * step;
            if t > hit_t { break; }
            let gx = ((rx + t * dx) / GRID_RES) as i32;
            let gy = ((ry + t * dy) / GRID_RES) as i32;
            if gx >= 0 && gy >= 0 && (gx as usize) < grid_w && (gy as usize) < grid_h {
                let idx = (gy as usize) * grid_w + (gx as usize);
                if grid[idx] < 0 { grid[idx] = 0; } // unknown → free
            }
        }

        // Mark hit cell as WALL and discover the wall
        if let Some(wi) = hit_wall {
            let hx = rx + hit_t * dx;
            let hy = ry + hit_t * dy;
            let gx = (hx / GRID_RES) as i32;
            let gy = (hy / GRID_RES) as i32;
            if gx >= 0 && gy >= 0 && (gx as usize) < grid_w && (gy as usize) < grid_h {
                grid[(gy as usize) * grid_w + (gx as usize)] = 1; // wall
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

/// Encode grid as compact string: 'U'=unknown, 'F'=free, 'W'=wall
fn grid_to_string(grid: &[i8]) -> String {
    grid.iter().map(|&v| match v {
        0 => 'F',
        1 => 'W',
        _ => 'U',
    }).collect()
}

// ==================== Scenario definitions ====================
struct ScenarioDef {
    name: String,
    world_size: f32,
    start: [f32; 2],
    goal: [f32; 2],
    walls: Vec<StaticRectangularObstacle<f32, 2>>,
    obstacle_spawn_ranges: [(f32, f32); 2],
}

/// "office" — 3x3 grid of rooms with corridors and furniture
fn make_office(ws: f32) -> ScenarioDef {
    let t = 0.3;
    let door = 2.5;
    let mut w = Vec::new();

    // Outer walls
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));

    // Vertical partitions at x=10.5 and x=21.5
    w.push(wall(10.5, t, 10.5 + t, 3.5));
    w.push(wall(10.5, 3.5 + door, 10.5 + t, 14.5));
    w.push(wall(10.5, 14.5 + door, 10.5 + t, 25.5));
    w.push(wall(10.5, 25.5 + door, 10.5 + t, ws - t));

    w.push(wall(21.5, t, 21.5 + t, 6.5));
    w.push(wall(21.5, 6.5 + door, 21.5 + t, 18.5));
    w.push(wall(21.5, 18.5 + door, 21.5 + t, 26.5));
    w.push(wall(21.5, 26.5 + door, 21.5 + t, ws - t));

    // Horizontal partitions at y=10.5 and y=21.5
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

    // Furniture
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

    ScenarioDef {
        name: "office".into(),
        world_size: ws,
        start: [3.0, 7.0],
        goal: [ws - 2.0, ws - 2.0],
        walls: w,
        obstacle_spawn_ranges: [(2.0, ws - 2.0), (2.0, ws - 2.0)],
    }
}

/// "building" — large building with main corridor + wings + lab rooms
fn make_building(ws: f32) -> ScenarioDef {
    let t = 0.3;
    let door = 2.5;
    let mut w = Vec::new();

    // Outer walls
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));

    // Main corridor y=14.5→16.8 (upper + lower boundary with doors)
    w.push(wall(t, 16.8, 7.0, 16.8 + t));
    w.push(wall(7.0 + door, 16.8, 16.0, 16.8 + t));
    w.push(wall(16.0 + door, 16.8, 25.0, 16.8 + t));
    w.push(wall(25.0 + door, 16.8, ws - t, 16.8 + t));
    w.push(wall(t, 14.5, 7.0, 14.5 + t));
    w.push(wall(7.0 + door, 14.5, 16.0, 14.5 + t));
    w.push(wall(16.0 + door, 14.5, 25.0, 14.5 + t));
    w.push(wall(25.0 + door, 14.5, ws - t, 14.5 + t));

    // South wing rooms
    w.push(wall(10.5, t, 10.5 + t, 5.5));
    w.push(wall(10.5, 5.5 + door, 10.5 + t, 14.5));
    w.push(wall(21.5, t, 21.5 + t, 8.5));
    w.push(wall(21.5, 8.5 + door, 21.5 + t, 14.5));
    w.push(wall(21.5 + t, 7.5, 24.5, 7.5 + t));
    w.push(wall(24.5 + door, 7.5, ws - t, 7.5 + t));

    // North wing rooms
    w.push(wall(8.5, 16.8 + t, 8.5 + t, 22.5));
    w.push(wall(8.5, 22.5 + door, 8.5 + t, ws - t));
    w.push(wall(20.0, 16.8 + t, 20.0 + t, 20.5));
    w.push(wall(20.0, 20.5 + door, 20.0 + t, ws - t));
    w.push(wall(t, 25.0, 8.5, 25.0 + t));
    w.push(wall(20.0 + t, 25.0, 25.5, 25.0 + t));
    w.push(wall(25.5 + door, 25.0, ws - t, 25.0 + t));

    // Furniture
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

    ScenarioDef {
        name: "building".into(),
        world_size: ws,
        start: [2.0, 2.0],
        goal: [ws - 2.0, ws - 2.0],
        walls: w,
        obstacle_spawn_ranges: [(2.0, ws - 2.0), (2.0, ws - 2.0)],
    }
}

/// "warehouse" — shelf racks with aisles
fn make_warehouse(ws: f32) -> ScenarioDef {
    let t = 0.3;
    let mut w = Vec::new();

    // Outer walls
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));

    // Shelf racks — vertical rows with gaps for crossing
    for &rx in &[5.0f32, 10.0, 15.0, 20.0, 25.0] {
        w.push(wall(rx, 2.0, rx + 0.8, 6.5));
        w.push(wall(rx, 9.5, rx + 0.8, 14.5));
        w.push(wall(rx, 17.5, rx + 0.8, 22.5));
        w.push(wall(rx, 25.5, rx + 0.8, 30.0));
    }

    // Cross-aisle barriers
    w.push(wall(6.0, 12.0, 14.5, 12.0 + t));
    w.push(wall(16.0, 20.0, 24.5, 20.0 + t));

    // Office corner top-right
    w.push(wall(27.0, 27.0, ws - t, 27.0 + t));
    w.push(wall(27.0, 27.0 + t, 27.0 + t, 28.5));
    w.push(wall(27.0, 31.0, 27.0 + t, ws - t));

    ScenarioDef {
        name: "warehouse".into(),
        world_size: ws,
        start: [2.0, 15.5],
        goal: [ws - 2.0, ws - 2.0],
        walls: w,
        obstacle_spawn_ranges: [(2.0, ws - 2.0), (2.0, ws - 2.0)],
    }
}

fn make_open(ws: f32) -> ScenarioDef {
    ScenarioDef {
        name: "open".into(),
        world_size: ws,
        start: [2.0, 2.0],
        goal: [ws - 2.0, ws - 2.0],
        walls: vec![],
        obstacle_spawn_ranges: [(4.0, ws - 4.0), (4.0, ws - 4.0)],
    }
}


/// "megacampus" — 128×128m campus with 200+ walls, corridors, labs, atrium
fn make_megacampus(ws: f32) -> ScenarioDef {
    let t = 0.3;
    let door = 2.5;
    let mut w = Vec::new();

    // ===== Outer boundary =====
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));

    // ===== Main horizontal corridors (y=32, y=64, y=96) =====
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

    // ===== Main vertical corridors (x=32, x=64, x=96) =====
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

    // ===== Quadrant rooms (16 quadrants, each ~30x30m) =====
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

    // ===== Central atrium (open space with pillars) =====
    for &px in &[48.0f32, 56.0, 64.0, 72.0, 80.0] {
        for &py in &[48.0f32, 56.0, 64.0, 72.0, 80.0] {
            w.push(wall(px, py, px + 1.0, py + 1.0));
        }
    }

    // ===== Stairwells (4 corners) =====
    for &(sx, sy) in &[(2.0, 2.0), (2.0, 120.0), (120.0, 2.0), (120.0, 120.0)] {
        w.push(wall(sx, sy, sx + 6.0, sy + t));
        w.push(wall(sx, sy, sx + t, sy + 6.0));
        w.push(wall(sx + 6.0, sy, sx + 6.0 + t, sy + 3.0));
        w.push(wall(sx, sy + 6.0, sx + 3.0, sy + 6.0 + t));
    }

    // ===== Long lab benches (scattered) =====
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

    ScenarioDef {
        name: "megacampus".into(),
        world_size: ws,
        start: [8.0, 8.0],
        goal: [ws - 4.0, ws - 4.0],
        walls: w,
        obstacle_spawn_ranges: [(4.0, ws - 4.0), (4.0, ws - 4.0)],
    }
}


/// "mega_hospital" — 96×96m hospital: wings, patient rooms, OR suites, ER bays
fn make_mega_hospital(ws: f32) -> ScenarioDef {
    let t = 0.3;    // wall thickness
    let door = 2.0; // door opening width
    let mut w = Vec::new();

    // ===== Outer boundary =====
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));

    // ===== Central E-W spine corridor: y=44..49 (5m wide) =====
    // South wall of spine (y=44) with doors every 10m
    {
        let mut x = t;
        while x < ws - t {
            let seg_end = (x + 8.0).min(ws - t);
            w.push(wall(x, 44.0, seg_end, 44.0 + t));
            x = seg_end + door;
        }
    }
    // North wall of spine (y=49) with doors every 10m
    {
        let mut x = t;
        while x < ws - t {
            let seg_end = (x + 8.0).min(ws - t);
            w.push(wall(x, 49.0, seg_end, 49.0 + t));
            x = seg_end + door;
        }
    }

    // ===== N-S corridors at x=24, x=48, x=72 (3m wide) =====
    for &cx in &[24.0f32, 48.0, 72.0] {
        // West wall of corridor
        let mut y = t;
        while y < ws - t {
            // Skip through spine corridor zone (y=44..49)
            if y >= 43.0 && y <= 50.0 { y = 50.5; continue; }
            let seg_end = (y + 6.0).min(ws - t);
            if seg_end > 43.0 && y < 43.0 { 
                w.push(wall(cx, y, cx + t, 43.0));
                y = 50.0;
                continue;
            }
            w.push(wall(cx, y, cx + t, seg_end));
            y = seg_end + door;
        }
        // East wall of corridor (cx+3)
        let mut y = t;
        while y < ws - t {
            if y >= 43.0 && y <= 50.0 { y = 50.5; continue; }
            let seg_end = (y + 6.0).min(ws - t);
            if seg_end > 43.0 && y < 43.0 {
                w.push(wall(cx + 3.0, y, cx + 3.0 + t, 43.0));
                y = 50.0;
                continue;
            }
            w.push(wall(cx + 3.0, y, cx + 3.0 + t, seg_end));
            y = seg_end + door;
        }
    }

    // ===== NORTH WING: Patient rooms (y=51..94) =====
    // Three zones between N-S corridors: x=[1..24], [27..48], [51..72], [75..95]
    let north_zones = [(1.5, 23.5), (27.5, 47.5), (51.5, 71.5), (75.5, 94.5)];
    for &(zx_start, zx_end) in &north_zones {
        let zone_w = zx_end - zx_start;
        if zone_w < 8.0 { continue; }
        // Horizontal partitions creating room rows at y=58, y=68, y=78, y=88
        for &ry in &[58.0f32, 68.0, 78.0, 88.0] {
            if ry + t > ws - 2.0 { continue; }
            let mut x = zx_start;
            while x < zx_end {
                let seg_end = (x + 5.0f32).min(zx_end);
                w.push(wall(x, ry, seg_end, ry + t));
                x = seg_end + door;
            }
        }
        // Vertical partitions creating individual rooms (every 5m)
        let mut rx = zx_start + 5.0;
        while rx < zx_end - 2.0 {
            // Rooms between each horizontal pair
            for &(ry_lo, ry_hi) in &[(51.0, 57.5), (58.5, 67.5), (68.5, 77.5), (78.5, 87.5), (88.5, 94.5)] {
                if ry_hi > ws - 2.0 { continue; }
                let wall_hi = ry_hi.min(ws - 1.5);
                // Wall with door gap in middle
                let mid = (ry_lo + wall_hi) / 2.0;
                if mid - 1.0 > ry_lo {
                    w.push(wall(rx, ry_lo, rx + t, mid - 1.0));
                }
                if mid + 1.0 < wall_hi {
                    w.push(wall(rx, mid + 1.0, rx + t, wall_hi));
                }
            }
            rx += 5.0;
        }
    }

    // ===== SOUTH WING: ER + OR suites (y=2..43) =====

    // -- ER department (x=2..46, y=2..25): open trauma bays --
    // Bay dividers (partial walls, 2m tall conceptually but 2D so just short walls)
    for &bx in &[10.0f32, 18.0, 26.0, 34.0, 42.0] {
        if bx + t > 46.0 { continue; }
        w.push(wall(bx, 2.5, bx + t, 10.0));   // bay divider south row
        w.push(wall(bx, 14.0, bx + t, 22.0));   // bay divider north row
    }
    // ER central aisle wall
    w.push(wall(2.0, 11.5, 20.0, 11.5 + t));
    w.push(wall(22.0, 11.5, 46.0, 11.5 + t));
    // Triage room
    w.push(wall(2.0, 23.0, 15.0, 23.0 + t));
    w.push(wall(17.0, 23.0, 23.0, 23.0 + t));
    w.push(wall(23.0, 15.0, 23.0 + t, 23.0));

    // -- OR suites (x=50..94, y=2..35): dead-end corridors + rooms --
    // OR access corridor (y=28..31, x=50..94)
    w.push(wall(50.0, 28.0, 60.0, 28.0 + t));
    w.push(wall(62.0, 28.0, 75.0, 28.0 + t));
    w.push(wall(77.0, 28.0, 94.0, 28.0 + t));
    w.push(wall(50.0, 31.0, 65.0, 31.0 + t));
    w.push(wall(67.0, 31.0, 80.0, 31.0 + t));
    w.push(wall(82.0, 31.0, 94.0, 31.0 + t));

    // Individual OR rooms south of corridor (y=2..28)
    for &orx in &[54.0f32, 62.0, 70.0, 78.0, 86.0] {
        if orx + t > 94.0 { continue; }
        w.push(wall(orx, 2.5, orx + t, 12.0));
        w.push(wall(orx, 14.0, orx + t, 27.5));
    }
    // OR horizontal divider
    w.push(wall(50.5, 13.0, 65.0, 13.0 + t));
    w.push(wall(67.0, 13.0, 80.0, 13.0 + t));
    w.push(wall(82.0, 13.0, 93.5, 13.0 + t));

    // -- Radiology wing (x=50..94, y=33..43): L-shaped thick-wall rooms --
    w.push(wall(50.0, 36.0, 62.0, 36.0 + t));
    w.push(wall(64.0, 36.0, 78.0, 36.0 + t));
    w.push(wall(80.0, 36.0, 94.0, 36.0 + t));
    w.push(wall(50.0, 40.0, 58.0, 40.0 + t));
    w.push(wall(60.0, 40.0, 75.0, 40.0 + t));
    w.push(wall(77.0, 40.0, 94.0, 40.0 + t));
    // Shielding walls (thicker: 0.6m)
    for &rx in &[58.0f32, 68.0, 80.0] {
        if rx + 0.6 > 94.0 { continue; }
        w.push(wall(rx, 33.0, rx + 0.6, 36.0));
        w.push(wall(rx, 36.5, rx + 0.6, 40.0));
        w.push(wall(rx, 40.5, rx + 0.6, 43.5));
    }

    // ===== LOBBY (x=2..22, y=26..43): open area with seating =====
    w.push(wall(5.0, 30.0, 8.0, 30.5));
    w.push(wall(5.0, 34.0, 8.0, 34.5));
    w.push(wall(12.0, 28.0, 15.0, 28.5));
    w.push(wall(12.0, 32.0, 15.0, 32.5));
    w.push(wall(12.0, 36.0, 15.0, 36.5));
    w.push(wall(18.0, 30.0, 21.0, 30.5));
    w.push(wall(18.0, 38.0, 21.0, 38.5));

    // ===== Stairwells (4 locations) =====
    for &(sx, sy) in &[(2.0, 2.0), (2.0, 88.0), (88.0, 2.0), (88.0, 88.0)] {
        w.push(wall(sx, sy, sx + 5.5, sy + t));
        w.push(wall(sx, sy, sx + t, sy + 5.5));
        w.push(wall(sx + 5.5, sy, sx + 5.5 + t, sy + 2.5));
        w.push(wall(sx, sy + 5.5, sx + 2.5, sy + 5.5 + t));
    }

    // ===== Nurse stations (counter islands at corridor intersections) =====
    for &(nx, ny) in &[(25.5, 46.0), (49.5, 46.0), (73.5, 46.0)] {
        w.push(wall(nx, ny, nx + 1.5, ny + 0.5));
        w.push(wall(nx, ny + 2.0, nx + 1.5, ny + 2.5));
    }

    // ===== Elevator lobby (center) =====
    w.push(wall(46.0, 46.5, 46.5, 48.5));
    w.push(wall(49.5, 46.5, 50.0, 48.5));

    ScenarioDef {
        name: "mega_hospital".into(),
        world_size: ws,
        start: [4.0, 46.5],
        goal: [ws - 4.0, ws - 4.0],
        walls: w,
        obstacle_spawn_ranges: [(3.0, ws - 3.0), (3.0, ws - 3.0)],
    }
}


/// "mega_airport" — 160×160m airport: terminal hall, security, concourse, gates
fn make_mega_airport(ws: f32) -> ScenarioDef {
    let t = 0.3;
    let door = 2.5;
    let mut w = Vec::new();

    // ===== Outer boundary =====
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));

    // ===== CHECK-IN HALL (y=1..28): open area with columns and counters =====
    // Check-in counter rows (long islands)
    for &cy in &[6.0f32, 12.0, 18.0, 24.0] {
        for col in 0..6 {
            let cx = 15.0 + (col as f32) * 22.0;
            if cx + 12.0 > ws - 5.0 { continue; }
            w.push(wall(cx, cy, cx + 12.0, cy + 0.5));
        }
    }
    // Structural columns in check-in hall
    for &px in &[20.0f32, 50.0, 80.0, 110.0, 140.0] {
        for &py in &[8.0f32, 20.0] {
            if px + 1.0 > ws - 2.0 { continue; }
            w.push(wall(px, py, px + 1.0, py + 1.0));
        }
    }

    // ===== SECURITY WALL (y=28..30): solid wall with 4 narrow screening lanes =====
    // Continuous wall with 4 gaps (2.5m each)
    let sec_y = 28.0;
    let lane_positions = [35.0f32, 65.0, 95.0, 125.0];
    let mut prev_x = t;
    for &lx in &lane_positions {
        w.push(wall(prev_x, sec_y, lx, sec_y + 2.0));
        prev_x = lx + door;
    }
    w.push(wall(prev_x, sec_y, ws - t, sec_y + 2.0));

    // ===== POST-SECURITY SHOPS (y=32..52): corridor with shop obstacles =====
    // North and south walls of shopping area
    {
        let mut x = t;
        while x < ws - t {
            let seg = (x + 10.0).min(ws - t);
            w.push(wall(x, 52.0, seg, 52.0 + t));
            x = seg + door;
        }
    }
    // Shop units (rectangular blocks along sides)
    for &sx in &[5.0f32, 25.0, 45.0, 65.0, 95.0, 115.0, 135.0] {
        if sx + 12.0 > ws - 3.0 { continue; }
        w.push(wall(sx, 33.0, sx + 12.0, 33.0 + t));
        w.push(wall(sx, 33.0, sx + t, 38.0));
        w.push(wall(sx + 12.0, 33.0, sx + 12.0 + t, 38.0));
        w.push(wall(sx, 38.0, sx + 4.0, 38.0 + t)); // partial back wall with opening
        w.push(wall(sx + 6.5, 38.0, sx + 12.0, 38.0 + t));
    }
    // Food court islands
    for &(fx, fy) in &[(30.0, 42.0), (60.0, 42.0), (90.0, 42.0), (120.0, 42.0)] {
        if fx + 8.0 > ws - 3.0 { continue; }
        w.push(wall(fx, fy, fx + 8.0, fy + 0.5));
        w.push(wall(fx, fy + 4.0, fx + 8.0, fy + 4.5));
    }

    // ===== MAIN CONCOURSE SPINE (y=52..145, x=73..87, 14m wide) =====
    // West wall of concourse
    {
        let mut y = 52.5;
        while y < 145.0 {
            let seg = (y + 12.0f32).min(145.0);
            w.push(wall(73.0, y, 73.0 + t, seg));
            y = seg + door;
        }
    }
    // East wall of concourse
    {
        let mut y = 52.5;
        while y < 145.0 {
            let seg = (y + 12.0f32).min(145.0);
            w.push(wall(87.0, y, 87.0 + t, seg));
            y = seg + door;
        }
    }
    // Concourse columns
    for &py in &[60.0f32, 75.0, 90.0, 105.0, 120.0, 135.0] {
        w.push(wall(78.0, py, 79.0, py + 1.0));
        w.push(wall(81.0, py, 82.0, py + 1.0));
    }

    // ===== GATE CONCOURSES (branches off main spine) =====
    // West gates at y=68, y=93, y=118 — extending x=5..73
    // East gates at y=68, y=93, y=118 — extending x=87..155
    let gate_ys = [68.0f32, 93.0, 118.0];

    for &gy in &gate_ys {
        // --- WEST GATE CONCOURSE ---
        // Corridor walls (3m wide corridor at gy..gy+3)
        let mut x = 5.0f32;
        while x < 72.5 {
            let seg = (x + 8.0).min(72.5);
            w.push(wall(x, gy, seg, gy + t));
            x = seg + door;
        }
        x = 5.0;
        while x < 72.5 {
            let seg = (x + 8.0).min(72.5);
            w.push(wall(x, gy + 3.0, seg, gy + 3.0 + t));
            x = seg + door;
        }
        // Gate rooms (south of corridor) — 4 gates per branch
        for gate_i in 0..4 {
            let gx = 8.0 + (gate_i as f32) * 16.0;
            if gx + 10.0 > 70.0 { continue; }
            // Gate room: 10×8m south of corridor
            w.push(wall(gx, gy - 8.0, gx + t, gy - 0.5));
            w.push(wall(gx + 10.0, gy - 8.0, gx + 10.0 + t, gy - 0.5));
            w.push(wall(gx, gy - 8.0, gx + 3.5, gy - 8.0 + t));
            w.push(wall(gx + 6.0, gy - 8.0, gx + 10.0, gy - 8.0 + t));
            // Jet bridge (narrow 2m passage extending further south)
            w.push(wall(gx + 3.5, gy - 14.0, gx + 3.5 + t, gy - 8.0));
            w.push(wall(gx + 6.0, gy - 14.0, gx + 6.0 + t, gy - 8.0));
            w.push(wall(gx + 3.5, gy - 14.0, gx + 6.0, gy - 14.0 + t));
        }
        // Gate rooms (north of corridor) — 4 more gates
        for gate_i in 0..4 {
            let gx = 8.0 + (gate_i as f32) * 16.0;
            if gx + 10.0 > 70.0 { continue; }
            w.push(wall(gx, gy + 3.5, gx + t, gy + 11.5));
            w.push(wall(gx + 10.0, gy + 3.5, gx + 10.0 + t, gy + 11.5));
            w.push(wall(gx, gy + 11.5, gx + 3.5, gy + 11.5 + t));
            w.push(wall(gx + 6.0, gy + 11.5, gx + 10.0, gy + 11.5 + t));
        }

        // --- EAST GATE CONCOURSE (mirror) ---
        let mut x = 87.5f32;
        while x < ws - 5.0 {
            let seg = (x + 8.0).min(ws - 5.0);
            w.push(wall(x, gy, seg, gy + t));
            x = seg + door;
        }
        x = 87.5;
        while x < ws - 5.0 {
            let seg = (x + 8.0).min(ws - 5.0);
            w.push(wall(x, gy + 3.0, seg, gy + 3.0 + t));
            x = seg + door;
        }
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

    // ===== End-of-concourse wall =====
    w.push(wall(73.0, 145.0, 87.0, 145.0 + t));

    ScenarioDef {
        name: "mega_airport".into(),
        world_size: ws,
        start: [80.0, 4.0],
        goal: [ws - 8.0, 135.0],
        walls: w,
        obstacle_spawn_ranges: [(5.0, ws - 5.0), (5.0, ws - 5.0)],
    }
}


/// "mega_university" — 192×192m campus: 6 buildings + walkways + quad
fn make_mega_university(ws: f32) -> ScenarioDef {
    let t = 0.3;
    let door = 2.5;
    let mut w = Vec::new();

    // ===== Outer boundary =====
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));

    // Helper: add a rectangular building shell with doors on each wall
    // Returns the interior coordinate ranges for placing internal walls
    // bx,by = bottom-left corner; bw,bh = width,height
    macro_rules! building_shell {
        ($bx:expr, $by:expr, $bw:expr, $bh:expr) => {{
            let (bx, by, bw, bh) = ($bx as f32, $by as f32, $bw as f32, $bh as f32);
            // South wall
            w.push(wall(bx, by, bx + bw * 0.4, by + t));
            w.push(wall(bx + bw * 0.4 + door, by, bx + bw, by + t));
            // North wall
            w.push(wall(bx, by + bh, bx + bw * 0.6, by + bh + t));
            w.push(wall(bx + bw * 0.6 + door, by + bh, bx + bw, by + bh + t));
            // West wall
            w.push(wall(bx, by, bx + t, by + bh * 0.45));
            w.push(wall(bx, by + bh * 0.45 + door, bx + t, by + bh));
            // East wall
            w.push(wall(bx + bw, by, bx + bw + t, by + bh * 0.55));
            w.push(wall(bx + bw, by + bh * 0.55 + door, bx + bw + t, by + bh));
        }};
    }

    // ==================== LIBRARY (NW quadrant: 8..58, 130..180) ====================
    building_shell!(8.0, 130.0, 50.0, 50.0);
    // Dense shelving rows (parallel vertical walls every 3.5m)
    {
        let mut sx = 12.0f32;
        while sx < 54.0 {
            // Shelf: tall narrow wall with gap at south end
            w.push(wall(sx, 135.0, sx + t, 155.0));
            w.push(wall(sx, 158.0, sx + t, 176.0));
            sx += 3.5;
        }
        // Reading room partitions (horizontal)
        w.push(wall(12.0, 155.0, 30.0, 155.0 + t));
        w.push(wall(32.5, 155.0, 55.0, 155.0 + t));
        w.push(wall(12.0, 165.0, 25.0, 165.0 + t));
        w.push(wall(27.5, 165.0, 55.0, 165.0 + t));
    }

    // ==================== ENGINEERING (NE: 130..185, 130..180) ====================
    building_shell!(130.0, 130.0, 55.0, 50.0);
    // Large labs with equipment
    {
        // Vertical partitions
        for &lx in &[148.0f32, 165.0] {
            w.push(wall(lx, 132.0, lx + t, 148.0));
            w.push(wall(lx, 150.5, lx + t, 165.0));
            w.push(wall(lx, 167.5, lx + t, 178.0));
        }
        // Horizontal partitions
        for &ly in &[148.0f32, 162.0] {
            w.push(wall(132.0, ly, 147.5, ly + t));
            w.push(wall(150.0, ly, 164.5, ly + t));
            w.push(wall(167.0, ly, 183.0, ly + t));
        }
        // Lab benches (equipment islands)
        for &(ex, ey) in &[
            (135.0, 136.0), (135.0, 142.0), (152.0, 136.0), (152.0, 142.0),
            (170.0, 136.0), (170.0, 142.0), (135.0, 152.0), (152.0, 152.0),
            (170.0, 152.0), (135.0, 170.0), (152.0, 170.0), (170.0, 170.0),
        ] {
            w.push(wall(ex, ey, ex + 8.0, ey + 0.5));
        }
    }

    // ==================== ADMIN (SW: 8..55, 8..55) ====================
    building_shell!(8.0, 8.0, 47.0, 47.0);
    // Office cubicle grid
    {
        // Vertical cubicle walls every 4m
        let mut cx = 13.0f32;
        while cx < 52.0 {
            w.push(wall(cx, 10.0, cx + t, 22.0));
            w.push(wall(cx, 24.5, cx + t, 36.0));
            w.push(wall(cx, 38.5, cx + t, 52.0));
            cx += 4.0;
        }
        // Horizontal cubicle walls every 6m
        for &cy in &[22.0f32, 36.0] {
            let mut cx2 = 10.0f32;
            while cx2 < 52.0 {
                let seg = (cx2 + 3.0).min(52.0);
                w.push(wall(cx2, cy, seg, cy + t));
                cx2 = seg + 1.5; // small gap between cubicle sections
            }
        }
        // Conference rooms (SW and NE corners inside building)
        w.push(wall(10.0, 10.0, 18.0, 10.0 + t));
        w.push(wall(18.0, 10.0, 18.0 + t, 18.0));
        w.push(wall(10.0, 18.0, 18.0, 18.0 + t));
        w.push(wall(40.0, 42.0, 52.0, 42.0 + t));
        w.push(wall(40.0, 42.0, 40.0 + t, 52.0));
    }

    // ==================== SCIENCE (SE: 130..185, 8..55) ====================
    building_shell!(130.0, 8.0, 55.0, 47.0);
    // Lab benches + fume hood walls
    {
        // Horizontal lab benches
        for row in 0..5 {
            let ly = 12.0 + (row as f32) * 8.0;
            if ly + 6.0 > 52.0 { continue; }
            for col in 0..4 {
                let lx = 134.0 + (col as f32) * 13.0;
                if lx + 10.0 > 183.0 { continue; }
                w.push(wall(lx, ly, lx + 10.0, ly + 0.5));
            }
        }
        // Fume hood walls (thick, along north wall of each lab section)
        for &fy in &[18.0f32, 34.0] {
            w.push(wall(132.0, fy, 155.0, fy + 0.6));
            w.push(wall(158.0, fy, 183.0, fy + 0.6));
        }
        // Storage room partition
        w.push(wall(160.0, 10.0, 160.0 + t, 25.0));
        w.push(wall(160.0, 27.5, 160.0 + t, 42.0));
    }

    // ==================== GYM / STUDENT CENTER (center-south: 68..125, 10..55) ====================
    building_shell!(68.0, 10.0, 57.0, 45.0);
    {
        // Main gym hall is mostly open — just equipment islands
        w.push(wall(75.0, 20.0, 85.0, 20.5));
        w.push(wall(75.0, 30.0, 85.0, 30.5));
        w.push(wall(75.0, 40.0, 85.0, 40.5));
        // Partition between gym and cafeteria
        w.push(wall(95.0, 12.0, 95.0 + t, 28.0));
        w.push(wall(95.0, 30.5, 95.0 + t, 52.0));
        // Cafeteria tables
        for &(cx, cy) in &[
            (100.0, 18.0), (100.0, 26.0), (100.0, 34.0), (100.0, 42.0),
            (110.0, 18.0), (110.0, 26.0), (110.0, 34.0), (110.0, 42.0),
        ] {
            w.push(wall(cx, cy, cx + 6.0, cy + 0.5));
        }
    }

    // ==================== LECTURE HALL (center-north: 68..125, 130..175) ====================
    building_shell!(68.0, 130.0, 57.0, 45.0);
    {
        // Tiered seating walls (horizontal rows)
        for row in 0..6 {
            let ry = 135.0 + (row as f32) * 6.0;
            if ry + t > 170.0 { continue; }
            // Each tier is a partial wall (leave aisles at sides)
            w.push(wall(73.0, ry, 90.0, ry + t));
            w.push(wall(93.0, ry, 120.0, ry + t));
        }
        // Stage/podium wall at north end
        w.push(wall(72.0, 168.0, 122.0, 168.0 + 0.5));
        // Central aisle
        w.push(wall(90.0 + t, 132.0, 93.0, 132.0 + t)); // aisle marker
    }

    // ==================== CENTRAL QUAD (75..120, 65..120): open with paths ====================
    // Benches and planters (small obstacles in outdoor space)
    for &(px, py) in &[
        (82.0, 75.0), (82.0, 85.0), (82.0, 95.0), (82.0, 105.0),
        (100.0, 75.0), (100.0, 85.0), (100.0, 95.0), (100.0, 105.0),
        (113.0, 80.0), (113.0, 100.0),
    ] {
        w.push(wall(px, py, px + 3.0, py + 0.5));
    }
    // Fountain / monument (center of quad)
    w.push(wall(93.0, 88.0, 95.0, 90.0));
    w.push(wall(99.0, 88.0, 101.0, 90.0));
    w.push(wall(93.0, 94.0, 95.0, 96.0));
    w.push(wall(99.0, 94.0, 101.0, 96.0));

    // ==================== COVERED WALKWAYS (narrow outdoor corridors) ====================
    // These connect buildings through the quad area with 2.5m wide paths
    // bounded by low walls (hedges/railings)

    // Library → Lecture Hall (west side, x≈60..68)
    w.push(wall(60.0, 58.0, 60.0 + t, 128.0));
    w.push(wall(63.0, 58.0, 63.0 + t, 128.0));

    // Engineering → Science (east side, x≈127..130)
    w.push(wall(127.0, 58.0, 127.0 + t, 128.0));
    w.push(wall(129.5, 58.0, 129.5 + t, 128.0));

    // Admin → Gym (south middle, y≈56..68)
    w.push(wall(58.0, 56.0, 65.0, 56.0 + t));
    w.push(wall(58.0, 58.5, 65.0, 58.5 + t));

    // Gym → Science (south, y≈56..68)
    w.push(wall(126.0, 56.0, 128.0, 56.0 + t));
    w.push(wall(126.0, 58.5, 128.0, 58.5 + t));

    // North walkway: Library → Lecture Hall → Engineering (y≈126..130)
    w.push(wall(60.0, 126.0, 66.0, 126.0 + t));
    w.push(wall(60.0, 128.5, 66.0, 128.5 + t));
    w.push(wall(126.0, 126.0, 128.0, 126.0 + t));
    w.push(wall(126.0, 128.5, 128.0, 128.5 + t));

    ScenarioDef {
        name: "mega_university".into(),
        world_size: ws,
        start: [12.0, 12.0],
        goal: [ws - 8.0, ws - 8.0],
        walls: w,
        obstacle_spawn_ranges: [(5.0, ws - 5.0), (5.0, ws - 5.0)],
    }
}


fn make_zigzag_wide(ws: f32) -> ScenarioDef {
    let t = 0.3;
    let mut w = Vec::new();
    // Outer walls
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));
    // Thinner Z-walls: 2m thick instead of 8.4m, gap = 10m
    w.push(wall(t, 8.0, 22.0, 10.0));   // left wall, gap on right
    w.push(wall(10.0, 20.0, ws - t, 22.0)); // right wall, gap on left
    // Small pillars
    w.push(wall(8.0, 3.0, 9.0, 4.0));
    w.push(wall(16.0, 2.5, 17.0, 3.5));
    w.push(wall(12.0, 12.0, 13.0, 13.0));
    w.push(wall(22.0, 12.5, 23.0, 13.5));
    w.push(wall(10.0, 24.0, 11.0, 25.0));
    w.push(wall(20.0, 24.5, 21.0, 25.5));
    ScenarioDef {
        name: "zigzag_wide".into(),
        world_size: ws,
        start: [2.0, 3.0],
        goal: [ws - 2.0, 27.5],
        walls: w,
        obstacle_spawn_ranges: [(2.0, ws - 2.0), (2.0, ws - 2.0)],
    }
}

fn make_zigzag(ws: f32) -> ScenarioDef {
    let t = 0.3;
    let mut w = Vec::new();
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));
    w.push(wall(t, 5.3, 26.0, 13.7));
    w.push(wall(6.0, 17.3, ws - t, 25.7));
    w.push(wall(8.0, 3.0, 9.0, 4.0));
    w.push(wall(16.0, 2.5, 17.0, 4.5));
    w.push(wall(12.0, 14.5, 13.0, 16.5));
    w.push(wall(22.0, 14.8, 23.0, 16.2));
    w.push(wall(10.0, 26.5, 11.0, 28.0));
    w.push(wall(20.0, 26.2, 21.0, 28.3));
    w.push(wall(27.0, 8.0, 28.5, 9.0));
    w.push(wall(3.5, 20.0, 5.0, 21.0));
    ScenarioDef {
        name: "zigzag".into(),
        world_size: ws,
        start: [2.0, 3.5],
        goal: [ws - 2.0, 27.5],
        walls: w,
        obstacle_spawn_ranges: [(2.0, ws - 2.0), (2.0, ws - 2.0)],
    }
}

fn make_scattered(ws: f32) -> ScenarioDef {
    let t = 0.3;
    let mut w = Vec::new();
    w.push(wall(0.0, 0.0, ws, t));
    w.push(wall(0.0, ws - t, ws, ws));
    w.push(wall(0.0, 0.0, t, ws));
    w.push(wall(ws - t, 0.0, ws, ws));
    for &(x,y) in &[(5.0,4.0),(9.0,5.0),(13.0,4.5),(17.0,6.0),(22.0,4.0),(26.0,5.5),
        (3.0,8.5),(7.0,9.0),(11.0,8.0),(15.0,10.0),(19.5,8.5),(24.0,9.0),(28.0,8.0),
        (4.5,12.0),(8.5,13.5),(12.5,12.0),(16.5,14.0),(21.0,12.5),(25.5,13.0),
        (3.0,16.5),(7.5,17.0),(10.0,16.0),(14.0,18.0),(18.5,16.5),(23.0,17.5),(27.5,16.0),
        (5.0,20.0),(9.0,21.5),(13.5,20.5),(17.0,22.0),(21.5,20.0),(26.0,21.5),
        (4.0,24.0),(8.0,25.5),(12.0,24.5),(16.0,26.0),(20.5,24.0),(25.0,25.5),(29.0,24.0)] {
        w.push(wall(x, y, x+1.0, y+1.0));
    }
    ScenarioDef {
        name: "scattered".into(),
        world_size: ws,
        start: [2.0, 2.0],
        goal: [ws - 2.0, ws - 2.0],
        walls: w,
        obstacle_spawn_ranges: [(3.0, ws - 3.0), (3.0, ws - 3.0)],
    }
}

fn make_scenario(name: &str, ws: f32) -> ScenarioDef {
    match name {
        "open" => make_open(ws),
        "office" => make_office(ws),
        "building" => make_building(ws),
        "warehouse" => make_warehouse(ws),
        "megacampus" => make_megacampus(128.0),
        "mega_hospital" => make_mega_hospital(96.0),
        "mega_airport" => make_mega_airport(160.0),
        "mega_university" => make_mega_university(192.0),
        "zigzag" => make_zigzag(ws),
        "zigzag_wide" => make_zigzag_wide(ws),
        "scattered" => make_scattered(ws),
        _ => {
            eprintln!("Unknown scenario '{}', defaulting to 'building'", name);
            make_building(ws)
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

// ==================== Build/rebuild SMART from a position ====================
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
    // Inflate walls by robot body radius so RRT* nodes stay clear of walls
    let inflated = inflate_walls(walls, robot_radius);

    let static_vc: Box<dyn ValidityChecker<f32, 2>> = if inflated.is_empty() {
        Box::new(AlwaysValid::new())
    } else {
        Box::new(AnalyticValidityChecker::new(inflated))
    };

    let area = (ranges[0].1 - ranges[0].0) * (ranges[1].1 - ranges[1].0);
    let gamma = optimal_gamma(area, 2);

    let sd: Box<dyn SamplingDistribution<f32, 2>> = if goal_bias > 0.001 {
        Box::new(GoalBiasedDistribution::new(ranges, from.clone(), goal_bias))
    } else {
        Box::new(UniformDistribution::new(ranges))
    };

    let mut rrt_star: RRTstar<f32, 2, KdTreeNearestNeighbors<f32, 2>> = RRTstar::new(
        goal.clone(), from.clone(), 1.0, static_vc,
        sd, 1.0, 1.7, gamma,
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

    if shortcut_iters > 0 {
        smart.shortcut_current_path(shortcut_iters);
    }

    let path = smart.get_path()?;
    Some((smart, path))
}

// ==================== Run one scenario ====================
fn run_scenario(scenario: &ScenarioDef, args: &CliArgs, out: &mut io::StdoutLock) -> bool {
    let ws = scenario.world_size;
    let obstacle_radius = 0.6;
    let ohz_time = 0.5;
    let sensor_range = args.sensor_range;

    let start = RealVectorState::new(scenario.start);
    let goal = RealVectorState::new(scenario.goal);
    let ranges = [(0.0, ws), (0.0, ws)];
    let iters = args.rrt_iterations.max(5000);

    eprintln!("\n=== Scenario: {} ===", scenario.name);
    eprintln!("  World: {0}x{0}m | TRUE walls: {1} (unknown to robot!) | People: {2} @ {3}m/s | Sensor: {4}m | Robot radius: {5}m",
        ws, scenario.walls.len(), args.num_obstacles, args.obstacle_speed, sensor_range, args.robot_radius);

    // ============================================================
    // KEY: Robot starts knowing NOTHING about walls.
    // Initial RRT* is built in OPEN SPACE — tree goes everywhere.
    // As robot moves, LiDAR discovers walls → known_walls grows
    // → SMART is rebuilt with updated known_walls.
    // ============================================================
    let mut known_walls: Vec<StaticRectangularObstacle<f32, 2>> = Vec::new();
    let mut walls_discovered: Vec<bool> = vec![false; scenario.walls.len()];

    // ===== Occupancy grid: fog of war =====
    let grid_w = (ws / GRID_RES) as usize;
    let grid_h = (ws / GRID_RES) as usize;
    let mut scanned_grid: Vec<i8> = vec![-1; grid_w * grid_h]; // -1=unknown

    // Initial LiDAR scan from start position
    let sv = start.values();
    let initial_new = lidar_scan(
        sv[0], sv[1], sensor_range, &scenario.walls,
        &mut walls_discovered, &mut known_walls, &mut scanned_grid, grid_w, grid_h,
    );
    eprintln!("  Initial scan: discovered {}/{} walls near start", known_walls.len(), scenario.walls.len());

    // Build SMART with only the walls discovered so far (may be 0!)
    eprintln!("  Building initial RRT* ({} iters, {} known walls)...", iters, known_walls.len());
    let (mut smart, initial_path) = match build_smart(&start, &goal, &known_walls, ranges, args.robot_speed, args.robot_radius, iters, args.goal_bias, args.shortcut_iters, args.lrz_time, args.lsr_max) {
        Some(s) => s,
        None => {
            eprintln!("  ERROR: No initial path! Try --rrt-iterations 10000");
            let j = object! { "type": "scenario_error", "scenario": scenario.name.as_str(), "error": "No initial path" };
            let _ = writeln!(out, "{}", j.dump());
            return true;
        }
    };
    let lrz_radius = args.robot_speed * args.lrz_time;
    let lsr_max_radius = args.lsr_max;
    eprintln!("  Initial path found ✓ (may go through undiscovered walls!)");

    // Spawn dynamic obstacles (the TRUE map knows about walls, people bounce off them)
    let spawn = scenario.obstacle_spawn_ranges;
    let mut obs_sampling = UniformDistribution::new(spawn);
    let mut obstacles: Vec<Box<dyn DynamicSphericalObstacle<f32, 2>>> = Vec::new();
    let margin = obstacle_radius + 0.3;
    let mut spawned = 0;
    let mut attempts = 0;
    while spawned < args.num_obstacles && attempts < args.num_obstacles * 50 {
        let pos = obs_sampling.sample();
        attempts += 1;
        let pv = pos.values();
        let in_wall = scenario.walls.iter().any(|w| {
            let min = w.min_corner().values();
            let max = w.max_corner().values();
            pv[0] >= min[0] - margin && pv[0] <= max[0] + margin &&
            pv[1] >= min[1] - margin && pv[1] <= max[1] + margin
        });
        let near_start = (pv[0] - scenario.start[0]).powi(2) + (pv[1] - scenario.start[1]).powi(2) < 9.0;
        let near_goal = (pv[0] - scenario.goal[0]).powi(2) + (pv[1] - scenario.goal[1]).powi(2) < 9.0;
        if !in_wall && !near_start && !near_goal {
            let c = StaticSphericalObstacle::new(pos, obstacle_radius);
            obstacles.push(Box::new(RandomDirectionCircle::new(
                c, args.obstacle_speed, 0.5, 5.0, spawn, scenario.walls.clone(), // TRUE walls for physics
            )));
            spawned += 1;
        }
    }
    eprintln!("  Spawned {} people", spawned);

    // Metadata — send TRUE walls for RViz2 rendering (viewer can see everything)
    let meta = object! {
        "type": "metadata",
        "scenario": scenario.name.as_str(),
        "world_size": ws,
        "start": [scenario.start[0], scenario.start[1]],
        "goal": [scenario.goal[0], scenario.goal[1]],
        "robot_speed": args.robot_speed,
        "obstacle_speed": args.obstacle_speed,
        "num_obstacles": spawned as u32,
        "obstacle_radius": obstacle_radius,
        "robot_radius": args.robot_radius,
        "ohz_time": ohz_time,
        "lrz_radius": lrz_radius,
        "lsr_max_radius": lsr_max_radius,
        "sensor_range": sensor_range,
        "dt": args.dt,
        "walls": walls_to_json(&scenario.walls),
        "grid_res": GRID_RES,
        "grid_w": grid_w as u32,
        "grid_h": grid_h as u32,
        "grid": grid_to_string(&scanned_grid),
    };
    if writeln!(out, "{}", meta.dump()).is_err() { return false; }

    // Simulation state
    let dt = args.dt;
    let mut robot = start.clone();
    let mut current_path = initial_path;
    let mut path_idx = 1usize;
    let mut frame: u64 = 0;
    let mut sim_time = 0.0f32;
    let delay = Duration::from_secs_f32(dt / args.playback_speed);
    let mut rebuild_count = 0u32;
    let mut cached_inflated = inflate_walls(&known_walls, args.robot_radius);
    let mut total_walls_known = known_walls.len();
    let mut frames_since_detour_rebuild: u64 = 0;
    let mut detour_fail_streak: u32 = 0;
    let mut last_progress_pos = robot.clone();
    let mut frames_since_progress: u64 = 0;

    // Senior mode overrides
    let shortcut_iters: usize = if args.senior_mode { 0 } else { args.shortcut_iters };

    eprintln!("  Simulating ({}x speed{})...", args.playback_speed,
        if args.senior_mode { ", SENIOR MODE" } else { "" });

    let mut emergency_stopped = false;

    loop {
        let t0 = Instant::now();
        sim_time += dt;
        frames_since_detour_rebuild += 1;

        // Step 1: Move obstacles FIRST
        for o in &mut obstacles { o.update(dt); }

        // Step 2: Emergency stop check
        let collision_margin = args.robot_radius + obstacle_radius + 0.15;
        let rv_pre = robot.values();
        let (would_be_pos, _) = holonomic_move_along_path(
            &robot, &current_path, path_idx, args.robot_speed * dt);
        let wv = would_be_pos.values();
        let all_ohz_tmp = make_spherical_obstacles_with_ohzs(&obstacles, args.obstacle_speed, ohz_time);
        let imminent_collision = all_ohz_tmp.iter().any(|obs| {
            let oc = obs.obstacle().center().values();
            let d_cur = ((rv_pre[0]-oc[0]).powi(2) + (rv_pre[1]-oc[1]).powi(2)).sqrt();
            let d_next = ((wv[0]-oc[0]).powi(2) + (wv[1]-oc[1]).powi(2)).sqrt();
            d_cur < collision_margin || d_next < collision_margin
        });

        // Step 3: Move robot only if safe
        if imminent_collision {
            if !emergency_stopped {
                eprintln!("  🛑 Emergency stop — obstacle too close at ({:.1},{:.1})",
                    rv_pre[0], rv_pre[1]);
            }
            emergency_stopped = true;
        } else {
            let (new_robot, new_idx) = holonomic_move_along_path(
                &robot, &current_path, path_idx, args.robot_speed * dt);
            // Wall collision check: don't move if path to next position crosses a known wall
            let wv = robot.values();
            let robot_inside_wall = cached_inflated.iter().any(|w| {
                let mn = w.min_corner().values();
                let mx = w.max_corner().values();
                wv[0] >= mn[0] && wv[0] <= mx[0] && wv[1] >= mn[1] && wv[1] <= mx[1]
            });
            let wall_blocked = robot_inside_wall || cached_inflated.iter().any(|w| {
                w.intersects_edge(&robot, &new_robot)
            });
            if wall_blocked {
                // Don't move — path crosses a known wall. Trigger immediate replan.
                eprintln!("  🧱 Wall block at ({:.1},{:.1}) — triggering replan", rv_pre[0], rv_pre[1]);
                // Try incremental repair first (LSR hot-node search)
                let inflated = inflate_walls(&known_walls, args.robot_radius);
                let repair_vc: Box<dyn ValidityChecker<f32, 2>> = if inflated.is_empty() {
                    Box::new(AlwaysValid::new())
                } else {
                    Box::new(AnalyticValidityChecker::new(inflated))
                };
                let mut rt = MaxTimeTermination::new(Duration::from_secs_f32(dt * 2.0));
                let repair_ok = if args.senior_mode {
                    panic::catch_unwind(panic::AssertUnwindSafe(|| {
                        smart.update_static_obstacles_path_only(repair_vc, Some(&mut rt), robot.clone())
                    }))
                } else {
                    panic::catch_unwind(panic::AssertUnwindSafe(|| {
                        smart.update_static_obstacles(repair_vc, Some(&mut rt), robot.clone())
                    }))
                };
                match repair_ok {
                    Ok(SmartUpdateResult::ReplanningSuccessful) => {
                        if let Some(new_path) = smart.get_path() {
                            // Senior mode: validate path doesn't cross walls
                            let path_dirty = args.senior_mode && cached_inflated.iter().any(|w| {
                                new_path.windows(2).any(|pair| w.intersects_edge(&pair[0], &pair[1]))
                            });
                            if path_dirty {
                                rebuild_count += 1;
                                eprintln!("  ⚠ Senior wall-block repair: path still dirty — full rebuild (#{})", rebuild_count);
                                match build_smart(&robot, &goal, &known_walls, ranges,
                                        args.robot_speed, args.robot_radius, iters, args.goal_bias, shortcut_iters, args.lrz_time, args.lsr_max) {
                                    Some((ns, np)) => {
                                        smart = ns; current_path = np; path_idx = 1;
                                        cached_inflated = inflate_walls(&known_walls, args.robot_radius);
                                    }
                                    None => { eprintln!("  ⚠ Full rebuild also failed — keeping current position"); }
                                }
                            } else {
                                current_path = new_path;
                                path_idx = 0;
                                cached_inflated = inflate_walls(&known_walls, args.robot_radius);
                                eprintln!("  ✓ Wall block repaired via LSR");
                            }
                        }
                    }
                    _ => {
                        // LSR repair failed — fallback to full rebuild
                        rebuild_count += 1;
                        eprintln!("  ⚠ LSR repair failed — full rebuild (#{})", rebuild_count);
                        match build_smart(&robot, &goal, &known_walls, ranges,
                                args.robot_speed, args.robot_radius, iters, args.goal_bias, shortcut_iters, args.lrz_time, args.lsr_max) {
                            Some((ns, np)) => {
                                smart = ns; current_path = np; path_idx = 1;
                                cached_inflated = inflate_walls(&known_walls, args.robot_radius);
                            }
                            None => { eprintln!("  ⚠ Full rebuild also failed — keeping current position"); }
                        }
                    }
                }
            } else {
                robot = new_robot;
                path_idx = new_idx;
            }
            if emergency_stopped {
                eprintln!("  ✅ Resuming movement — path clear");
                emergency_stopped = false;
            }
        }

        let all_ohz = make_spherical_obstacles_with_ohzs(&obstacles, args.obstacle_speed, ohz_time);

        // Sensor filter for dynamic obstacles (distance + wall occlusion)
        let rv = robot.values();
        let visible_ohz: Vec<_> = all_ohz.iter().filter(|obs| {
            let oc = obs.obstacle().center().values();
            let d = ((rv[0] - oc[0]).powi(2) + (rv[1] - oc[1]).powi(2)).sqrt();
            d <= sensor_range + obs.obstacle().radius()
                && is_visible_through_walls(rv[0], rv[1], oc[0], oc[1], &known_walls)
        }).cloned().collect();

        // ============================================================
        // LiDAR SCAN: cast rays, update fog-of-war grid, discover walls
        // Rays are physically blocked by walls (no see-through!)
        // ============================================================
        let new_walls_found = lidar_scan(
            rv[0], rv[1], sensor_range, &scenario.walls,
            &mut walls_discovered, &mut known_walls,
            &mut scanned_grid, grid_w, grid_h,
        );

        // Rebuild SMART when new walls discovered
        let mut replanned = false;
        if new_walls_found {
            let new_count = known_walls.len() - total_walls_known;
            total_walls_known = known_walls.len();
            eprintln!("  🔍 Discovered {} new wall(s)! Total known: {}/{}. Incremental update...",
                new_count, known_walls.len(), scenario.walls.len());

            let new_walls_slice = &known_walls[known_walls.len() - new_count..];
            let inflated_new = inflate_walls(new_walls_slice, args.robot_radius);
            cached_inflated.extend(inflated_new.iter().cloned());

            // Check if goal just became unreachable (inside lethal wall)
            let gv_ck = goal.values();
            if cached_inflated.iter().any(|w| {
                let mn = w.min_corner().values(); let mx = w.max_corner().values();
                gv_ck[0] >= mn[0] && gv_ck[0] <= mx[0] && gv_ck[1] >= mn[1] && gv_ck[1] <= mx[1]
            }) {
                eprintln!("  ❌ UNREACHABLE: Goal ({:.1},{:.1}) inside lethal wall — stopping",
                    gv_ck[0], gv_ck[1]);
                break;
            }

            // Check ALL known walls (not just new ones) — matches 3D logic
            let path_affected = cached_inflated.iter().any(|w| {
                current_path.windows(2).any(|pair| w.intersects_edge(&pair[0], &pair[1]))
            });

            if !path_affected {
                let lazy_vc: Box<dyn ValidityChecker<f32, 2>> = if cached_inflated.is_empty() {
                    Box::new(AlwaysValid::new())
                } else {
                    Box::new(AnalyticValidityChecker::new(cached_inflated.clone()))
                };
                smart.set_static_validity_checker(lazy_vc);
                eprintln!("  ✓ New walls don't affect path — lazy checker swap");
            } else {

            let inflated = inflate_walls(&known_walls, args.robot_radius);
            let new_vc: Box<dyn ValidityChecker<f32, 2>> = if inflated.is_empty() {
                Box::new(AlwaysValid::new())
            } else {
                Box::new(AnalyticValidityChecker::new(inflated))
            };

            let mut rt = MaxTimeTermination::new(Duration::from_secs_f32(dt * 1.5));
            let inc_result = if args.senior_mode {
                panic::catch_unwind(panic::AssertUnwindSafe(|| {
                    smart.update_static_obstacles_path_only(new_vc, Some(&mut rt), robot.clone())
                }))
            } else {
                panic::catch_unwind(panic::AssertUnwindSafe(|| {
                    smart.update_static_obstacles(new_vc, Some(&mut rt), robot.clone())
                }))
            };

            match inc_result {
                Ok(SmartUpdateResult::ReplanningSuccessful) => {
                    if let Some(new_path) = smart.get_path() {
                        // Senior mode: validate entire path against known walls.
                        // Path-only repair leaves dirty edges in the tree, so
                        // build_planned_path() can return a path that crosses walls.
                        let path_dirty = args.senior_mode && cached_inflated.iter().any(|w| {
                            new_path.windows(2).any(|pair| w.intersects_edge(&pair[0], &pair[1]))
                        });

                        if path_dirty {
                            // Path still crosses walls after path-only repair.
                            // Fall back to full rebuild (this is the senior baseline's cost).
                            rebuild_count += 1;
                            cached_inflated = inflate_walls(&known_walls, args.robot_radius);
                            eprintln!("  ⚠ Senior path-only: new path still crosses walls — full rebuild (#{})", rebuild_count);
                            match build_smart(&robot, &goal, &known_walls, ranges, args.robot_speed, args.robot_radius, iters, args.goal_bias, shortcut_iters, args.lrz_time, args.lsr_max) {
                                Some((new_smart, new_path)) => {
                                    smart = new_smart;
                                    current_path = new_path;
                                    path_idx = 1;
                                    replanned = true;
                                }
                                None => {
                                    eprintln!("  ⚠ Full rebuild also failed — keeping current position");
                                }
                            }
                        } else {
                        current_path = new_path;
                        cached_inflated = inflate_walls(&known_walls, args.robot_radius);
                        // NO shortcut here — shortcut creates long segments that cross undiscovered walls
                        // Shortcut only runs after build_smart() and update() where path quality is higher

                        // Forward-skip: find the first forward-facing, wall-clear, close point
                        // Avoids the 1~2m backwards hop when nearest tree node is behind robot
                        path_idx = 0;
                        let rv = robot.values();
                        let gv = goal.values();
                        let to_goal = [gv[0] - rv[0], gv[1] - rv[1]];
                        let max_skip_dist = 5.0f32; // Don't jump further than 5m
                        let scan = current_path.len().min(15);
                        for i in 1..scan {
                            let pv = current_path[i].values();
                            let to_pt = [pv[0] - rv[0], pv[1] - rv[1]];
                            let d = (to_pt[0]*to_pt[0] + to_pt[1]*to_pt[1]).sqrt();
                            if d > max_skip_dist { break; } // Too far, stop scanning
                            let dot_fwd = to_pt[0] * to_goal[0] + to_pt[1] * to_goal[1];
                            if dot_fwd <= 0.0 { continue; } // Behind us
                            let blocked = cached_inflated.iter().any(|w| {
                                w.intersects_edge(&robot, &current_path[i])
                            });
                            if !blocked {
                                path_idx = i; // Take the first valid forward point (closest, safest)
                                break;
                            }
                        }
                        replanned = true;
                        // Diagnostic
                        if path_idx < current_path.len() {
                            let pv = current_path[path_idx].values();
                            let to_path = [pv[0] - rv[0], pv[1] - rv[1]];
                            let dot = to_path[0] * to_goal[0] + to_path[1] * to_goal[1];
                            let d_path = (to_path[0]*to_path[0] + to_path[1]*to_path[1]).sqrt();
                            if dot < 0.0 && d_path > 1.0 {
                                eprintln!("  ❌ UTURN robot=({:.1},{:.1}) path[{}]=({:.1},{:.1}) goal=({:.1},{:.1}) dot={:.1}",
                                    rv[0], rv[1], path_idx, pv[0], pv[1], gv[0], gv[1], dot);
                            } else {
                                eprintln!("  ✓ Incremental OK — {} walls, skip={}, heading {:.0}° (dot={:.1}, d={:.1}m)",
                                    known_walls.len(), path_idx,
                                    (to_path[1].atan2(to_path[0])).to_degrees(),
                                    dot, d_path);
                            }
                        }
                        } // end !path_dirty
                    } else {
                        rebuild_count += 1;
                        cached_inflated = inflate_walls(&known_walls, args.robot_radius);
                        eprintln!("  ⚠ Incremental path=None — falling back to full rebuild (#{})", rebuild_count);
                        match build_smart(&robot, &goal, &known_walls, ranges, args.robot_speed, args.robot_radius, iters, args.goal_bias, shortcut_iters, args.lrz_time, args.lsr_max) {
                            Some((new_smart, new_path)) => {
                                smart = new_smart;
                                current_path = new_path;
                                path_idx = 1;
                                replanned = true;
                            }
                            None => {
                                eprintln!("  ⚠ Full rebuild also failed — keeping current position");
                            }
                        }
                    }
                }
                Ok(SmartUpdateResult::NoReplanningNeeded) => {
                    eprintln!("  ✓ New walls didn't affect tree — no repair needed");
                }
                Ok(_) | Err(_) => {
                    // Incremental update failed or panicked, fall back to full rebuild
                    rebuild_count += 1;
                    eprintln!("  ⚠ Incremental update failed — falling back to full rebuild (#{})", rebuild_count);
                    match build_smart(&robot, &goal, &known_walls, ranges, args.robot_speed, args.robot_radius, iters, args.goal_bias, shortcut_iters, args.lrz_time, args.lsr_max) {
                        Some((new_smart, new_path)) => {
                            smart = new_smart;
                            current_path = new_path;
                            path_idx = 1;
                            replanned = true;
                        }
                        None => {
                            eprintln!("  ⚠ Full rebuild also failed — keeping current position");
                        }
                    }
                }
            }
            } // end else (path_affected)
        }

        // SMART update for dynamic obstacles (only if we didn't just rebuild)
        let result = if !replanned {
            let mut rt = MaxTimeTermination::new(Duration::from_secs_f32(dt * 0.5));
            let update_result = panic::catch_unwind(panic::AssertUnwindSafe(|| {
                smart.update(robot.clone(), path_idx, &visible_ohz, Some(&mut rt))
            }));

            match update_result {
                Ok(SmartUpdateResult::ReplanningSuccessful) => {
                    current_path = smart.get_path().unwrap();
                    path_idx = 0;
                    replanned = true;
                    SmartUpdateResult::ReplanningSuccessful
                }
                Ok(other) => other,
                Err(_) => {
                    // SMART panicked — rebuild with known walls
                    rebuild_count += 1;
                    eprintln!("  🔄 SMART internal error — rebuilding (#{})", rebuild_count);
                    match build_smart(&robot, &goal, &known_walls, ranges, args.robot_speed, args.robot_radius, iters, args.goal_bias, shortcut_iters, args.lrz_time, args.lsr_max) {
                        Some((new_smart, new_path)) => {
                            smart = new_smart;
                            current_path = new_path;
                            path_idx = 1;
                            replanned = true;
                            SmartUpdateResult::ReplanningSuccessful
                        }
                        None => SmartUpdateResult::ReplanningFailed,
                    }
                }
            }
        } else {
            SmartUpdateResult::ReplanningSuccessful
        };



        // ---- Detour detection ----
        let detour_cooldown = 100u64 * (1u64 << detour_fail_streak.min(5));
        if !replanned && current_path.len() >= 2 && frames_since_detour_rebuild > detour_cooldown && args.detour_threshold > 0.0 && !args.senior_mode {
            let rv = robot.values();
            let gv = goal.values();
            let straight_dist = ((rv[0]-gv[0]).powi(2) + (rv[1]-gv[1]).powi(2)).sqrt();
            let mut path_len: f32 = 0.0;
            let start_from = path_idx.min(current_path.len() - 1);
            let fp = current_path[start_from].values();
            path_len += ((rv[0]-fp[0]).powi(2) + (rv[1]-fp[1]).powi(2)).sqrt();
            for w in current_path[start_from..].windows(2) {
                let a = w[0].values();
                let b = w[1].values();
                path_len += ((a[0]-b[0]).powi(2) + (a[1]-b[1]).powi(2)).sqrt();
            }
            let detour_ratio = if straight_dist > 5.0 { path_len / straight_dist } else { 1.0 };
            let effective_threshold = args.detour_threshold + 10.0 / straight_dist.max(5.0);
            if detour_ratio > effective_threshold {
                let wall_factor = (known_walls.len() as f32) * 30.0;
                let adaptive_iters = ((5000.0 + straight_dist * 150.0 + wall_factor) as usize).min(iters).max(8000);
                eprintln!("  📐 Detour: path={:.0}m, straight={:.0}m, ratio={:.1}x > {:.1} → rebuild ({} iters)",
                    path_len, straight_dist, detour_ratio, effective_threshold, adaptive_iters);
                let hb = object! { "type": "heartbeat", "reason": "detour_rebuild" };
                let _ = writeln!(out, "{}", hb.dump());
                let _ = out.flush();
                match build_smart(&robot, &goal, &known_walls, ranges,
                        args.robot_speed, args.robot_radius, adaptive_iters, args.goal_bias, shortcut_iters, args.lrz_time, args.lsr_max) {
                    Some((ns, np)) => {
                        smart = ns; current_path = np; path_idx = 1;
                        rebuild_count += 1; frames_since_detour_rebuild = 0; detour_fail_streak = 0;
                        cached_inflated = inflate_walls(&known_walls, args.robot_radius);
                        eprintln!("  ✓ Detour rebuild OK");
                    }
                    None => {
                        frames_since_detour_rebuild = 0; detour_fail_streak += 1;
                        eprintln!("  ⚠ Detour rebuild failed (streak={})", detour_fail_streak);
                    }
                }
            }
        }

        // JSON frame
        let mut obs_json = JsonValue::new_array();
        for obs in &visible_ohz {
            let oc = obs.obstacle().center().values();
            obs_json.push(object! { "x": oc[0], "y": oc[1],
                "radius": obs.obstacle().radius(), "ohz_radius": obs.ohz().radius() }).unwrap();
        }

        let mut all_obs_json = JsonValue::new_array();
        for obs in &all_ohz {
            let oc = obs.obstacle().center().values();
            let d = ((rv[0] - oc[0]).powi(2) + (rv[1] - oc[1]).powi(2)).sqrt();
            let in_range = d <= sensor_range + obs.obstacle().radius();
            let vis = in_range && is_visible_through_walls(rv[0], rv[1], oc[0], oc[1], &known_walls);
            all_obs_json.push(object! { "x": oc[0], "y": oc[1], "radius": obs.obstacle().radius(),
                "visible": vis }).unwrap();
        }

	let mut path_json = JsonValue::new_array();
        path_json.push(json::array![rv[0], rv[1]]).unwrap();
        let slice_start = path_idx.min(current_path.len().saturating_sub(1));
        for pt in &current_path[slice_start..] {
            let pv = pt.values();
            path_json.push(json::array![pv[0], pv[1]]).unwrap();
        }
        let mut tree_json = JsonValue::Null;
        if args.include_tree && frame % 5 == 0 {
            let sn = smart.get_nodes();
            let mut edges = JsonValue::new_array();
            for node in sn.iter() {
                if let Some(pi) = node.parent {
                    let pn = sn[pi].state.values();
                    let cn = node.state.values();
                    edges.push(json::array![pn[0], pn[1], cn[0], cn[1], node.tree_id]).unwrap();
                }
            }
            tree_json = edges;
        }

        let mut wv = JsonValue::new_array();
        for &d in &walls_discovered { wv.push(d).unwrap(); }

        // Send grid every 3 frames (compact string: U=unknown, F=free, W=wall)
        let grid_str = if frame % 3 == 0 {
            JsonValue::from(grid_to_string(&scanned_grid))
        } else {
            JsonValue::Null
        };

        let fj = object! {
            "type": "frame", "frame": frame, "time": sim_time,
            "robot": [rv[0], rv[1]], "lrz_radius": lrz_radius, "lsr_max_radius": lsr_max_radius, "sensor_range": sensor_range,
            "obstacles": obs_json, "all_obstacles": all_obs_json,
            "path": path_json, "path_index": path_idx,
            "replanned": replanned, "rebuilds": rebuild_count,
            "walls_known": known_walls.len() as u32,
            "walls_total": scenario.walls.len() as u32,
            "result": format!("{:?}", result),
            "tree": tree_json, "walls_visible": wv,
            "grid": grid_str,
            "num_visible": visible_ohz.len() as u32, "num_total": all_ohz.len() as u32,
        };

        if writeln!(out, "{}", fj.dump()).is_err() { return false; }
        out.flush().unwrap_or(());
        frame += 1;

        // Goal unreachable detection: robot hasn't moved significantly (matches 3D)
        if robot.euclidean_distance(&last_progress_pos) > 2.0 {
            last_progress_pos = robot.clone();
            frames_since_progress = 0;
        } else {
            frames_since_progress += 1;
        }
        if frames_since_progress > 400 {
            let rv_stuck = robot.values();
            let gv_stuck = goal.values();
            let d_stuck = robot.euclidean_distance(&goal);
            eprintln!("  ❌ UNREACHABLE: no progress for {} frames, dist_to_goal={:.1}m",
                frames_since_progress, d_stuck);
            eprintln!("     Robot ({:.1},{:.1}) Goal ({:.1},{:.1})",
                rv_stuck[0], rv_stuck[1], gv_stuck[0], gv_stuck[1]);
            break;
        }

        if robot.euclidean_distance(&goal) <= 1.5 {
            eprintln!("  ✓ GOAL REACHED (frame {}, t={:.1}s)", frame, sim_time);
            break;
        }
        if sim_time >= 600.0 { eprintln!("  Timeout at 600s"); break; }

        let elapsed = t0.elapsed();
        if elapsed < delay { thread::sleep(delay - elapsed); }
    }

    let ej = object! { "type": "end", "scenario": scenario.name.as_str(),
        "total_frames": frame, "total_time": sim_time };
    let _ = writeln!(out, "{}", ej.dump());
    true
}

fn main() {
    // Suppress default panic messages — we catch SMART panics intentionally
    panic::set_hook(Box::new(|_| {}));

    let args = CliArgs::parse();
    let stdout = io::stdout();
    let mut out = stdout.lock();

    let ws = 32.0f32;
    let names: Vec<&str> = if args.scenario == "all_mega" {
        vec!["megacampus", "mega_hospital", "mega_airport", "mega_university"]
    } else if args.scenario == "all" {
        vec!["open", "office", "building", "warehouse"]
    } else {
        vec![args.scenario.as_str()]
    };

    eprintln!("=== SMART-3D RViz2 v3.1 — LiDAR + WallInflation + 3D ===");
    eprintln!("Scenarios: {:?}", names);

    for (i, &name) in names.iter().enumerate() {
        if i > 0 {
            eprintln!("\n--- Pause {:.0}s ---", args.pause_between);
            let pj = object! { "type": "pause", "duration": args.pause_between, "next_scenario": name };
            let _ = writeln!(out, "{}", pj.dump());
            out.flush().unwrap_or(());
            thread::sleep(Duration::from_secs_f32(args.pause_between));
        }
        let s = make_scenario(name, ws);
        if !run_scenario(&s, &args, &mut out) { break; }
    }
    eprintln!("=== All done ===");
}
