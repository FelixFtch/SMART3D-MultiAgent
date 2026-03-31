/// SMART-3D Planner Server v2 — arbitrary coordinate bounds
///
/// Key change from v1: accepts `bounds:[xmin,ymin,xmax,ymax]` instead of world_size.
/// This means Cartographer map-frame coordinates work directly — no offset needed.
///
/// Protocol (JSON stdin/stdout):
///
///   BUILD — full RRT* rebuild with known walls:
///   {"cmd":"build","from":[x,y],"goal":[x,y],"walls":[[x1,y1,x2,y2],...],"bounds":[-15,-15,15,15],"robot_speed":0.2,"rrt_iters":8000}
///
///   UPDATE — SMART prune/repair for dynamic obstacles (fast, ~5ms):
///   {"cmd":"update","robot":[x,y],"path_idx":0,"obstacles":[{"x":1,"y":2,"r":0.3,"speed":0.5},...]}
///
///   PATH — get current path:
///   {"cmd":"path"}
///
///   QUIT:
///   {"cmd":"quit"}

use json::{self, object, JsonValue};
use smart_3d::obstacles::{
    AnalyticValidityChecker, RectangularObstacle, SphericalObstacle,
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
use std::io::{self, BufRead, Write};
use std::panic;
use std::time::Duration;

struct PlannerState {
    smart: Option<SMART<f32, 2, KdTreeNearestNeighbors<f32, 2>>>,
    path: Vec<RealVectorState<f32, 2>>,
    walls: Vec<StaticRectangularObstacle<f32, 2>>,
    bounds: [f32; 4], // [xmin, ymin, xmax, ymax]
    robot_speed: f32,
}

impl PlannerState {
    fn new() -> Self {
        PlannerState {
            smart: None,
            path: Vec::new(),
            walls: Vec::new(),
            bounds: [-15.0, -15.0, 15.0, 15.0],
            robot_speed: 0.2,
        }
    }

    fn build(
        &mut self,
        from: [f32; 2],
        goal: [f32; 2],
        walls: Vec<StaticRectangularObstacle<f32, 2>>,
        bounds: [f32; 4],
        robot_speed: f32,
        rrt_iters: usize,
    ) -> Result<Vec<[f32; 2]>, String> {
        self.bounds = bounds;
        self.robot_speed = robot_speed;
        self.walls = walls;

        // Clamp from/goal to bounds with margin
        let margin = 0.3;
        let clamp = |v: f32, lo: f32, hi: f32| v.max(lo + margin).min(hi - margin);
        let from_c = [
            clamp(from[0], bounds[0], bounds[2]),
            clamp(from[1], bounds[1], bounds[3]),
        ];
        let goal_c = [
            clamp(goal[0], bounds[0], bounds[2]),
            clamp(goal[1], bounds[1], bounds[3]),
        ];

        let start = RealVectorState::new(from_c);
        let goal_state = RealVectorState::new(goal_c);

        // Sampling ranges directly from bounds — no offset needed!
        let ranges = [
            (bounds[0], bounds[2]),
            (bounds[1], bounds[3]),
        ];

        let static_vc: Box<dyn ValidityChecker<f32, 2>> = if self.walls.is_empty() {
            Box::new(AlwaysValid::new())
        } else {
            Box::new(AnalyticValidityChecker::new(self.walls.clone()))
        };

        let area = (ranges[0].1 - ranges[0].0) * (ranges[1].1 - ranges[1].0);
        let gamma = optimal_gamma(area, 2);

        let mut rrt_star: RRTstar<f32, 2, KdTreeNearestNeighbors<f32, 2>> = RRTstar::new(
            goal_state.clone(),
            start.clone(),
            0.25,       // step_size
            static_vc,
            Box::new(UniformDistribution::new(ranges)),
            1.0,       // goal_bias not used directly
            1.7,       // rewire_factor
            gamma,
        );

        let mut term = MaxIterationsTermination::new(rrt_iters);
        rrt_star.plan_until(&mut term);

        let (nodes, nn, svc, sd) = rrt_star.deconstruct_into_components();
        let lrz_radius = robot_speed * 1.0;

        let mut smart = SMART::new_from_initial_rrt_star_tree(
            &nodes,
            nn,
            start.clone(),
            goal_state.clone(),
            0.40,       // robot_radius (your car is 400x350mm → ~0.27m)
            svc,
            sd,
            lrz_radius,
            1.0,        // ohz_time
            1.5,        // rrt_star_rewire_factor
            10.0,       // max_edge_length
            1.7,        // gamma1
            1.7,        // gamma2
            false,       // use_disjoint_tree
            0.01,       // path_node_merge_distance
        );

        smart.initial_plan_add_node_at_start();
        smart.initial_plan_build_path();

        match smart.get_path() {
            Some(p) => {
                let pts: Vec<[f32; 2]> = p.iter().map(|s| {
                    let v = s.values();
                    [v[0], v[1]]
                }).collect();
                self.path = p;
                self.smart = Some(smart);
                Ok(pts)
            }
            None => {
                self.smart = None;
                self.path.clear();
                Err("no_path".to_string())
            }
        }
    }

    fn update(
        &mut self,
        robot: [f32; 2],
        path_idx: usize,
        obstacles: &[([f32; 2], f32, f32)], // (center, radius, speed)
    ) -> (String, Option<Vec<[f32; 2]>>) {
        let smart = match self.smart.as_mut() {
            Some(s) => s,
            None => return ("no_planner".to_string(), None),
        };

        let robot_state = RealVectorState::new(robot);
        let ohz_time = 0.5;

        let mut ohz_list = Vec::new();
        for &(center, radius, speed) in obstacles {
            let obs = StaticSphericalObstacle::new(
                RealVectorState::new(center), radius,
            );
            let ohz_radius = radius + speed * ohz_time;
            let ohz = StaticSphericalObstacle::new(
                RealVectorState::new(center), ohz_radius,
            );
            ohz_list.push(smart_3d::smart::SphericalObstacleWithOhz::new(obs, ohz));
        }

        let mut rt = MaxTimeTermination::new(Duration::from_millis(50));

        let result = panic::catch_unwind(panic::AssertUnwindSafe(|| {
            smart.update(robot_state, path_idx, &ohz_list, Some(&mut rt))
        }));

        match result {
            Ok(SmartUpdateResult::NoReplanningNeeded) => ("safe".to_string(), None),
            Ok(SmartUpdateResult::ReplanningSuccessful) => {
                if let Some(p) = smart.get_path() {
                    let pts: Vec<[f32; 2]> = p.iter().map(|s| {
                        let v = s.values();
                        [v[0], v[1]]
                    }).collect();
                    self.path = p;
                    ("replanned".to_string(), Some(pts))
                } else {
                    ("replanned_no_path".to_string(), None)
                }
            }
            Ok(SmartUpdateResult::ReplanningFailed) => ("replan_failed".to_string(), None),
            Ok(SmartUpdateResult::InCollision) => ("in_collision".to_string(), None),
            Ok(_) => ("unknown".to_string(), None),
            Err(_) => {
                self.smart = None;
                ("panic".to_string(), None)
            }
        }
    }

    fn get_path_pts(&self) -> Vec<[f32; 2]> {
        self.path.iter().map(|s| {
            let v = s.values();
            [v[0], v[1]]
        }).collect()
    }
}

fn parse_walls(arr: &JsonValue) -> Vec<StaticRectangularObstacle<f32, 2>> {
    let mut walls = Vec::new();
    for w in arr.members() {
        if w.len() >= 4 {
            let x1 = w[0].as_f32().unwrap_or(0.0);
            let y1 = w[1].as_f32().unwrap_or(0.0);
            let x2 = w[2].as_f32().unwrap_or(0.0);
            let y2 = w[3].as_f32().unwrap_or(0.0);
            walls.push(StaticRectangularObstacle::new(
                RealVectorState::new([x1.min(x2), y1.min(y2)]),
                RealVectorState::new([x1.max(x2), y1.max(y2)]),
            ));
        }
    }
    walls
}

fn path_to_json(pts: &[[f32; 2]]) -> JsonValue {
    let mut arr = JsonValue::new_array();
    for p in pts {
        arr.push(json::array![p[0], p[1]]).unwrap();
    }
    arr
}

fn main() {
    panic::set_hook(Box::new(|_| {})); // suppress panic output

    let stdin = io::stdin();
    let stdout = io::stdout();
    let mut out = stdout.lock();

    let mut state = PlannerState::new();

    eprintln!("[smart_planner_server] v2 ready (arbitrary bounds)");

    for line in stdin.lock().lines() {
        let line = match line {
            Ok(l) => l,
            Err(_) => break,
        };
        let line = line.trim().to_string();
        if line.is_empty() { continue; }

        let data = match json::parse(&line) {
            Ok(d) => d,
            Err(e) => {
                let r = object! { "ok": false, "error": format!("parse: {}", e) };
                let _ = writeln!(out, "{}", r.dump());
                let _ = out.flush();
                continue;
            }
        };

        let cmd = data["cmd"].as_str().unwrap_or("");

        let response = match cmd {
            "build" => {
                let from = [
                    data["from"][0].as_f32().unwrap_or(0.0),
                    data["from"][1].as_f32().unwrap_or(0.0),
                ];
                let goal = [
                    data["goal"][0].as_f32().unwrap_or(1.0),
                    data["goal"][1].as_f32().unwrap_or(1.0),
                ];
                let bounds = if data["bounds"].len() >= 4 {
                    [
                        data["bounds"][0].as_f32().unwrap_or(-15.0),
                        data["bounds"][1].as_f32().unwrap_or(-15.0),
                        data["bounds"][2].as_f32().unwrap_or(15.0),
                        data["bounds"][3].as_f32().unwrap_or(15.0),
                    ]
                } else {
                    let ws = data["world_size"].as_f32().unwrap_or(20.0);
                    [-ws / 2.0, -ws / 2.0, ws / 2.0, ws / 2.0]
                };
                let speed = data["robot_speed"].as_f32().unwrap_or(0.2);
                let iters = data["rrt_iters"].as_usize().unwrap_or(8000);
                let walls = parse_walls(&data["walls"]);

                eprintln!("[build] from={:?} goal={:?} walls={} bounds={:?}",
                         from, goal, walls.len(), bounds);

                match state.build(from, goal, walls, bounds, speed, iters) {
                    Ok(pts) => {
                        eprintln!("[build] OK — {} pts", pts.len());
                        object! {
                            "ok": true,
                            "path": path_to_json(&pts),
                            "num_walls": state.walls.len(),
                        }
                    }
                    Err(e) => {
                        eprintln!("[build] FAIL: {}", e);
                        object! { "ok": false, "error": e.as_str() }
                    }
                }
            }

            "update" => {
                let robot = [
                    data["robot"][0].as_f32().unwrap_or(0.0),
                    data["robot"][1].as_f32().unwrap_or(0.0),
                ];
                let path_idx = data["path_idx"].as_usize().unwrap_or(0);

                let mut obstacles = Vec::new();
                for o in data["obstacles"].members() {
                    let cx = o["x"].as_f32().unwrap_or(0.0);
                    let cy = o["y"].as_f32().unwrap_or(0.0);
                    let r = o["r"].as_f32().unwrap_or(0.3);
                    let s = o["speed"].as_f32().unwrap_or(0.5);
                    obstacles.push(([cx, cy], r, s));
                }

                let (result, new_path) = state.update(robot, path_idx, &obstacles);

                let mut resp = object! {
                    "ok": true,
                    "result": result.as_str(),
                };
                if let Some(pts) = new_path {
                    resp["path"] = path_to_json(&pts);
                }
                if result == "panic" {
                    resp["ok"] = false.into();
                    resp["error"] = "smart_panic".into();
                    eprintln!("[update] SMART panicked — need rebuild");
                }
                resp
            }

            "path" => {
                object! { "ok": true, "path": path_to_json(&state.get_path_pts()) }
            }

            "quit" => {
                let _ = writeln!(out, "{}", object! { "ok": true }.dump());
                let _ = out.flush();
                break;
            }

            _ => object! { "ok": false, "error": format!("unknown: {}", cmd) }
        };

        let _ = writeln!(out, "{}", response.dump());
        let _ = out.flush();
    }
}
