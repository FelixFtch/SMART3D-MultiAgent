#!/bin/bash
# =================================================================
# SMART-3D Real-Car Autonomous Navigation — Pi 5 Deployment
# =================================================================
# Creates all files needed for SMART-3D autonomous navigation:
#   1. Robot URDF model (rectangular chassis + LiDAR)
#   2. Rust planning server (smart_planner_server)
#   3. Python navigation node (smart_nav_node.py)
#   4. cmd_vel motor driver (cmd_vel_driver.py)
#   5. Startup script (start_smart_nav.sh)
#
# Run on Pi 5 host: chmod +x deploy_smart_nav.sh && ./deploy_smart_nav.sh
# =================================================================
set -e

echo "============================================"
echo " SMART-3D Real-Car Navigation Deployment"
echo "============================================"

# ================================================================
# 1. URDF Robot Model — 400mm x 350mm rectangle + LiDAR on top
# ================================================================
echo ""
echo "[1/5] Installing robot URDF model..."

docker exec ros2-humble bash -c 'mkdir -p /root/robotcar_description && cat > /root/robotcar_description/robotcar.urdf << "URDFEOF"
<?xml version="1.0" ?>
<robot name="robotcar">
  <link name="base_footprint"/>
  <joint name="footprint_joint" type="fixed">
    <parent link="base_footprint"/><child link="base_link"/>
    <origin xyz="0 0 0.06" rpy="0 0 0"/>
  </joint>
  <link name="base_link">
    <visual><geometry><box size="0.350 0.400 0.120"/></geometry>
    <material name="chassis"><color rgba="0.25 0.25 0.28 1.0"/></material></visual>
    <collision><geometry><box size="0.350 0.400 0.120"/></geometry></collision>
    <inertial><mass value="3.5"/>
    <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.04" iyz="0" izz="0.06"/></inertial>
  </link>
  <joint name="lf_joint" type="continuous">
    <parent link="base_link"/><child link="lf_wheel"/>
    <origin xyz="0.175 0.200 -0.01" rpy="-1.5708 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <link name="lf_wheel"><visual><geometry><cylinder radius="0.050" length="0.040"/></geometry>
    <material name="blk"><color rgba="0.1 0.1 0.1 1"/></material></visual></link>
  <joint name="rf_joint" type="continuous">
    <parent link="base_link"/><child link="rf_wheel"/>
    <origin xyz="0.175 -0.200 -0.01" rpy="-1.5708 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <link name="rf_wheel"><visual><geometry><cylinder radius="0.050" length="0.040"/></geometry>
    <material name="blk"/></visual></link>
  <joint name="lr_joint" type="continuous">
    <parent link="base_link"/><child link="lr_wheel"/>
    <origin xyz="-0.175 0.200 -0.01" rpy="-1.5708 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <link name="lr_wheel"><visual><geometry><cylinder radius="0.050" length="0.040"/></geometry>
    <material name="blk"/></visual></link>
  <joint name="rr_joint" type="continuous">
    <parent link="base_link"/><child link="rr_wheel"/>
    <origin xyz="-0.175 -0.200 -0.01" rpy="-1.5708 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <link name="rr_wheel"><visual><geometry><cylinder radius="0.050" length="0.040"/></geometry>
    <material name="blk"/></visual></link>
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/><child link="laser_link"/>
    <origin xyz="0 0 0.050" rpy="0 0 0"/>
  </joint>
  <link name="laser_link"><visual><geometry><cylinder radius="0.035" length="0.045"/></geometry>
    <material name="lidar_blue"><color rgba="0.1 0.4 0.8 1"/></material></visual></link>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/><child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <link name="imu_link"/>
</robot>
URDFEOF
echo "  URDF installed"'

# ================================================================
# 2. Rust Planning Server
# ================================================================
echo ""
echo "[2/5] Installing Rust smart_planner_server..."

docker exec ros2-humble bash -c 'cat > /root/SMART3D/examples/smart_planner_server.rs << "RUSTEOF"
/// SMART-3D Real-Time Planning Server
/// Reads JSON commands from stdin, outputs JSON results to stdout.
/// Driven by Python ROS2 node that feeds real sensor data.
///
/// Commands:
///   {"cmd":"build","from":[x,y],"goal":[x,y],"walls":[[x1,y1,x2,y2],...],"world_size":20.0,"speed":0.3,"iters":3000}
///   {"cmd":"update","pos":[x,y],"path_idx":0,"obstacles":[{"x":1,"y":2,"radius":0.3,"speed":0.5}]}
///   {"cmd":"get_path"}
///   {"cmd":"quit"}

use json::{self, object, JsonValue};
use smart_3d::obstacles::{
    AnalyticValidityChecker, StaticRectangularObstacle, StaticSphericalObstacle,
    SphericalObstacle,
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

mod scenarios;

fn wall(x1: f32, y1: f32, x2: f32, y2: f32) -> StaticRectangularObstacle<f32, 2> {
    StaticRectangularObstacle::new(
        RealVectorState::new([x1.min(x2), y1.min(y2)]),
        RealVectorState::new([x1.max(x2), y1.max(y2)]),
    )
}

struct Planner {
    smart: Option<SMART<f32, 2, KdTreeNearestNeighbors<f32, 2>>>,
    path: Vec<RealVectorState<f32, 2>>,
    goal: RealVectorState<f32, 2>,
    walls: Vec<StaticRectangularObstacle<f32, 2>>,
    ws: f32,
    spd: f32,
    iters: usize,
    rebuilds: u32,
}

impl Planner {
    fn new() -> Self {
        Planner { smart: None, path: vec![], goal: RealVectorState::new([0.0,0.0]),
                  walls: vec![], ws: 20.0, spd: 0.3, iters: 3000, rebuilds: 0 }
    }

    fn build(&mut self, from: [f32;2], goal: [f32;2], walls_raw: Vec<[f32;4]>,
             ws: f32, spd: f32, iters: usize) -> bool {
        self.ws = ws; self.spd = spd; self.iters = iters;
        self.goal = RealVectorState::new(goal);
        self.walls = walls_raw.iter().map(|w| wall(w[0],w[1],w[2],w[3])).collect();
        let from_s = RealVectorState::new(from);
        let ranges = [(0.0, ws), (0.0, ws)];
        match self.do_build(&from_s, &self.goal.clone(), ranges) {
            Some((s, p)) => { self.smart = Some(s); self.path = p; self.rebuilds += 1; true }
            None => false
        }
    }

    fn do_build(&self, from: &RealVectorState<f32,2>, goal: &RealVectorState<f32,2>,
                ranges: [(f32,f32);2])
        -> Option<(SMART<f32,2,KdTreeNearestNeighbors<f32,2>>, Vec<RealVectorState<f32,2>>)>
    {
        let vc: Box<dyn ValidityChecker<f32,2>> = if self.walls.is_empty() {
            Box::new(AlwaysValid::new())
        } else {
            Box::new(AnalyticValidityChecker::new(self.walls.clone()))
        };
        let area = (ranges[0].1-ranges[0].0)*(ranges[1].1-ranges[1].0);
        let gamma = optimal_gamma(area, 2);
        let mut rrt: RRTstar<f32,2,KdTreeNearestNeighbors<f32,2>> = RRTstar::new(
            goal.clone(), from.clone(), 1.0, vc,
            Box::new(UniformDistribution::new(ranges)), 1.0, 1.7, gamma);
        let mut term = MaxIterationsTermination::new(self.iters);
        rrt.plan_until(&mut term);
        let (nodes, nn, svc, sd) = rrt.deconstruct_into_components();
        let lrz = self.spd * 1.0;
        let mut smart = SMART::new_from_initial_rrt_star_tree(
            &nodes, nn, from.clone(), goal.clone(), 1.5, svc, sd, lrz,
            1.0, 1.5, 10.0, 1.7, 1.7, false, 0.01);
        smart.initial_plan_add_node_at_start();
        smart.initial_plan_build_path();
        let path = smart.get_path()?;
        Some((smart, path))
    }

    fn update(&mut self, pos: [f32;2], pidx: usize, obs: &[(f32,f32,f32,f32)]) -> (String, bool) {
        let smart = match self.smart.as_mut() { Some(s)=>s, None=>return("no_planner".into(),false) };
        let robot = RealVectorState::new(pos);
        let ohz_time = 0.5;
        let ohz_obs: Vec<_> = obs.iter().map(|&(x,y,r,spd)| {
            let c = StaticSphericalObstacle::new(RealVectorState::new([x,y]), r);
            let o = StaticSphericalObstacle::new(RealVectorState::new([x,y]), r+spd*ohz_time);
            smart_3d::smart::obstacle::SmartObstacle::new(c, o)
        }).collect();
        let mut rt = MaxTimeTermination::new(Duration::from_millis(50));
        let result = panic::catch_unwind(panic::AssertUnwindSafe(|| {
            smart.update(robot.clone(), pidx, &ohz_obs, Some(&mut rt))
        }));
        match result {
            Ok(SmartUpdateResult::ReplanningSuccessful) => {
                if let Some(p) = smart.get_path() { self.path = p; ("replanned".into(), true) }
                else { ("replan_no_path".into(), false) }
            }
            Ok(SmartUpdateResult::NoReplanningNeeded) => ("no_change".into(), false),
            Ok(SmartUpdateResult::ReplanningFailed) => ("replan_failed".into(), false),
            Ok(_) => ("other".into(), false),
            Err(_) => {
                eprintln!("[server] SMART panic — rebuilding");
                let ranges = [(0.0, self.ws), (0.0, self.ws)];
                let g = self.goal.clone();
                match self.do_build(&robot, &g, ranges) {
                    Some((s,p)) => { self.smart=Some(s); self.path=p; self.rebuilds+=1;
                                     ("rebuilt".into(), true) }
                    None => ("rebuild_failed".into(), false)
                }
            }
        }
    }

    fn path_json(&self) -> JsonValue {
        let mut a = JsonValue::new_array();
        for pt in &self.path { let v=pt.values(); a.push(json::array![v[0],v[1]]).unwrap(); }
        a
    }
}

fn main() {
    panic::set_hook(Box::new(|_| {}));
    let stdin = io::stdin(); let stdout = io::stdout(); let mut out = stdout.lock();
    let mut p = Planner::new();
    eprintln!("[smart_planner_server] Ready");
    let _ = writeln!(out, "{}", object!{"status":"ready"}.dump()); out.flush().unwrap();

    for line in stdin.lock().lines() {
        let line = match line { Ok(l)=>l, Err(_)=>break };
        let line = line.trim().to_string();
        if line.is_empty() { continue; }
        let data = match json::parse(&line) {
            Ok(d)=>d, Err(e)=> {
                let _ = writeln!(out,"{}",object!{"status":"error","msg":format!("{}",e)}.dump());
                out.flush().unwrap(); continue;
            }
        };
        let cmd = data["cmd"].as_str().unwrap_or("");
        match cmd {
            "build" => {
                let from = [data["from"][0].as_f32().unwrap_or(0.0), data["from"][1].as_f32().unwrap_or(0.0)];
                let goal = [data["goal"][0].as_f32().unwrap_or(0.0), data["goal"][1].as_f32().unwrap_or(0.0)];
                let ws = data["world_size"].as_f32().unwrap_or(20.0);
                let spd = data["speed"].as_f32().unwrap_or(0.3);
                let iters = data["iters"].as_usize().unwrap_or(3000);
                let mut walls = vec![];
                if data["walls"].is_array() {
                    for w in data["walls"].members() {
                        if w.len()>=4 { walls.push([w[0].as_f32().unwrap_or(0.0),w[1].as_f32().unwrap_or(0.0),
                                                     w[2].as_f32().unwrap_or(0.0),w[3].as_f32().unwrap_or(0.0)]); }
                    }
                }
                eprintln!("[server] BUILD from={:?} goal={:?} walls={} iters={}", from, goal, walls.len(), iters);
                if p.build(from, goal, walls, ws, spd, iters) {
                    let _ = writeln!(out,"{}",object!{"status":"ok","path":p.path_json(),"rebuilds":p.rebuilds}.dump());
                } else {
                    let _ = writeln!(out,"{}",object!{"status":"no_path"}.dump());
                }
            }
            "update" => {
                let pos = [data["pos"][0].as_f32().unwrap_or(0.0), data["pos"][1].as_f32().unwrap_or(0.0)];
                let pidx = data["path_idx"].as_usize().unwrap_or(0);
                let mut obs = vec![];
                if data["obstacles"].is_array() {
                    for o in data["obstacles"].members() {
                        obs.push((o["x"].as_f32().unwrap_or(0.0), o["y"].as_f32().unwrap_or(0.0),
                                  o["radius"].as_f32().unwrap_or(0.3), o["speed"].as_f32().unwrap_or(0.5)));
                    }
                }
                let (st, changed) = p.update(pos, pidx, &obs);
                let mut r = object!{"status":st.as_str(),"rebuilds":p.rebuilds};
                if changed { r["path"] = p.path_json(); }
                let _ = writeln!(out,"{}",r.dump());
            }
            "get_path" => { let _ = writeln!(out,"{}",object!{"status":"ok","path":p.path_json(),"rebuilds":p.rebuilds}.dump()); }
            "quit" => { let _ = writeln!(out,"{}",object!{"status":"bye"}.dump()); break; }
            _ => { let _ = writeln!(out,"{}",object!{"status":"unknown_cmd"}.dump()); }
        }
        out.flush().unwrap();
    }
}
RUSTEOF
echo "  smart_planner_server.rs installed"'

# Compile
echo "  Compiling smart_planner_server (may take 1-2 min)..."
docker exec ros2-humble bash -lc '
  export PATH="/root/.cargo/bin:$PATH"
  cd /root/SMART3D
  cargo build --release --example smart_planner_server 2>&1 | tail -5
  if [ -f target/release/examples/smart_planner_server ]; then
    echo "  Compilation OK"
  else
    echo "  ⚠ Check errors above"
  fi
'

# ================================================================
# 3. Python Navigation Node
# ================================================================
echo ""
echo "[3/5] Installing smart_nav_node.py..."

docker exec ros2-humble python3 -c "
code = '''#!/usr/bin/env python3
\"\"\"SMART-3D Real-Car Navigation — subscribes /scan + TF, drives via /cmd_vel\"\"\"
import math, json, subprocess, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf2_ros

def rgba(r,g,b,a=1.0):
    c=ColorRGBA(); c.r,c.g,c.b,c.a=float(r),float(g),float(b),float(a); return c

class OccGrid:
    def __init__(self, size_m=20.0, res=0.1, ox=-10.0, oy=-10.0):
        self.res=res; self.ox=ox; self.oy=oy
        self.w=int(size_m/res); self.h=int(size_m/res)
        self.data=np.full(self.w*self.h, -1, dtype=np.int8)

    def update(self, rx, ry, yaw, scan):
        angle=scan.angle_min
        for r in scan.ranges:
            if r<scan.range_min or r>scan.range_max or math.isnan(r) or math.isinf(r):
                angle+=scan.angle_increment; continue
            ra=yaw+angle; ca=math.cos(ra); sa=math.sin(ra)
            step=self.res*0.5; steps=int(r/step)
            for s in range(steps):
                t=s*step; gx=int((rx+t*ca-self.ox)/self.res); gy=int((ry+t*sa-self.oy)/self.res)
                if 0<=gx<self.w and 0<=gy<self.h: self.data[gy*self.w+gx]=0
            gx=int((rx+r*ca-self.ox)/self.res); gy=int((ry+r*sa-self.oy)/self.res)
            if 0<=gx<self.w and 0<=gy<self.h: self.data[gy*self.w+gx]=100
            angle+=scan.angle_increment

    def extract_walls(self, min_cluster=3):
        occ=(self.data.reshape(self.h,self.w)==100).astype(np.uint8)
        walls=[]; visited=np.zeros_like(occ)
        for y in range(self.h):
            for x in range(self.w):
                if occ[y,x] and not visited[y,x]:
                    q=[(x,y)]; visited[y,x]=1; cells=[]
                    while q:
                        cx,cy=q.pop(0); cells.append((cx,cy))
                        for dx,dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                            nx,ny=cx+dx,cy+dy
                            if 0<=nx<self.w and 0<=ny<self.h and occ[ny,nx] and not visited[ny,nx]:
                                visited[ny,nx]=1; q.append((nx,ny))
                    if len(cells)>=min_cluster:
                        xs=[c[0] for c in cells]; ys=[c[1] for c in cells]
                        pad=0.05
                        walls.append([min(xs)*self.res+self.ox-pad, min(ys)*self.res+self.oy-pad,
                                     (max(xs)+1)*self.res+self.ox+pad, (max(ys)+1)*self.res+self.oy+pad])
        return walls

    def to_msg(self, stamp):
        m=OccupancyGrid(); m.header.stamp=stamp; m.header.frame_id=\"map\"
        m.info.resolution=self.res; m.info.width=self.w; m.info.height=self.h
        m.info.origin.position.x=self.ox; m.info.origin.position.y=self.oy
        m.info.origin.orientation.w=1.0; m.data=self.data.tolist(); return m

class ObsDetector:
    def __init__(self, cdist=0.5, minp=3):
        self.cdist=cdist; self.minp=minp; self.prev=[]

    def detect(self, rx, ry, yaw, scan):
        if scan is None: return []
        pts=[]; angle=scan.angle_min
        for r in scan.ranges:
            if scan.range_min<r<scan.range_max and not math.isnan(r):
                pts.append((rx+r*math.cos(yaw+angle), ry+r*math.sin(yaw+angle)))
            angle+=scan.angle_increment
        if not pts: return []
        clusters=[]; used=[False]*len(pts)
        for i in range(len(pts)):
            if used[i]: continue
            cl=[pts[i]]; used[i]=True
            for j in range(i+1,len(pts)):
                if used[j]: continue
                px,py=pts[j]
                for cx,cy in cl:
                    if (px-cx)**2+(py-cy)**2<self.cdist**2:
                        cl.append(pts[j]); used[j]=True; break
            if len(cl)>=self.minp:
                xs=[p[0] for p in cl]; ys=[p[1] for p in cl]
                cx=sum(xs)/len(xs); cy=sum(ys)/len(ys)
                rad=max(max(abs(x-cx) for x in xs), max(abs(y-cy) for y in ys))+0.1
                if rad<1.0: clusters.append((cx,cy,rad))
        obs=[]
        for cx,cy,rad in clusters:
            speed=0.0; best_d=2.0
            for px,py,_ in self.prev:
                d=math.sqrt((cx-px)**2+(cy-py)**2)
                if d<best_d: best_d=d; speed=d/0.1
            if speed>0.1: obs.append((cx,cy,rad,speed))
        self.prev=clusters
        return obs

class PurePursuit:
    def __init__(self, look=0.5, maxv=0.3, maxw=1.2, tol=0.4):
        self.look=look; self.maxv=maxv; self.maxw=maxw; self.tol=tol

    def compute(self, rx, ry, yaw, path, pidx):
        if not path or pidx>=len(path): return 0,0,pidx,True
        ti=pidx
        for i in range(pidx,len(path)):
            dx=path[i][0]-rx; dy=path[i][1]-ry
            if math.sqrt(dx*dx+dy*dy)>=self.look: ti=i; break
        else: ti=len(path)-1
        while pidx<len(path)-1:
            dx=path[pidx][0]-rx; dy=path[pidx][1]-ry
            if math.sqrt(dx*dx+dy*dy)<self.look*0.5: pidx+=1
            else: break
        gx,gy=path[-1]
        if math.sqrt((gx-rx)**2+(gy-ry)**2)<self.tol: return 0,0,pidx,True
        tx,ty=path[ti]; dx=tx-rx; dy=ty-ry
        ae=math.atan2(dy,dx)-yaw
        while ae>math.pi: ae-=2*math.pi
        while ae<-math.pi: ae+=2*math.pi
        if abs(ae)>0.8:
            v=0.0; w=self.maxw*(1 if ae>0 else -1)
        else:
            v=self.maxv*(1-abs(ae)/1.5); v=max(0.05,min(v,self.maxv))
            w=2*v*math.sin(ae)/max(self.look,0.1); w=max(-self.maxw,min(w,self.maxw))
        return v,w,pidx,False

class SmartNavNode(Node):
    def __init__(self):
        super().__init__(\"smart_nav_node\")
        self.declare_parameter(\"world_size\", 20.0)
        self.declare_parameter(\"grid_resolution\", 0.1)
        self.declare_parameter(\"robot_speed\", 0.3)
        self.declare_parameter(\"rrt_iterations\", 3000)
        self.declare_parameter(\"wall_threshold\", 5)

        self.ws=self.get_parameter(\"world_size\").value
        self.gres=self.get_parameter(\"grid_resolution\").value
        self.spd=self.get_parameter(\"robot_speed\").value
        self.iters=self.get_parameter(\"rrt_iterations\").value
        self.wthresh=self.get_parameter(\"wall_threshold\").value

        self.rx=0.0; self.ry=0.0; self.ryaw=0.0; self.got_pose=False
        self.goal=None; self.path=[]; self.pidx=0
        self.navigating=False; self.prev_wcount=0; self.last_build=0.0
        self.last_scan=None

        half=self.ws/2
        self.grid=OccGrid(self.ws, self.gres, -half, -half)
        self.obs_det=ObsDetector()
        self.pursuit=PurePursuit(look=0.5, maxv=self.spd, maxw=1.2, tol=0.4)

        self.tf_buf=tf2_ros.Buffer()
        self.tf_lis=tf2_ros.TransformListener(self.tf_buf, self)

        sq=QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, \"/scan\", self.on_scan, sq)
        self.create_subscription(PoseStamped, \"/goal_pose\", self.on_goal, 10)

        self.cmd_pub=self.create_publisher(Twist, \"/cmd_vel\", 10)
        self.path_pub=self.create_publisher(Path, \"/smart/path\", 10)
        self.marker_pub=self.create_publisher(MarkerArray, \"/smart/markers\", 10)
        lq=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub=self.create_publisher(OccupancyGrid, \"/smart/grid\", lq)

        self.proc=None; self.plock=threading.Lock()
        self.start_planner()

        self.create_timer(0.1, self.nav_loop)
        self.create_timer(1.0, self.pub_grid)
        self.create_timer(0.2, self.pub_viz)
        self.get_logger().info(f\"SMART Nav started ws={self.ws} spd={self.spd}\")

    def start_planner(self):
        try:
            self.proc=subprocess.Popen(
                [\"/root/SMART3D/target/release/examples/smart_planner_server\"],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, bufsize=1)
            line=self.proc.stdout.readline().strip()
            d=json.loads(line)
            if d.get(\"status\")==\"ready\":
                self.get_logger().info(\"Rust planner connected\")
            else:
                self.get_logger().error(f\"Planner: {line}\")
        except Exception as e:
            self.get_logger().error(f\"Cannot start planner: {e}\")
            self.proc=None

    def pcmd(self, cmd):
        if not self.proc: return {\"status\":\"no_planner\"}
        try:
            with self.plock:
                self.proc.stdin.write(json.dumps(cmd)+\"\\n\")
                self.proc.stdin.flush()
                line=self.proc.stdout.readline().strip()
                return json.loads(line) if line else {\"status\":\"empty\"}
        except Exception as e:
            self.get_logger().error(f\"Planner error: {e}\")
            return {\"status\":\"error\"}

    def get_pose(self):
        try:
            t=self.tf_buf.lookup_transform(\"map\",\"base_link\",rclpy.time.Time())
            self.rx=t.transform.translation.x; self.ry=t.transform.translation.y
            q=t.transform.rotation
            self.ryaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.got_pose=True; return True
        except: return False

    def on_scan(self, msg):
        self.last_scan=msg
        if not self.get_pose(): return
        self.grid.update(self.rx, self.ry, self.ryaw, msg)

    def on_goal(self, msg):
        gx=msg.pose.position.x; gy=msg.pose.position.y
        self.goal=(gx,gy); self.navigating=True; self.pidx=0
        self.get_logger().info(f\"Goal: ({gx:.2f},{gy:.2f})\")
        self.build_plan()

    def build_plan(self):
        if not self.goal or not self.got_pose: return
        walls=self.grid.extract_walls()
        self.prev_wcount=len(walls)
        r=self.pcmd({\"cmd\":\"build\",\"from\":[self.rx,self.ry],\"goal\":list(self.goal),
                      \"walls\":walls,\"world_size\":self.ws,\"speed\":self.spd,\"iters\":self.iters})
        if r.get(\"status\")==\"ok\":
            self.path=r.get(\"path\",[]); self.pidx=0
            self.get_logger().info(f\"Plan: {len(self.path)} pts, {len(walls)} walls, rebuild #{r.get(\\\"rebuilds\\\",0)}\")
        else:
            self.get_logger().warn(f\"Plan failed: {r.get(\\\"status\\\")}\")
        self.last_build=time.time()

    def nav_loop(self):
        if not self.navigating or not self.goal or not self.got_pose: return
        walls=self.grid.extract_walls()
        wd=abs(len(walls)-self.prev_wcount)
        if wd>=self.wthresh and time.time()-self.last_build>2.0:
            self.get_logger().info(f\"New walls ({self.prev_wcount}->{len(walls)}), rebuilding\")
            self.build_plan(); return
        obs=self.obs_det.detect(self.rx,self.ry,self.ryaw,self.last_scan) if self.last_scan else []
        if obs and self.path:
            od=[{\"x\":o[0],\"y\":o[1],\"radius\":o[2],\"speed\":o[3]} for o in obs]
            r=self.pcmd({\"cmd\":\"update\",\"pos\":[self.rx,self.ry],\"path_idx\":self.pidx,\"obstacles\":od})
            if r.get(\"status\")==\"replanned\":
                self.path=r.get(\"path\",self.path); self.pidx=0
                self.get_logger().info(\"SMART replanned\")
        if not self.path: return
        v,w,ni,reached=self.pursuit.compute(self.rx,self.ry,self.ryaw,self.path,self.pidx)
        self.pidx=ni
        if reached:
            self.get_logger().info(\"GOAL REACHED!\"); self.navigating=False; v,w=0.0,0.0
        t=Twist(); t.linear.x=v; t.angular.z=w; self.cmd_pub.publish(t)
        pm=Path(); pm.header.frame_id=\"map\"; pm.header.stamp=self.get_clock().now().to_msg()
        for pt in self.path:
            ps=PoseStamped(); ps.header=pm.header
            ps.pose.position.x=float(pt[0]); ps.pose.position.y=float(pt[1])
            ps.pose.orientation.w=1.0; pm.poses.append(ps)
        self.path_pub.publish(pm)

    def pub_grid(self):
        self.grid_pub.publish(self.grid.to_msg(self.get_clock().now().to_msg()))

    def pub_viz(self):
        ma=MarkerArray(); s=self.get_clock().now().to_msg()
        if self.goal:
            m=Marker(); m.header.frame_id=\"map\"; m.header.stamp=s
            m.ns=\"goal\"; m.id=0; m.type=Marker.CYLINDER; m.action=Marker.ADD
            m.pose.position.x=self.goal[0]; m.pose.position.y=self.goal[1]; m.pose.position.z=0.1
            m.pose.orientation.w=1.0; m.scale.x=0.5; m.scale.y=0.5; m.scale.z=0.2
            m.color=rgba(1,0.2,0.2,0.8); ma.markers.append(m)
        m=Marker(); m.header.frame_id=\"map\"; m.header.stamp=s
        m.ns=\"status\"; m.id=0; m.type=Marker.TEXT_VIEW_FACING; m.action=Marker.ADD
        m.pose.position.x=self.rx; m.pose.position.y=self.ry; m.pose.position.z=0.5
        m.pose.orientation.w=1.0; m.scale.z=0.15; m.color=rgba(1,1,1)
        if self.navigating and self.goal:
            d=math.sqrt((self.goal[0]-self.rx)**2+(self.goal[1]-self.ry)**2)
            m.text=f\"NAV d={d:.1f}m {self.pidx}/{len(self.path)}\"
        else: m.text=\"IDLE - set goal in RViz\"
        ma.markers.append(m)
        self.marker_pub.publish(ma)

    def destroy_node(self):
        t=Twist(); self.cmd_pub.publish(t)
        if self.proc:
            try: self.pcmd({\"cmd\":\"quit\"}); self.proc.wait(timeout=2)
            except: self.proc.kill()
        super().destroy_node()

def main():
    rclpy.init(); node=SmartNavNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()

if __name__==\"__main__\": main()
'''
with open('/root/smart_nav_node.py','w') as f: f.write(code)
print('  smart_nav_node.py installed')
"

# ================================================================
# 4. cmd_vel Motor Driver
# ================================================================
echo ""
echo "[4/5] Installing cmd_vel_driver.py..."

docker exec ros2-humble python3 -c "
code = '''#!/usr/bin/env python3
\"\"\"cmd_vel -> DDSM115 RS485 motor driver for differential drive.
Subscribes /cmd_vel, sends RPM commands to 4 motors via RS485.
Motor IDs: Left front=1, Left rear=3, Right front=2, Right rear=4
Right side reversed. Wheel radius=0.05m, track width=0.4m.
\"\"\"
import math, struct, threading, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

MAX_RPM = 80
TIMEOUT = 0.5  # stop if no cmd_vel for 0.5s

def crc8(data):
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01: crc = (crc >> 1) ^ 0x8C
            else: crc >>= 1
    return crc

class CmdVelDriver(Node):
    def __init__(self):
        super().__init__(\"cmd_vel_driver\")
        self.wr = 0.050  # wheel radius
        self.tw = 0.400  # track width
        self.ser = None
        self.lk = threading.Lock()
        self.last_cmd = time.time()
        self.target_left = 0
        self.target_right = 0

        try:
            self.ser = serial.Serial(\"/dev/ttyUSB0\", 115200, timeout=0.01)
            self.get_logger().info(\"RS485 opened /dev/ttyUSB0\")
        except Exception as e:
            self.get_logger().error(f\"Cannot open RS485: {e}\")

        self.create_subscription(Twist, \"/cmd_vel\", self.on_cmd, 10)
        self.create_timer(0.05, self.send_motors)  # 20Hz motor update
        self.get_logger().info(\"cmd_vel driver started\")

    def on_cmd(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        # Differential drive: v_left = v - w*L/2, v_right = v + w*L/2
        vl = v - w * self.tw / 2
        vr = v + w * self.tw / 2
        # Convert m/s to RPM: rpm = v / (2*pi*r) * 60
        rpm_l = vl / (2 * math.pi * self.wr) * 60
        rpm_r = vr / (2 * math.pi * self.wr) * 60
        # Clamp
        self.target_left = int(max(-MAX_RPM, min(MAX_RPM, rpm_l)))
        self.target_right = int(max(-MAX_RPM, min(MAX_RPM, rpm_r)))
        self.last_cmd = time.time()

    def set_rpm(self, mid, rpm):
        if not self.ser: return
        # DDSM115 speed command: [ID, 0x64, rpm_h, rpm_l, 0,0,0,0,0, CRC]
        rpm_bytes = struct.pack(\">h\", int(rpm))
        frame = bytes([mid, 0x64]) + rpm_bytes + bytes([0, 0, 0, 0, 0])
        frame = frame + bytes([crc8(frame)])
        with self.lk:
            try:
                self.ser.write(frame)
            except:
                pass

    def send_motors(self):
        # Safety timeout
        if time.time() - self.last_cmd > TIMEOUT:
            self.target_left = 0
            self.target_right = 0

        # Left motors (IDs 1,3) — positive RPM = forward
        self.set_rpm(1, self.target_left)
        self.set_rpm(3, self.target_left)
        # Right motors (IDs 2,4) — reversed mounting, negate RPM
        self.set_rpm(2, -self.target_right)
        self.set_rpm(4, -self.target_right)

def main():
    rclpy.init()
    node = CmdVelDriver()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        # Stop motors
        node.target_left = 0; node.target_right = 0
        node.send_motors()
        node.destroy_node(); rclpy.try_shutdown()

if __name__ == \"__main__\": main()
'''
with open('/root/cmd_vel_driver.py','w') as f: f.write(code)
print('  cmd_vel_driver.py installed')
"

# ================================================================
# 5. Startup Script
# ================================================================
echo ""
echo "[5/5] Installing start_smart_nav.sh..."

docker exec ros2-humble python3 -c "
code = '''#!/bin/bash
# ===================================================
# SMART-3D Autonomous Navigation — Full Stack Startup
# ===================================================
# Starts: SLAM + SMART planner + motor driver + nav node
# Usage: /root/start_smart_nav.sh [world_size] [speed]
# ===================================================
WORLD_SIZE=\${1:-20.0}
SPEED=\${2:-0.3}

echo \"========================================\"
echo \" SMART-3D Autonomous Navigation\"
echo \" World: \${WORLD_SIZE}m  Speed: \${SPEED}m/s\"
echo \"========================================\"

# Kill old
pkill -9 -f \"cartograph|lslidar|talker|imu_qos|ddsm115|static_transform|smart_nav|cmd_vel_driver|smart_planner\" 2>/dev/null || true
sleep 2
rm -rf /dev/shm/fastrtps_* /dev/shm/*fastdds* /dev/shm/*SHM* /tmp/*.log 2>/dev/null

# Devices
mknod /dev/ttyACM3 c 166 3 2>/dev/null || true
chmod 666 /dev/ttyACM3 2>/dev/null || true
mknod /dev/ttyUSB0 c 188 0 2>/dev/null || true
chmod 666 /dev/ttyUSB0 2>/dev/null || true
/root/mk_imu_dev.sh 2>/dev/null || true
/root/mk_m10p_dev.sh 2>/dev/null || true

export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source /root/m10p_ws/install/setup.bash
source /root/imu_ws/install/setup.bash

echo \"[1/8] Static TF...\"
ros2 run tf2_ros static_transform_publisher 0 0 0.05 0 0 0 base_link laser_link &
sleep 2

echo \"[2/8] Robot model (URDF)...\"
ros2 run robot_state_publisher robot_state_publisher \\
    --ros-args -p robot_description:=\"$(cat /root/robotcar_description/robotcar.urdf)\" &
sleep 1

echo \"[3/8] LiDAR...\"
ros2 run lslidar_driver lslidar_driver_node --ros-args --params-file /root/m10p_ws/m10p_uart.yaml > /tmp/lidar.log 2>&1 &
sleep 3

echo \"[4/8] IMU...\"
ros2 run hipnuc_imu talker --ros-args -p serial_port:=/dev/imu -p baud_rate:=115200 -p imu_topic:=/IMU_data -p frame_id:=base_link > /tmp/imu_talker.log 2>&1 &
sleep 2

echo \"[5/8] IMU QoS bridge...\"
python3 /root/imu_qos_bridge.py > /tmp/imu_bridge.log 2>&1 &
sleep 3

echo \"[6/8] Wheel odometry...\"
python3 /root/ddsm115_odom.py > /tmp/odom.log 2>&1 &
sleep 2

echo \"[7/8] Cartographer SLAM...\"
ros2 launch /root/cartographer_config/cartographer_launch.py config_dir:=/root/cartographer_config config_basename:=robotcar_2d_odom.lua > /tmp/cartographer.log 2>&1 &
sleep 8

echo \"[8/8] cmd_vel motor driver...\"
python3 /root/cmd_vel_driver.py > /tmp/cmdvel.log 2>&1 &
sleep 1

echo \"\"
echo \"========== VERIFICATION ==========\"
echo \"=== SCAN ===\" && timeout 3s ros2 topic hz /scan 2>&1 | head -2
echo \"=== TF ===\" && timeout 3s ros2 topic hz /tf 2>&1 | head -2
echo \"=== ODOM ===\" && tail -2 /tmp/odom.log
echo \"==================================\"
echo \"\"
echo \"System ready! Now start navigation node in another terminal:\"
echo \"  docker exec -it ros2-humble bash -lc \\\"source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=42 && python3 /root/smart_nav_node.py --ros-args -p world_size:=${WORLD_SIZE} -p robot_speed:=${SPEED}\\\"\"
echo \"\"
echo \"Then in RViz: use 2D Goal Pose to set destination.\"
echo \"Press Ctrl+C to stop all.\"
wait
'''
with open('/root/start_smart_nav.sh','w') as f: f.write(code)
import os; os.chmod('/root/start_smart_nav.sh', 0o755)
print('  start_smart_nav.sh installed')
"

echo ""
echo "============================================"
echo " Deployment Complete!"
echo "============================================"
echo ""
echo "=== 使用方法 ==="
echo ""
echo "终端1 — 启动系统 (SLAM + 传感器 + 电机驱动):"
echo "  docker restart ros2-humble && sleep 5"
echo "  docker exec -it ros2-humble bash -lc '/root/start_smart_nav.sh 20.0 0.3'"
echo ""
echo "终端2 — 启动导航节点 (SMART-3D 规划):"
echo "  docker exec -it ros2-humble bash -lc '"
echo "    source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=42 &&"
echo "    python3 /root/smart_nav_node.py --ros-args -p world_size:=20.0 -p robot_speed:=0.3'"
echo ""
echo "终端3 — RViz (可视化 + 设置目标点):"
echo "  docker exec -it ros2-gui bash -lc '"
echo "    source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=42 &&"
echo "    export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_udp.xml &&"
echo "    export LIBGL_ALWAYS_SOFTWARE=1 && export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe &&"
echo "    export QT_X11_NO_MITSHM=1 && rviz2'"
echo ""
echo "RViz 操作:"
echo "  1. Fixed Frame = map"
echo "  2. Add: /map, /scan(Best Effort), TF, RobotModel"
echo "  3. Add: Path → /smart/path"
echo "  4. Add: MarkerArray → /smart/markers"
echo "  5. Add: Map → /smart/grid"
echo "  6. 点击工具栏 '2D Goal Pose' 设置导航目标"
echo "  → 小车自动规划路径并导航过去！"
