# FUTURE OPTIMIZATION: Deferred Eager + Local Wall Query

> 下一步要实现的核心优化。当前系统能用，但远距离 Eager repair 浪费算力且路径质量差。

---

## 1. 当前系统的问题

Agent A 在左上角发现墙 W，Agent B 在右下角距离 80m。当前系统立刻对 B 的树做 Eager repair：15000 条边 × 300 面墙 = 450 万次碰撞检查，22ms。

问题：
- **算力浪费**：W 距离 B 有 80m，对 B 当前路段没有影响
- **路径质量差**：在 80m 外做 repair，那片区域节点稀疏，hot node 重连出来的路径绕弯
- **绕路无法自动修正**：除非遇到新墙再触发 repair，绕路会一直走完

---

## 2. 新方案：Deferred Eager + Local Wall Query

核心思想：**知道所有墙（checker 包含全部），但只 repair 附近的墙。**

### 2.1 流程

```
Agent A 发现墙 W → 加入全局 vec（和现在一样）

Agent B：
  → Lazy swap 更新 checker（checker 包含所有墙，0ms，和现在一样）
  → path_affected 只检查 B 前方 30m 内的墙（改动点 1）
     → 远处的墙穿路径了？不管，不触发 repair
     → 近处的墙穿路径了？触发 Eager
  → Eager repair 只传入 30m 内的墙（改动点 2）
     → 15000 × 20 = 30 万次碰撞检查，约 1.5ms
     → hot node 搜索在墙附近，节点更密，路径质量更高
  → B 继续走，逐步靠近远处的墙
  → 走到 30m 范围内 → path_affected 检测到穿墙 → 再次 Eager
  → 大方向始终朝 goal，细节逐步修正

全量 rebuild 时（stuck / repair 失败）：
  → 用完整 checker（包含全部 300 面墙）
  → 新树从一开始就知道所有墙
  → 路径是基于完整地图的全局最优
```

### 2.2 为什么 checker 要保留全部墙

两层设计：

- **repair 层**：只看附近的墙，省算力，路径质量高
- **rebuild 层**：用完整 checker，拿到全局最优路径

Lazy swap 更新 checker 的代价是 0ms，但让 rebuild 自动受益于所有已知墙壁。免费的保险。

### 2.3 为什么 30m

- 大于 LiDAR 扫描范围（10~15m），robot 自己发现的墙一定在范围内
- 小于 LSR（80m），不会白白 repair 太远的墙
- 以 0.5m/s 速度走 30m 需要 60 秒，给了足够提前量
- 可调参，也可设为 LSR 的一半

---

## 3. 空间索引 v2 的复用

当前 grid bucket 只用在 `is_edge_valid()`（每条边查穿过哪些墙）。

新增用途：local wall query（robot 附近 30m 有哪些墙）。

```rust
// 已有：is_edge_valid 用的
fn walls_near_edge(&self, p1: &[f32;2], p2: &[f32;2]) -> impl Iterator<Item=&Wall>

// 新增：local wall query 用的
fn walls_in_radius(&self, center: &[f32;2], radius: f32) -> Vec<&Wall> {
    // 计算 center ± radius 覆盖的 grid cells
    // 返回这些 cells 里的所有墙（允许重复，不分配去重 Vec）
}
```

同一个 grid bucket 数据结构，两种 query 方式。

---

## 4. 代码改动清单

### 4.1 smart_multi_agent.rs（仿真）

改动 1：`path_affected` 加距离过滤
```rust
// 现在：
let path_affected = agent.current_path.windows(2).any(|pair| {
    cached_inflated.iter().any(|w| w.intersects_edge(&pair[0], &pair[1]))
});

// 改为：
let robot_pos = agent.robot.values();
let local_walls: Vec<_> = cached_inflated.iter().filter(|w| {
    let cx = (w.min_corner().values()[0] + w.max_corner().values()[0]) / 2.0;
    let cy = (w.min_corner().values()[1] + w.max_corner().values()[1]) / 2.0;
    let dx = cx - robot_pos[0];
    let dy = cy - robot_pos[1];
    dx*dx + dy*dy < LOCAL_HORIZON * LOCAL_HORIZON  // LOCAL_HORIZON = 30.0
}).collect();

let path_affected = agent.current_path.windows(2).any(|pair| {
    local_walls.iter().any(|w| w.intersects_edge(&pair[0], &pair[1]))
});
```

改动 2：Eager repair 传入 local walls
```rust
// 现在：
let vc = Box::new(SpatialRectValidityChecker::new(cached_inflated.clone(), 4.0));

// 改为（仅 Eager 时）：
let vc_local = Box::new(SpatialRectValidityChecker::new(local_inflated.clone(), 4.0));
// 注意：Lazy swap 的 checker 仍然用完整 cached_inflated
```

改动 3：rebuild 保持用完整 checker（不改，已经是这样）

### 4.2 spatial_checker.rs

新增 `walls_in_radius()` 方法，复用现有 grid bucket。

### 4.3 新增 CLI 参数

```
--local-horizon 30.0    # 局部规划范围（米）
```

---

## 5. Benchmark 计划

对比两个配置：

| 配置 | path_affected | Eager scope |
|------|---------------|-------------|
| A（当前） | 全部墙 | 全部墙 |
| B（deferred） | 30m 内 | 30m 内 |

跑 1000 trials × 6 scenarios × 2 configs = 12000 runs。

度量：SimMed、WallMed、Rebuilds、IncOK、路径长度比（path_length / euclidean_distance）。

路径长度比是新的关键指标，之前没有测过。

---

## 6. 风险和边界情况

1. **30m 截断点刚好在一面墙上**：用墙中心点判断属于哪一边，不做精确裁剪。偶尔多查一面墙不影响正确性。

2. **Eager repair 用 local walls 但 checker 有全部墙**：repair 内部 `is_edge_valid()` 用的是 checker（全部墙），碰撞检测仍然完整。local walls 只影响"哪些边需要被检查"，不影响碰撞检测精度。安全。

3. **king/flee 不受影响**：king/flee 看的是 agent 之间的实时距离，不是路径点。

---

*Felix Fang — UConn LinksLab — March 2026*
