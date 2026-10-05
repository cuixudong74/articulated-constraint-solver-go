# articulated-constraint-solver-go

一个可复现的 2D 多刚体关节约束求解器：把多个刚体用关节连接成链式或树形结构，
通过顺序冲量（sequential impulse）求解约束并推进姿态。长时间运行稳定、
确定性可复现，支持并行岛屿求解、休眠唤醒与快照恢复。

## 快速开始

```go
cfg := solver.DefaultConfig()
w := solver.NewWorld(solver.Vec2{X: 0, Y: -10}, cfg)

anchor := solver.NewStaticBody(0, solver.Vec2{X: 0, Y: 5})
w.AddBody(anchor)
link := solver.NewBody(1, solver.Vec2{X: 0, Y: 4.5}, 1, 0.1)
w.AddBody(link)

w.AddJoint(&solver.Revolute{
    // A/B 通过嵌入的 jointBase 设置；使用导出的构造函数见下
})
```

关节通过结构体字面量构造，`A`/`B` 来自内嵌的 `jointBase`（字段导出）：

```go
j := &solver.Revolute{}
j.A, j.B = anchor, link
j.LocalAnchorB = solver.Vec2{X: 0, Y: 0.5}
j.EnableLimit = true
j.LowerAngle, j.UpperAngle = -0.5, 0.5
w.AddJoint(j)

for {
    _ = w.Step(1.0 / 60.0) // err 报告奇异/冲突，但世界仍可继续推进
}
```

## 求解语义

- **积分**：半隐式欧拉。每步顺序为：外力积分 → 岛屿划分 → 关节初始化
  → 热启动 → 速度迭代 → 位置积分 → 位置修正（NGS 伪冲量）→ 休眠更新。
- **速度层**：迭代求解 `C·v = 0`（等式）或不等式（限位），冲量累加并热启动，
  不向系统注入能量。
- **位置层**：直接修正位置/角度，误差扣除 slop 后按 `Relaxation` 缩放、
  按 `MaxLinear/AngularCorrection` 截断。越界关节被平滑回拉，不会穿透或突变。
- **关节类型**：
  - `Revolute`：两个局部锚点重合，可选相对角度限位 `[LowerAngle, UpperAngle]`。
  - `Prismatic`：沿 A 系固定轴滑动，垂直方向与相对转角被锁定，
    可选位移限位 `[LowerTranslation, UpperTranslation]`。
  - `Distance`：锚点间距保持在 `[MinLength, MaxLength]`，等长时表现为刚性杆。
- **岛屿并行**：互不相连的关节组（连通分量）被划分为独立岛屿，按最小刚体 ID
  确定性排序，由工作队列分发给 goroutine 并行求解。岛屿间不共享可写状态
  （静态刚体永不写入），因此并行与串行结果逐位一致，且快慢岛屿互不阻塞。
- **休眠**：连通分量整体休眠/唤醒。分量内所有刚体速度低于阈值并持续
  `SleepTime` 后整体休眠，不再参与计算；对任一刚体施加冲量
  （`ApplyImpulse`）或调用 `WakeIsland` 会连带唤醒整条连接链。
- **快照**：`Snapshot()` 捕获全部刚体状态、休眠计时器与关节热启动累加器；
  `Restore()` 后以相同输入序列推进，结果与未中断运行逐帧一致（浮点逐位相等）。
- **可复现性**：固定配置、固定输入序列下，所有计算顺序确定，输出逐帧一致。

## 配置项（Config）

| 字段 | 默认 | 含义 |
|---|---|---|
| `VelocityIterations` | 10 | 速度冲量迭代次数 |
| `PositionIterations` | 8 | 位置修正迭代次数 |
| `Relaxation` | 0.5 | 位置误差松弛系数 (0,1]，越小越柔和 |
| `LinearSlop` / `AngularSlop` | 0.005 / 0.01 | 位置/角度容差，低于此值不修正 |
| `MaxLinearCorrection` / `MaxAngularCorrection` | 0.5 / 0.5 | 单次迭代最大修正量 |
| `SleepLinearThreshold` / `SleepAngularThreshold` | 0.05 / 0.05 | 休眠速度阈值 |
| `SleepTime` | 0.5 | 低于阈值持续多少秒后休眠 |
| `EnableSleep` | true | 是否启用休眠 |
| `ParallelWorkers` | 4 | 岛屿并行求解的 goroutine 上限（≤1 为串行） |

参数可在运行中直接修改 `w.Config`；每次 `Step` 前会自动把参数截断到
安全范围，参数变化不会破坏已收敛状态的稳定性。

## 错误处理

`Step`/`AddJoint`/`AddBody` 返回 `*solver.Error`，`Kind` 可区分：

- `ErrKindInvalidParameter`：NaN/Inf 状态、重复 ID、非法限位区间、零长度轴、
  非法时间步等，在 API 边界拒绝。
- `ErrKindSingular`：奇异构型（如两个不可动刚体之间的关节、距离关节锚点重合）。
  出错关节被跳过，其余关节正常求解。
- `ErrKindConflicting`：位置约束在硬阈值（1m / 1rad）内仍不可满足，
  判定为冲突约束（如互不兼容的距离约束）。世界保持可继续推进。

任何情况下世界状态都不会出现 NaN；若数值异常发生，速度被清零并返回错误，
世界仍可继续 `Step`。

## 测试

```sh
go test ./...          # 单元与集成测试
go test -race ./...    # 竞态检测（并行岛屿）
```

覆盖：链式关节连接保持、限位越界回拉、休眠与连带唤醒、快照恢复逐帧一致、
并行/串行确定性、冲突构型、奇异与非法参数、长时间能量稳定（100s 无增长）。
