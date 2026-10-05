# articulated-constraint-solver-go

一个确定性、可复现的多刚体关节约束求解器（纯 Go，零外部依赖）。它把多个
刚体通过关节连接成**链式或树形**结构，按独立连通分量（岛屿）划分求解单元，
支持并行推进、休眠唤醒、快照恢复、越界回拉与运行期冲突诊断，并保证：

- 固定随机种子 + 相同输入序列 ⇒ 关节状态与刚体姿态**逐帧一致**；
- 串行与并行岛屿调度的输出**逐位相同**；
- 长时间运行能量不无故增长，不出现 NaN，世界在失败后仍可继续推进。

## 求解语义

求解器采用 **XPBD（Extended Position-Based Dynamics）** 风格的位置级约束求解
与半隐式欧拉积分：

1. **受力积分 / 预测**：对每个醒着的刚体，用累积的力、冲量、扭矩更新线速度
   与角速度（半隐式欧拉），再按速度把位置和姿态预测到 `xPred / qPred`。
2. **位置级约束迭代**：把每个关节展开成若干标量约束行，按**关节 ID 升序**
   做 Gauss-Seidel 扫描。每行在施加修正前都用**当前最新姿态即时重建**，
   因而共享同一刚体的多条约束（链、星型、树）能正确耦合。修正量为

       Δλ = -C / (w + α̃) · relaxation,   α̃ = compliance / dt²

   其中 `w` 是该约束行关于两端刚体的有效逆质量（含杠杆引起的转动项），
   `compliance` 为柔度（0 表示刚性）。
3. **速度回写**：用校正后姿态与预测姿态之差反推线/角速度，再施加阻尼。

角度与位移**限位不施加冲量、不发生突变**：仅当越界时生成一条位置级限位行，
把姿态**平滑回拉**到允许区间内，因此不会穿透或跳变。

### 关节类型

| 类型 | 约束 | 限位 |
| --- | --- | --- |
| `JointRevolute`（旋转/铰链） | 锚点重合；仅保留绕 `Axis` 的一个旋转自由度 | `MinAngle/MaxAngle`（弧度） |
| `JointPrismatic`（滑动） | 仅保留沿 `Axis` 的平移自由度，锁转 | `MinDisplacement/MaxDisplacement`（米） |
| `JointDistance`（距离） | 两锚点保持固定距离（默认取创建时距离） | — |

静态体（`Static: true`）质量与逆惯性为无穷大，不参与岛屿合并：挂在同一静态
体上的多条动态分支互为独立岛屿。

### 岛屿与并行

`CreateBody/CreateJoint` 后用并查集按关节连通性划分岛屿，岛屿以其最小成员
刚体 ID 稳定排序，故划分结果在相同输入下恒定。每个岛屿独立累积时钟
`timeAcc += dt · TimeScale`，各自取整步推进，互不阻塞；`Config.Parallel`
打开时各岛屿在独立 goroutine 中求解，结果按岛屿顺序归并。浮点数计算顺序在
串行与并行下保持一致，因此输出完全相同。

### 休眠与唤醒

当一个岛屿内**所有**刚体的线/角速度连续低于阈值并持续 `SleepDelay` 后，整岛
休眠并清零速度。任一刚体受到冲量、外力或显式 `Wake` 时，沿关节邻接表
（确定性顺序）唤醒**整条连接链**。

### 失败处理

失败通过每步返回的 `*StepReport.Failures` 上报，均带可区分的 `FailureKind`：

- `FailureInvalidParameter`（构造期同时返回 error）；
- `FailureSingular`：约束行有效权重近似为零却仍有残差（奇异构型，如退化轴线）；
- `FailureConflictingConstraints`：迭代结束后仍有非限位行残差超阈值
  （互相矛盾的约束，例如同对刚体上长度互斥的刚性距离约束）；
- `FailureNaNDetected`：检测到非有限值，该岛屿本步**回滚到步前姿态**。

任何情况下世界都保持有限值并可继续 `Step()`；构造期非法输入（负质量、NaN
姿态、零轴、限位上下界颠倒等）立即报错且不改变世界。

### 快照与复现

`Snapshot()` 以定长小端序序列化全部状态（配置、刚体、关节、缓存逆质量/惯性、
休眠计时、岛屿时钟、ID 计数器、流逝时间、RNG 流位置），`Restore()` 还原后与
未中断运行**逐帧一致**。确定性来自：固定宽度整数/浮点编码、升序遍历、纯整数
PCG 随机数、无 map 迭代依赖、无时间/调度相关分支。

## 配置项（`Config` / `DefaultConfig`）

| 字段 | 默认值 | 含义 |
| --- | --- | --- |
| `TimeStep` | `1/60 s` | 参考时钟固定步长（必须 > 0） |
| `Iterations` | `8` | XPBD Gauss-Seidel 扫描次数（必须 > 0） |
| `Relaxation` | `1.0` | 逐行修正的 SOR 松弛系数，合法 `(0,2]`；参数变化不破坏已收敛姿态 |
| `Compliance` | `0` | 默认柔度（逆刚度），0 为刚性 |
| `LinearSleepSpeed` | `0.02` | 休眠线速度阈值（m/s） |
| `AngularSleepSpeed` | `0.02` | 休眠角速度阈值（rad/s） |
| `SleepDelay` | `0.5 s` | 低速持续多久后休眠 |
| `LinearDamping` / `AngularDamping` | `0.05` | 每秒速度衰减比例 `[0,1]`，乘子为 `(1-d)^dt` |
| `MaxLinearSpeed` / `MaxAngularSpeed` | `100` | 速度幅值钳制，防爆冲量；`0` 关闭 |
| `RandomSeed` | `1` | PCG 随机种子，相同种子结果一致 |
| `Parallel` | `true` | 是否并行求解各岛屿（不影响结果） |

`BodyDef.TimeScale`（默认 1）设置刚体所属岛屿的时钟倍率。

## 快速开始

```go
cfg := acsolver.DefaultConfig()
cfg.RandomSeed = 42
w, _ := acsolver.NewWorld(cfg)

base, _ := w.CreateBody(acsolver.BodyDef{Static: true, Position: acsolver.Vec3{Y: 2}})
arm, _  := w.CreateBody(acsolver.BodyDef{Position: acsolver.Vec3{X: 1, Y: 2}, Mass: 1,
    InertiaDiag: acsolver.Vec3{1, 1, 1}})
_, _ = w.CreateJoint(acsolver.JointDef{
    Kind: acsolver.JointRevolute, BodyA: base, BodyB: arm,
    Axis: acsolver.Vec3{Z: 1}, AnchorB: acsolver.Vec3{X: 1},
    HasAngleLimit: true, MinAngle: -1, MaxAngle: 1,
})

for i := 0; i < 240; i++ {
    report := w.Step()
    if len(report.Failures) > 0 { /* 分类处理 */ }
}

snap, _ := w.Snapshot()      // 运行中快照
_ = w.Restore(snap)          // 精确恢复
st := w.State()              // 确定性状态导出（姿态/速度/关节量）
```

## 测试

```sh
go test ./...        # 链式/树形、越界回拉、休眠唤醒、快照、冲突、复现、并行
go test -race ./...  # 竞态检测
```
