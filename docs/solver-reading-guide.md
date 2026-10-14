# 从 DexLab 表格读懂 MuJoCo：约束算法、摩擦锥与积分器

**建议先修：**先沿 [Physics-Based Simulation 中文伴读路线](https://github.com/huangkiki/sim-learning/blob/main/docs/physics-based-simulation.md)建立离散化、优化、接触和摩擦基础，再读本引擎的具体配置。

**读完能做什么：** 把 `Newton / elliptic / implicitfast` 解释成三个不同层面的选择；区分 MuJoCo 非线性 CG 与线性系统 CG；为一次夹持滑脱列出可核验的诊断证据，并说明哪些结论还不能下。

面向已有 VLA、WAM、RL 经验的工程师及硕博。假定你熟悉向量、梯度与策略 rollout，但无需先读完接触力学教材。本课使用 **MuJoCo 3.15.0 原生 CPU 核心**的固定源码 `9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5`，先讲直觉和最小公式，再连接现有深入课程。所有数值例子均为手算；配置片段只检查 XML 结构，没有导入引擎、编译模型或运行物理。

> **English summary.** This source-grounded guide decodes MuJoCo's nonlinear CG, Newton and PGS constraint algorithms; elliptic/pyramidal friction cones; and Euler/implicitfast integration. It targets engineers and graduate researchers familiar with VLA, WAM and RL. Historical DexLab versions and unresolved `SolverMuJoCo` provenance remain separate from the pinned 3.15.0 native CPU course. Worked reasoning and answered exercises teach configuration interpretation and diagnosis; they do not report new simulation results. Full derivations and implementation details are linked to the existing lessons. E7 installation and runtime review remain paused.

[课程路线](curriculum.md) · [Sim Learning](https://github.com/huangkiki/sim-learning) · [本次验证](validation/learning-migration.md)

## 1. 先拆开这条配置

想象机器人已经学会输出关节目标，但夹起物体后仍缓慢滑落。你打开实验记录，看到 `MuJoCo / Newton / elliptic / implicitfast`。这些标签回答的并非同一个问题：

| DexLab 标签 | 在本课程中的角色 | 应去哪里核对 | 单凭标签不能推出什么 |
|---|---|---|---|
| MuJoCo | 物理核心；还要确定 native、MJX 或其他接入路径 | 核心/绑定版本、运行路径与模型身份 | 所有接入路径执行同一实现 |
| `CG (nonlinear)` | 加速度空间中优化约束代价的非线性共轭梯度 | `opt.solver`，primal 分支 | 正在用线性 CG 解一次固定 `Ax=b` |
| `Newton` | 同一 primal 目标上的 Newton 型约束求解 | `opt.solver`，梯度、Hessian 与线搜索 | 使用了 Newton Physics 引擎，或整个时间步完全隐式 |
| `PGS` | 约束力空间的投影块迭代 | `opt.solver`，dual 分支 | 与其他引擎的同名算法具有相同约束、停止条件和成本 |
| `elliptic` / `pyramidal` | 摩擦可行域及其坐标表示 | `opt.cone`、接触 `dim/friction` | 只换了求解精度，物理/数学问题完全不变 |
| `Euler` / `implicitfast` | 状态随时间推进的积分配置 | `opt.integrator/timestep` | 接触求解已收敛或物理响应准确 |
| `SolverMuJoCo; algorithm unqualified` | 实现/路径标签及算法资格缺口 | 原记录的宿主、适配器、配置读回与分支证据 | 一定执行了原生 Newton、CG 或 PGS |

前三类约束算法在原生枚举中明确标为 PGS **dual**、CG/Newton **primal**；摩擦锥和积分器是独立枚举。见[固定头文件](https://github.com/google-deepmind/mujoco/blob/9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5/include/mujoco/mjtype.h#L185-L211)及 [XML 配置入口](https://github.com/google-deepmind/mujoco/blob/9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5/doc/XMLreference.rst#L409-L465)。

这里解读的词表冻结自 [DexLab 配置分类](https://github.com/huangkiki/Dexlab/blob/7e4a0774efb4f8b332e866e1495e032b4969c8aa/docs/solver-taxonomy.zh-CN.md)和[版本概览](https://github.com/huangkiki/Dexlab/blob/7e4a0774efb4f8b332e866e1495e032b4969c8aa/README.md)。概览把不同批次出现过的名称汇总起来，不保证任意组合都运行过，也不构成性能排名。

## 2. CG、Newton、PGS 到底在求什么

冻结当前姿态、速度与接触集合，先限定为本课的普通连续加速度软约束路径，不含 IPC、adhesion 和 NoSlip 后处理。令 $a_0$ 是不加约束力时的广义加速度，$M$ 是广义惯量矩阵，$J$ 把广义加速度映射到约束空间。约束力 $f$ 会把加速度改成：

$$
a=a_0+M^{-1}J^T f.
$$

夹爪、物体与桌面的约束彼此耦合：改变一处力会通过运动改变别处的约束。求解器要为**整个装配问题**找到一致的近似解。增加迭代可以改善数值求解，却不会自动修正错误的摩擦系数、碰撞几何或控制目标。

CG 与 Newton 可从下面的 primal 目标理解：

$$
\min_a\ \frac12(a-a_0)^TM(a-a_0)+s(Ja-a_r).
$$

第一项约束加速度相对自由运动的偏离；$a_r$ 是模型给定的参考约束加速度，$s$ 是由软约束与力可行域定义的惩罚。目标随接触/摩擦状态切换分段变化，因此“CG”在这里不是只对一个固定二次型应用线性 CG。该式是加速度层优化目标，不直接当作物理能量、reward 或 VLA 训练 loss。[官方优化推导](https://github.com/google-deepmind/mujoco/blob/9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5/doc/computation/index.rst#L1330-L1410)

| 方法 | 本版实现的直觉 | 阅读时需要保留的细节 |
|---|---|---|
| 非线性 CG | 根据预条件梯度与上次方向构造新方向，再沿该方向搜索 | 目标/梯度随当前加速度与约束状态更新；主编译分支用 Hager–Zhang 更新，条件宏可切换 PR+ |
| Newton | 利用当前二阶曲率求方向，再线搜索 | 本版 Hessian 由惯量与约束二阶块组成；Cholesky/增量更新属于实现内部，不是新增一个与 Newton 并列的公开 solver 选项 |
| PGS | 在力空间依次改一个约束块，并投影回允许范围 | elliptic 摩擦是耦合块，不能简化成每个切向独立 clamp；本版还有动量和块访问顺序处理 |

固定实现入口：[CG/Newton 方向与终止](https://github.com/google-deepmind/mujoco/blob/9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5/src/engine/engine_solver.c#L2800-L3000)、[Hessian](https://github.com/google-deepmind/mujoco/blob/9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5/src/engine/engine_solver.c#L2338-L2470)、[PGS](https://github.com/google-deepmind/mujoco/blob/9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5/src/engine/engine_solver.c#L449-L740)。完整 dual 方程、量纲、停止条件与 warmstart 见[深入课：求解与积分](solvers-and-integration.md)。不能从算法结构直接推断哪个在你的训练环境中最快；需要相同任务、明确误差要求与真实成本记录。

## 3. 摩擦锥改变允许的力

先只考虑 `condim=3`、无 adhesion、各向同性滑动摩擦系数 $\mu>0$。elliptic 的圆形截面约束为：

$$
f_n\ge0,\qquad \sqrt{f_{t1}^2+f_{t2}^2}\le\mu f_n.
$$

若 $f_n=10\,\mathrm N$、$\mu=0.5$，切向合力最多 $5\,\mathrm N$。请求 $(f_{t1},f_{t2})=(4,4)\,\mathrm N$ 时，两轴各自都小于 5，合力却约为 5.66，不能满足这个锥。这解释了为什么逐轴夹紧不等于摩擦锥投影。

pyramidal 使用非负的锥边权重重建接触力；在这个三维力例子中，横截面是内接菱形，满足 $|f_{t1}|+|f_{t2}|\le\mu f_n$。手算 $(3,3)\,\mathrm N$ 可以落在上述 elliptic 截面内，却不在该 pyramidal 截面内。因此换锥时允许的力集合、方向性与软约束表示都可能改变，不能把轨迹差异全归因于 solver 数值误差。[官方锥和边基](https://github.com/google-deepmind/mujoco/blob/9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5/doc/computation/index.rst#L1093-L1139)

这里的 $10\,\mathrm N$ 是给定的手算条件，不是从夹爪控制值推断的实际法向力。实际力还取决于几何、驱动、柔顺参数与接触状态。原生 `cone` 与 `solver` 分开选择，PGS/CG/Newton 均有两类锥路径；这不意味着任何 flex/IPC 等组合都支持。[算法范围](https://github.com/google-deepmind/mujoco/blob/9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5/doc/computation/index.rst#L1464-L1490)；更多 `condim`、扭转/滚动摩擦、参数混合和源码中的陈旧措辞见[深入课：接触模型](contact-models.md)。

## 4. Euler 与 implicitfast 回答时间推进问题

对一个没有姿态流形问题的平移自由度，用 $h$ 表示步长，semi-implicit Euler 的基本顺序是：

$$
v_{k+1}=v_k+h a_k,\qquad q_{k+1}=q_k+h v_{k+1}.
$$

MuJoCo 的 `Euler` 先更新速度再更新位置，并在相应开关与模型条件下隐式处理阻尼；不能把它一律解释成全显式 Euler。真实转动状态需要原生位置积分函数，不能逐项相加四元数。[状态推进与 Euler 实现](https://github.com/google-deepmind/mujoco/blob/9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5/src/engine/engine_forward.c#L1470-L1580)

`implicitfast` 在速度相关力的线性化中使用一部分导数，并保持相应全局矩阵的对称形式，省略全局 RNE 速度导数；3.15 的独立自由刚体另有局部陀螺项处理。这里省略的是特定**导数处理**，不能简写成“模拟中删除了所有科氏力或陀螺力”，也不能理解成每个时间步对全部非线性动力学求到精确隐式解。[官方积分说明](https://github.com/google-deepmind/mujoco/blob/9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5/doc/computation/index.rst#L630-L673)、[局部刚体导数条件](https://github.com/google-deepmind/mujoco/blob/9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5/src/engine/engine_derivative.c#L808-L860)

把一个 20 ms 策略动作窗口拆成十个 2 ms 物理步，是本课的时序示例，不是建议默认值。减小 $h$ 改变时间离散；增加 `iterations` 改变每次约束求解预算；提高策略频率改变闭环控制采样。它们对 rollout 的影响不同，不能拿一个参数替代另外两个。更多积分器和有效 metric 的限制见[状态与时间](state-and-time.md)及[求解与积分](solvers-and-integration.md)。

## 5. 读懂一段原生配置，再决定比较什么

下面只是可嵌入 `<mujoco>` 的配置片段，**没有模型与运行证据**。数值是教学示例，不是 DexLab 原始配置或通用调参建议。

```xml
<option solver="Newton" cone="elliptic" integrator="implicitfast"
        timestep="0.002" iterations="100" tolerance="1e-8"/>
```

逐项翻译：用 Newton 求解原生约束优化，采用 elliptic 摩擦锥，按 implicitfast 推进，物理步长 2 ms，主求解迭代上限为 100，并按对应终止条件检查 tolerance。`100` 不表示每步恰好执行 100 轮，`1e-8` 也不是物体位置误差为 $10^{-8}$ m 的保证。CG/Newton 的线搜索还有独立预算；warmstart 满足条件时可以零轮结束。[配置与停止说明](https://github.com/google-deepmind/mujoco/blob/9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5/doc/XMLreference.rst#L435-L465)

**若目标是解释算法差异：** 在同版本、同路径、同模型、同锥、同积分/步长、同驱动与初态条件下，声明只改变 `solver` 及如何约定停止精度，记录各自实际迭代、残差和成本。相同迭代上限不代表相同工作量；更改 `cone` 后还需要单独分析模型变化。

**若目标是为 RL/VLA 任务选配置：** 可以比较完整配置组合，但应明确目标任务、容许误差、失败条件、训练/推理时序与成本，并保留失败。回报较高只说明该任务契约下的结果，无法独立证明材料响应或 sim-to-real 可靠性。现有 [DexLab MuJoCo 斜面报告](https://github.com/huangkiki/Dexlab/blob/7e4a0774efb4f8b332e866e1495e032b4969c8aa/docs/mujoco-incline-results.zh-CN.md)提供带版本与工况的阅读材料；本课未重跑该报告，也不把斜面结论外推到夹持或学习性能。

## 6. 案例诊断：夹持慢滑，先找哪层证据

假设策略能够闭合夹爪，物体随后缓慢下落。以下是**待检验假设与所需证据**，不是本轮观察到的故障或实验结果：

| 次序 | 竞争解释 | 应保留/核对的证据 | 为什么先查它 |
|---|---|---|---|
| 1 | 模型或执行路径与报告不一致 | 核心/绑定/宿主版本、模型 hash、native/MJX/适配器身份、实际配置读回 | 配置文件中的名字不能证明执行路径 |
| 2 | 接触根本没装配，或几何不合适 | 碰撞过滤、contact 数、距离、法向、有效接触行 | solver 不能补救未生成的接触 |
| 3 | 控制输入没有产生预期夹力 | actuator 类型、限幅、传动、目标时序与输出力 | 位置目标不是直接法向力 |
| 4 | 摩擦/柔顺参数或锥改变了问题 | 最终 contact 的 friction/dim/solref/solimp，pair/override，`opt.cone` | 增加求解迭代不等于改变材料模型 |
| 5 | 读数的坐标或时点错了 | 接触作用侧、frame、参考点、力/冲量单位与查询阶段 | 错把法向分量当世界 z 会误判支撑力 |
| 6 | 有限预算、积分或闭环采样影响 | solver 统计、warmstart/NoSlip、步长、控制周期与独立时间戳 | 只有前面条件清楚，误差归因才有含义 |

沿[接触模型](contact-models.md) → [驱动与控制](actuation-and-control.md) → [力观测](force-observations.md) → [求解与积分](solvers-and-integration.md)追到原生字段。若后续实验要比较“加迭代”与“减步长”，先冻结对照协议并分别改变变量；当前只交付诊断路径，不启动已推迟的运行阶段。

## 7. 历史版本与未知标签怎么写

| 身份 | 本课能确认的内容 | 仍需逐条恢复的内容 |
|---|---|---|
| DexLab 概览中的 3.11.0、3.14.0、3.15.0 | 这些版本被列在历史配置概览中 | 哪个任务、二进制、绑定、核心、native/MJWarp 路径与哪个配置对应；概览不是逐批对应表 |
| 课程 3.15.0 | 公式与算法说明固定到上述官方 SHA；文件身份见 [sources.json](sources.json) | 无本机安装、构建或物理运行资格证明 |
| 旧版算法细节 | 可按原批次的源码与 manifest 另行解释 | 本课未逐文件复核 3.11/3.14，因此不将 3.15 的 Hager–Zhang、PGS 动量、默认值或终止条件回填到旧实验 |
| `SolverMuJoCo; algorithm unqualified` | 保留 DexLab 的实现名称和未确认状态 | 需要原宿主/适配器版本、模型/选项读回及实际分支证据，才能给算法身份结论 |

`unqualified` 在这里表示证据缺口，不等于“算法一定错误”或“物理任务不可能”。类名、导入成功、字段存在分别只能证明相应层面的事实。核心、运行路径、模型、solver 的链条必须由同一批次证据连接起来。版本与运行边界详见[版本记录](versions.md)；3.15 原生 CPU 的结论不自动覆盖 MJX JAX、MJX Warp 或外部宿主。

## 8. 阅读练习与答案

**题 1：** `CG / elliptic / Euler` 中哪个是线性系统求解器？

**答：** 这条 MuJoCo 配置没有明确列出线性子求解器。CG 指约束目标的非线性优化方法；elliptic 是摩擦锥，Euler 是积分器。不能因看到 CG 就把这一行与 SuperDex 的线性 CG 并列比较。

**题 2：** 将 `Newton / pyramidal` 改为 `CG / elliptic`，轨迹更稳，能证明 CG 更好吗？

**答：** 不能。求解方法与摩擦表示同时变化。先定义是在选完整配置还是归因算法；后者需要固定问题、时序与误差目标，分别控制变量并检查实际统计。

**题 3：** 给定 $f_n=10$ N、$\mu=0.5$、切向 $(3,3)$ N，它满足本课两种截面吗？

**答：** elliptic 下 $\sqrt{18}\approx4.24\le5$，满足；三维力示例的 pyramidal 下 $3+3=6>5$，不满足。这里只判断手算可行域，不预测实际引擎最终接触力。

**题 4：** `iterations=100`，日志却是 `solver_niter=0`，是否说明没有接触？

**答：** 不能。warmstart 可能已满足停止条件；还要看 contact/约束装配、solver 统计所属岛与配置。迭代上限、实际次数、接触数量是不同量。

**题 5：** `implicitfast` 中的 implicit 与约束 solver `Newton` 是同一件事吗？

**答：** 不是。前者描述速度相关导数如何参与时间推进，后者描述约束优化搜索方向。只有读取实际执行链才能知道各阶段怎样连接，不能由名称推断完全隐式或绝对稳定。

**题 6：** 历史记录只剩 `SolverMuJoCo`，能用当前原生默认 Newton 补齐吗？

**答：** 不能。保留 `algorithm unqualified`，恢复原核心/宿主/适配器版本和实际选项/分支证据。当前默认值或同名类不能替代历史运行事实。

下一步可按需要阅读[完整动力学执行链](dynamics-and-pipeline.md)、[接触模型](contact-models.md)、[求解与积分](solvers-and-integration.md)与[力观测](force-observations.md)。本次导读与品牌迁移属于独立切片；A0 完整安装、原生运行及 E7 全路线审校保持暂停。
