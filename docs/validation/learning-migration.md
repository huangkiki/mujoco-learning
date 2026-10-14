# Learning 改名与 Solver 导读：本地验证记录

关联 [Sim Learning #1](https://github.com/huangkiki/sim-learning/issues/1)。本切片基于 `067f2e3f409410ea505d0d3b92f693f46d5560ce`，分支 `autodev/learning-migration`；记录范围为本仓文件与静态验收，远端改名、合入和发布由总仓迁移任务另行核实。

## 交付内容

- 现行标题与系列入口统一为 MuJoCo Learning / Sim Learning，覆盖中英文 README、工程入口、源码地图及现行课程正文；现行自有总仓链接改为 `huangkiki/sim-learning`。
- 新增[Solver 配置导读](../solver-reading-guide.md)，连接 README、课程目录和 roadmap。面向有 VLA/WAM/RL 经验的工程师与硕博，含英文摘要、最小公式、原生配置解读、夹持诊断路径及 6 道带答案练习。
- 覆盖非线性 CG、Newton、PGS，elliptic/pyramidal，Euler/implicitfast；将 `SolverMuJoCo; algorithm unqualified` 保留为实现/路径与证据缺口，未推断其算法。
- 保留 DexLab 3.11.0/3.14.0/3.15.0 历史概览与课程固定 3.15.0 的区别。没有将当前算法细节、默认值或源码支持回填为历史运行证据。

## 静态检查与来源

本地 Python 3.10.12 检查：

```sh
git diff --check
python scripts/check_docs.py
python scripts/check_examples.py
```

以上通过。相对 Markdown 文件链接有效；既有片段检查仍覆盖 **7 个 Python 文件、7 个文档 Python 片段、6 个 XML 文件**。新增导读的 **1 个 XML 配置片段**另用标准库 `ElementTree.fromstring` 解析通过；该片段不含完整模型，解析不是 MJCF schema、模型编译或运行验收。

新增导读的 **12 个官方 blob 链接、6 个源文件**全部指向 MuJoCo 固定提交 `9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5`。下载内容按 Git blob 算法（`sha1("blob " + byte_length + NUL + bytes)`）与已有 [sources.json](../sources.json) 逐一核对一致，所有行锚在实际文件范围内。逐段核对枚举、primal/dual、CG 主编译分支、Newton Hessian、PGS 接触块、摩擦边基、积分/局部导数与终止条件；未改动源码基线或来源 manifest。身份和行号检查不替代上述语义阅读。

DexLab 词表、首页与 MuJoCo 斜面报告冻结在 `7e4a0774efb4f8b332e866e1495e032b4969c8aa`；对应文件路径由 GitHub 内容 API 读回核对。导读只引用报告及其适用范围，没有新算分或复现。一次直接下载斜面报告遇到 SSL 中断，随后读取总仓取得的同提交报告并用 API 核对远端文件存在；未把失败下载当作完成证据。

初次本地链接检查包含了忽略目录中的下载材料，并发现尚未写出的本记录链接；将下载缓存改为非 Markdown 文件并补齐本记录后重新运行，未修改检查器或排除现行课程。最终自审检查了公式假设、教学数值与实验结果的区别、6 道练习、E7 暂停边界以及改名例外。

## 明确保留的历史名称

以下文件保持原内容，本次改名不改写历史身份和原始模型标签：

| 保留位置 | 内容 | 原因 |
|---|---|---|
| [E1 验证](e1.md)、[E2 验证](e2.md)、[E3 验证](e3.md)、[E4 验证](e4.md)、[E5 验证](e5.md)、[E6 验证](e6.md) | `mujoco-atlas/issues/2` 至 `/7`；部分阶段的 Sim Atlas 导航交付表述 | 与当时源码、提交和验证范围绑定的历史记录，字节不变；现行导航使用新名称 |
| [frames.xml](../../examples/modeling-state-time/frames.xml) | `atlas_frames` | 原生模型标识，不是仓库入口 |
| [two_link.xml](../../examples/control-robotics/two_link.xml) | `atlas_two_link` | 原生模型标识，不是仓库入口 |
| [slider.xml](../../examples/batch-learning-data/slider.xml) | `atlas_batch_slider` | 原生模型标识，不是仓库入口 |
| [flex_patch.xml](../../examples/extensions-boundaries/flex_patch.xml) | `atlas_flex_patch` | 原生模型标识，不是仓库入口 |

这些历史文件与模型文件已与本切片 base 逐字节比较一致。课程仓库迁移不改变其行为/身份；旧仓库链接的远端重定向验收由总仓负责，不把本地链接检查记作远程验证。

## 未验证与后续范围

本轮没有原生 MuJoCo/MJX/Warp import、安装、模型编译、forward/step、设备/JIT/渲染、收敛或性能测试、物理实验、训练、sim-to-real 评测。3.11/3.14 历史源码差异没有逐文件重新审校；未知算法标签没有取得新的分支证据。配置解释与手算不证明任何特定任务通过。

E7 完整安装与全路线审校继续暂停。本次局部导读及其 DexLab 链接不等于 E7 的完整证据索引或运行验收完成；后续只按另行授权的范围推进。
