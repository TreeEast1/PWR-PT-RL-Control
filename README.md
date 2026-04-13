# 高精度快速负荷跟踪控制：PC-1 工况下压水堆的预设时间收敛仿真

## 项目标题

**High-precision Rapid Load-following Control of PWR under PC-1**

## 项目简介

本项目实现了一个用于压水堆（PWR）负荷跟踪研究的 Python 仿真平台，面向论文 *High-precision rapid load following control strategy of a PWR under Plant Condition 1* 所描述的问题场景进行工程化复现。系统围绕 PC-1（Plant Condition 1）随机工况展开，重点考虑三类模式切换：正常运行、轻微燃料元件损伤、以及许可范围内的蒸汽发生器泄漏。

为描述上述随机突变工况，项目采用带马尔可夫跳变参数的四状态堆芯模型，状态变量为：

- `n_r`：相对中子功率
- `c_r`：相对缓发中子先驱核密度
- `T_f`：平均燃料温度
- `T_l`：平均冷却剂出口温度

在控制设计上，项目实现了三层协同结构：

- 固定时间稳定微分器（FSD），用于在存在测量噪声时恢复 `T_l` 及其导数信息；
- 有限时间状态观测器，用于在线重构 `c_r` 与 `T_f`；
- 预设时间稳定控制器（P-T Controller），使功率跟踪误差在用户可配置的时间参数 `T_p` 下快速压缩。

仿真工况为 100% 满功率（FP）到 50% FP 的线性降功率过程，速率为 `0.0025 FP/s`，并用 `scipy.integrate.solve_ivp` 完成闭环数值积分。

## 系统架构

系统由“堆芯模型 - 观测器 - 控制器”三部分构成，数据流如下：

```text
           +---------------------------+
           |   Markov PC-1 Mode Chain  |
           | Normal / Fuel / SG Leak   |
           +-------------+-------------+
                         |
                         v
+------------------------+------------------------+
|                   Reactor Model                 |
|     states: n_r, c_r, T_f, T_l                  |
|     nonlinear kinetics + thermal feedback       |
+------------------------+------------------------+
                         |
                noisy T_l measurement
                         |
                         v
+------------------------+------------------------+
|         Fixed-time Stable Differentiator        |
|      output: T_l estimate and dT_l/dt           |
+------------------------+------------------------+
                         |
                         v
+------------------------+------------------------+
|          Finite-time State Observer             |
|         estimate: c_r hat, T_f hat              |
+------------------------+------------------------+
                         |
                         v
+------------------------+------------------------+
|        Preassigned-time Stable Controller       |
|    input: tracking error + estimated states     |
|    output: control reactivity                   |
+------------------------+------------------------+
                         |
                         v
                  back to reactor
```

## 数学基础

### 1. 四方程压水堆堆芯模型

项目采用一群点堆动力学与集中参数热工水力模型耦合形式：

```math
\dot{n}_r = \frac{\rho - \beta}{\Lambda} n_r + \lambda c_r
```

```math
\dot{c}_r = \frac{\beta}{\Lambda} n_r - \lambda c_r
```

```math
\dot{T}_f = q_f(n_r,\delta) - \frac{T_f - T_l}{\tau_f}
```

```math
\dot{T}_l = k_c (T_f - T_l) - g(\delta)\frac{T_l - T_{in}}{\tau_l}
```

其中：

- `rho = u + alpha_f (T_f - T_f^*) + alpha_l (T_l - T_l^*) + rho_delta`
- `u` 为控制反应性输入；
- `delta` 为马尔可夫跳变模式；
- `rho_delta`、热交换系数和冷却损失系数随模式切换而变化。

### 2. 马尔可夫跳变参数

项目建立三模态 PC-1 模型：

- 模式 1：正常运行
- 模式 2：轻微燃料元件损伤
- 模式 3：蒸汽发生器泄漏

马尔可夫链由连续时间生成矩阵 `Q` 描述，离散仿真时采用：

```math
P = e^{Q \Delta t}
```

从而在每个仿真步长内产生随机模式跃迁。

### 3. 预设时间稳定性思想

控制器采用时间增益调度形式，使误差压缩速度显式受 `T_p` 调节：

```math
u = -k_1 \Gamma(t)\operatorname{sig}(e)^{1/2} - k_2 \Gamma(t)e + u_{temp}
```

```math
\Gamma(t) = \left(\frac{T_p}{T_p - t}\right)^2, \quad t < T_p
```

其中 `e = n_r - n_d` 为功率跟踪误差，`T_p` 是可配置的预设收敛时间参数。达到 `T_p` 后，控制器切换至后续保持增益以抑制残余误差和随机工况扰动。

## 安装与使用

### 1. 环境依赖

项目依赖：

- `numpy`
- `scipy`
- `matplotlib`

安装命令：

```bash
pip install -r requirements.txt
```

### 2. 运行仿真

在仓库根目录执行：

```bash
python main.py
```

程序将完成以下工作：

- 构建三模态 PC-1 马尔可夫跳变场景；
- 进行 100% FP 到 50% FP 的线性降功率仿真；
- 计算 FSD、有限时间观测器和 P-T 控制器闭环响应；
- 在 `outputs/` 目录保存图像；
- 同时弹出图窗（若本地环境支持 GUI）。

## 关键特性

### 预设时间收敛

项目中的控制器将 `T_p` 作为显式参数暴露，便于直接研究不同预设收敛时间对负荷跟踪品质和控制作用强度的影响。

### 面向 PC-1 工况切换的鲁棒性

模型不仅考虑正常运行，还引入轻微燃料损伤和蒸汽发生器泄漏两类随机模式，通过马尔可夫链切换评估控制系统在随机突变工况下的保持能力。

### 含噪测量下的状态重构

冷却剂出口温度采用加噪测量，FSD 负责恢复平滑的 `T_l` 与 `dT_l/dt`，有限时间观测器进一步重构难以直接在线测量的 `c_r` 与 `T_f`。

### 面向报告和扩展的代码组织

项目使用面向对象结构拆分为堆芯模型、模式链、观测器、控制器、仿真和绘图模块，便于后续替换参数、扩展控制律或引入强化学习模块。

## 结果可视化

程序默认生成两张图：

### 1. `load_following_summary.png`

该图展示：

- 目标功率轨迹与实际功率轨迹；
- 功率跟踪误差与控制反应性输入；
- PC-1 模式的马尔可夫随机切换过程。

适合用于分析负荷跟踪精度、控制动作强度和随机工况变化时的闭环稳定性。

### 2. `observer_differentiator_summary.png`

该图展示：

- `T_l` 的真实值、含噪测量值与 FSD 估计值；
- `T_f` 的真实值与观测值；
- `c_r` 的真实值与观测值。

适合用于分析微分器抗噪性能与有限时间观测器的状态重构效果。

## 代码风格

本项目遵循以下实现原则：

- 面向对象设计；
- 模块化拆分，便于阅读与复用；
- 关键计算过程带有必要注释；
- 提供统一主入口 `main.py`；
- 运行后自动保存图像，可用于课程设计、项目报告和方法展示。

## 项目结构

```text
.
├── main.py
├── README.md
├── requirements.txt
├── outputs/
└── pwr_pt_rl_control/
    ├── __init__.py
    ├── config.py
    ├── controller.py
    ├── markov.py
    ├── model.py
    ├── observer.py
    ├── plotting.py
    └── simulation.py
```

## 说明

本仓库的实现重点是复现论文所强调的**结构与控制思路**：四状态 PWR 堆芯模型、PC-1 马尔可夫跳变、固定时间微分器、有限时间观测器、以及含预设收敛时间的控制器。若需要进一步与论文数值结果逐点对齐，可在现有代码框架上继续替换为论文原始参数表、完整神经网络补偿项和更细化的热工边界条件。
