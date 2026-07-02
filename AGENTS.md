# AGENTS.md — 14-DOF 四轮独立驱动/转向底盘动力学模型

本文件为 Codex 提供项目上下文和维护规则。

## 项目概述

14 自由度底盘动力学模型，基于邓涛等 (2012) 论文扩展。核心特征：
- **14 DOF**：车身 6（纵向/横向/垂向/侧倾/俯仰/横摆）+ 车轮垂向 4 + 车轮旋转 4
- **29 状态变量**：含 8 个轮胎松弛力状态（一阶滞后动力学）
- **11 控制输入**：四轮转向角 δ(1:4) + 四轮电机扭矩 T_m(1:4) + 制动踏板 β_brk + 坡度比 i_h, i_c
- **四轮独立驱动 (4WID)** + **四轮独立转向 (4WIS)**

## 环境

- **平台**：MATLAB（Windows）
- **ODE 求解器**：`ode15s`（刚性系统，悬架+轮胎跨多时间尺度）
- **Simulink**：可选，`build_model.m` 程序化构建模型
- **Python 辅助**：豆包视觉模型分析脚本（`_analyze_all.py`），依赖 curl + Python 3

## 文件结构

```
├── config/
│   └── vehicle_params.m         # 参数定义（全字母符号，数字仅在赋值处）
├── control/
│   ├── control_params.m         # 扭矩分配控制参数（独立配置）
│   ├── torque_distribution.m    # 扭矩分配主控制器（载荷/ASR/TVC）
│   └── yaw_rate_reference.m     # 线性自行车模型 → 目标横摆角速度
├── dynamics/
│   ├── vehicle_dynamics.m       # 主ODE函数 (29状态导数)
│   ├── suspension.m             # 非线性悬架模型D
│   ├── tire_kinematics.m        # 四轮独立转向运动学
│   ├── tire_forces.m            # Magic Formula + 摩擦椭圆
│   ├── tire_vertical.m          # 轮胎垂向力
│   ├── slope_load.m             # 坡度载荷与垂向力分配
│   ├── aero.m                   # 空气阻力
│   ├── brake.m                  # 液压制动系统
│   ├── motor_drive.m            # 四轮独立电机驱动
│   ├── core_dynamics_wrapper.m  # Simulink封装
│   └── core_dynamics_wrapper_sfun.m  # S-Function封装
├── simulation/
│   ├── simulate.m               # ODE仿真封装（含静平衡初值计算）
│   ├── plot_results.m           # 单工况可视化（7类曲线）
│   └── plot_multi_case.m        # 多工况对比（3图）
├── simulink/
│   ├── build_model.m            # 程序化构建Simulink模型
│   ├── create_sfun_subsystem.m  # S-Function子系统
│   └── run_sim_simulink.m       # Simulink仿真入口
├── run_sim.m                    # 一键运行入口
├── simulation_results.mat       # 仿真结果缓存
├── results_figures/             # 45张可视化图片输出
└── README.md                    # 项目说明
```

## 常用操作

### 运行仿真

```matlab
>> run_sim
```

自动执行：参数加载 → 6 工况 ODE 仿真 → 可视化 → 保存图片和 .mat 结果。

仿真耗时约 0.3~0.5s/工况，共约 2~3 秒。

### 添加新工况

在 `run_sim.m` 中按以下模式添加：
1. 定义控制向量 `u = [delta(1:4), T_m(1:4), beta_brk, i_h, i_c]`
2. 调用 `R = simulate(p, u, [], T_sim)`
3. 调用 `plot_results(R, '工况名')`
4. 将 R 传入 `plot_multi_case` 的第 N+1 个参数（需同步修改该函数签名）

### 仅运行单个工况

```matlab
p = vehicle_params();
u = [deg2rad([2,2,1,1]), 80,80,50,50, 0, 0, 0];
R = simulate(p, u, [], 5);
plot_results(R, '自定义工况');
```

### 调整车辆参数

修改 `config/vehicle_params.m`。参数命名规则：全字母符号，物理意义明确。修改后 `run_sim` 自动重新加载。

### 使用扭矩分配控制模块

控制模块在 `control/` 目录下，通过 `p.ctrl.mode` 切换策略：

```matlab
p = vehicle_params();          % 自动加载 control_params（默认 mode=0）

% 切换控制模式
p.ctrl.mode = 0;  % 禁用 — 四轮均摊（向后兼容）
p.ctrl.mode = 1;  % 载荷比例分配 T_i ∝ Fz_mf(i)
p.ctrl.mode = 2;  % 载荷比例 + ASR/TCS 驱动防滑
p.ctrl.mode = 3;  % 载荷比例 + ASR/TCS + TVC/DYC 横摆力矩控制

R = simulate(p, u, [], T_sim);
```

**控制参数调优**（`control/control_params.m`）：
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ctrl.kappa_thr` | 0.15 | ASR滑移率阈值 |
| `ctrl.Kp_yaw` | 500 | 横摆PI-P增益 |
| `ctrl.Ki_yaw` | 100 | 横摆PI-I增益 |
| `ctrl.tau_tvc` | 0.05s | TVC力矩滤波时间常数 |

**状态持久化**：`torque_distribution.m` 使用 `persistent` 变量存储横摆积分器，自动检测 t 回退时重置（支持连续多工况仿真）。

## MATLAB 编码约定

### 参数命名
- 所有公式中只使用字母符号，不出现具体数字
- 数字仅在参数赋值处出现
- 命名规范：`p.field_name`，如 `p.K_s1_f`（前悬架线性刚度）、`p.d_roll`（重心至侧倾轴线距离）

### 关键物理参数
| 参数 | 值 | 说明 |
|------|-----|------|
| `p.d_roll` | 0.46 m | 重心至侧倾轴线垂直距离 |
| `p.d_pitch` | 0.46 m | 重心至俯仰轴线垂直距离 |
| `p.T_ramp` | 1.0 s | 控制输入光滑过渡时间 |
| `p.eps_gnd` | 1e-3 m | 地面接触光滑过渡宽度 |
| `p.inv_tau_min` | 3.0 1/s | 最小松弛频率（防低速冻结） |

### 光滑过渡规则
- **禁止硬 if/else 切换**：ODE 求解器需要连续导数
- 使用 sigmoid 过渡：`s = 1 / (1 + exp(-K * (x - x0)))`
- 输入平滑：C⁴ 连续九阶多项式 ramp（`p.T_ramp` 内完成过渡）
- 违反此规则会导致 `ode15s` 积分失败或错误振荡

### 防御性编程
- `vehicle_dynamics.m` 入口强制列向量：`x = x(:); u = u(:)`
- 所有控制输入有限幅检查
- 被零保护：`max(abs(vx_w), p.eps_v)` 等

## 6 种仿真工况

| 工况 | 描述 | 转向角 | 坡度 | 特殊说明 |
|------|------|--------|------|---------|
| 1 | 平路转向 | FL/FR=2°, RL/RR=1° | 无 | 常规转弯 |
| 2 | 纵坡直线 | 0 | i_c=0.1 | 纯直线，物理上应无侧倾/横摆 |
| 3 | 横坡直线 | 0 | i_h=0.1 | 观察横坡漂移效应 |
| 4 | 恒速巡航 | 0 | 无 | 100km/h匀速，验证稳态速度维持 |
| 5 | 蟹行模式 | 四轮同向 5° | 无 | 四轮转向特有 |
| 6 | 小半径转向 | FL/FR=±3°, RL/RR=∓3° | 无 | 前后反向偏转 |

**工况 4 特别注意**：`run_sim.m` 中临时修改 `p.vx0 = 100/3.6`，仿真后恢复。不可在 `vehicle_params.m` 中永久修改 `vx0`（会影响其他工况）。

## 绘图规则（重要）

### ylim 最小范围守卫

MATLAB 自动缩放会将机器精度噪声（10⁻¹⁵ 级别）放大显示为"振荡"。**所有涉及车身姿态和平动速度的 subplot 必须添加 ylim 守卫**。

`plot_results.m` 中已添加 5 处守卫：
```matlab
% 速度类 (vy, vz)：最小范围 ±0.01 m/s
y_range = max(abs(ylim));  if y_range < 0.01, ylim([-0.01, 0.01]); end

% 角度类 (phi, theta, psi_dot)：最小范围 ±0.1 deg 或 deg/s
y_range = max(abs(ylim));  if y_range < 0.1, ylim([-0.1, 0.1]); end
```

`plot_multi_case.m` 中已添加 3 处守卫（phi, theta, psi_dot）。

**新增 subplot 或修改现有图时，遵循同样规则。**

### 图片输出
- 格式：PNG，通过 `saveas` 保存
- 文件名：去除 `\/:*?"<>|` 等非法字符
- 输出目录：`results_figures/`
- 共生成 45 张图片（6 工况 × 7 图 + 3 多工况对比）

## 已修复的历史问题

以下问题已在之前的会话中修复，**不要回退这些修改**：

1. **`p.d_roll` / `p.d_pitch`**：原值分离了重力项和惯性项力臂。修正为统一的 `0.46 m`（= h_g - 侧倾轴线高度 ≈ 0.55 - 0.09）。

2. **`vehicle_dynamics.m` 松弛频率**：原为全局 `inv_tau`，修正为各轮独立 `inv_tau(i)`（各轮 `vx_w` 不同）。

3. **`suspension.m` 静态预载**：原未考虑坡度，修正为使用 `cos_g` 参数。

4. **`slope_load.m` 轮荷转移**：原分母误用单侧轮距，修正为 `2 * p.B_f` 和 `2 * p.B_r`。

5. **`tire_vertical.m` 接地判断**：原 `eps_clamp = 1`（硬阈值 1m），修正为 `p.eps_gnd * p.K_t`（力阈值，物理合理）。

6. **`motor_drive.m` 恒功率切换**：原为硬 if/else，修正为 sigmoid 光滑过渡。

7. **`run_sim.m`**：原用 `try/finally`（MATLAB 不支持 `finally`），修正为显式 `p.vx0_save`/恢复。工况 6 转向角从 ±5° 修正为 ±3°。

8. **`vehicle_dynamics.m` 俯仰惯性项符号**：原为 `-m_s·d_pitch·ax_body`（加速→车头下沉），修正为 `+m_s·d_pitch·ax_body`（加速→车头上扬/squat，制动→车头下沉/dive）。与侧倾惯量项符号体系统一。

9. **`suspension.m` 非对称阻尼**：原平方阻尼项 `C_s2·|v|·v` 产生对称 progressive 阻尼但注释声称不对称。修正为 sigmoid 光滑分离压缩/回弹系数，回弹阻尼为压缩阻尼 2.75 倍（`xi_reb_comp=2.75`，`K_d_susp=15`），保持每周期能量耗散不变。

10. **`vehicle_dynamics.m` 轮胎松弛长度分离**：原纵滑和侧偏共用 `σ=0.3m`。修正为 `σ_x=0.30m`（纵滑）和 `σ_y=0.60m`（侧偏，通常为纵滑2~3倍），各轮独立 `inv_tau_x(i)` 和 `inv_tau_y(i)`。

11. **`torque_distribution.m` ASR 滑移率估算**：原使用 CG 速度 `vx` 代替各轮轮心速度，弯道中内外轮速度差可达 ±(B/2)·ψ_dot。修正为各轮 `vx_hub(i) = vx ± (B_f或B_r)/2·ψ_dot`，含横摆角速度效应。

12. **`control_params.m` TVC 滤波时间常数**：`tau_tvc` 从 0.02s 提高至 0.05s，降低横摆力矩控制的 ODE 刚度。

13. **`run_sim.m` 工况4描述**：从"零起步直线全力加速"修正为"百公里恒速巡航"，与实际仿真条件一致（`vx0=100km/h`，速度控制器维持稳态）。

14. **`_analyze_all.py` API Key 安全**：移除硬编码密钥，改为从环境变量 `DOUBAO_API_KEY` 读取。

## 豆包视觉模型分析

### 调用方式

豆包 MCP 服务（`doubao-vision`）可能不在会话中可用。备用方案：使用 Python 脚本通过 curl 直接调用 API。

```bash
cd "E:\何瀚林研究生文件\何瀚林研究生文件\底盘动力学模型\14DOF四轮独立驱动模型"
python _analyze_all.py
```

API 配置（在 `_analyze_all.py` 中）：
- Base URL: `https://ark.cn-beijing.volces.com/api/v3`
- Endpoint: `ep-m-20260601115556-qjkdn`
- API Key: 已硬编码在脚本中（注意安全，不应提交到 git）

### 已知误报

豆包视觉模型会将 MATLAB 自动缩放放大的机器精度噪声误判为"振荡"。**在判断图像异常前，先用数值提取确认**：
```matlab
fprintf('phi: [%g, %g] deg, pk-pk=%g\n', min(phi)*180/pi, max(phi)*180/pi, peak2peak(phi)*180/pi);
fprintf('psi_dot: [%g, %g] deg/s\n', min(psi_dot)*180/pi, max(psi_dot)*180/pi);
```

如果 pk-pk < 1e-6，则为机器精度噪声，非物理振荡。

### 图片分析脚本规范
- 结果写入 UTF-8 JSON 文件，**不要打印到 stdout**（Windows GBK 编码会崩溃）
- 使用 ASCII-safe 的 print 语句输出进度
- 并发数 ≤ 4（避免 API 限流）

## Windows/MATLAB 兼容性注意事项

### 路径
- MATLAB `-batch` 模式不接受中文路径（报"文本字符无效"）
- 临时脚本放在 ASCII 路径下（如 `D:\debug_c4.m`）
- 使用正斜杠 `/` 作为路径分隔符：`run('D:/debug_c4.m')`

### genpath 陷阱
- `genpath(root_dir)` 会包含所有子目录，可能 shadow MATLAB 内置函数（`clc`, `home`, `pause`, `cos`, `sin` 等）
- **推荐**：只添加需要的子目录
```matlab
addpath(root_dir);
addpath(fullfile(root_dir, 'config'));
addpath(fullfile(root_dir, 'dynamics'));
addpath(fullfile(root_dir, 'simulation'));
addpath(fullfile(root_dir, 'visualization'));
```
- 尽量避免 `addpath(genpath(root_dir))`

### 编码
- `.m` 文件使用 UTF-8，中文注释直接写
- Python 脚本输出避免非 ASCII 字符（Windows GBK 终端会崩溃）
- 文件名避免特殊字符

## 模型验证清单

验证仿真结果是否正确，检查以下项目：

- [ ] 工况 2（纵坡直线）：侧倾角 φ ≈ 0，横摆角速度 ψ_dot ≈ 0（物理约束）
- [ ] 工况 4（恒速巡航）：φ ≈ 0，ψ_dot ≈ 0，vy ≈ 0（对称工况）
- [ ] 所有工况：vx 终点接近目标车速（速度控制器有效）
- [ ] 参数校验通过：`M = m_s + 2×m_uf + 2×m_ur`（vehicle_params 自动检查）
- [ ] 松弛力初始值合理，无初始瞬态跳变
- [ ] 图像中无异常"振荡"（先检查数值，再判断）
- [ ] 加速工况：theta > 0（车头上扬/squat），非antisquat
- [ ] 制动工况：theta < 0（车头下沉/dive）

## 参考文献

邓涛, 等. "考虑坡度影响的车辆行驶动力学建模与仿真." 2012.
