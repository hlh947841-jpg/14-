function T_m_out = torque_distribution(t, T_driver, vx, dvx_err, psi_dot, ...
    omega_w, delta_eff, Fz_mf, cos_g, sin_g_lon, p)
%% torque_distribution.m — 四轮独立驱动扭矩分配控制器
% 根据 p.ctrl.mode 选择分配策略，全部使用光滑sigmoid过渡以保证ODE求解器收敛
%
% 控制模式:
%   mode=0: 禁用 — 四轮均摊（此函数不应在mode=0时被调用，由调用方gate处理）
%   mode=1: 载荷比例分配 — T_i ∝ Fz_mf(i)
%   mode=2: 载荷比例 + ASR/TCS — 滑移率超限时削减
%   mode=3: 载荷比例 + ASR/TCS + TVC/DYC — 横摆角速度闭环PI + 左右差动
%
% Input:
%   t          — 当前时间 [s]
%   T_driver   — 原始驾驶员扭矩命令 (4×1) [N·m]
%   vx         — 纵向车速 [m/s]
%   dvx_err    — 速度跟踪误差 p.vx0 - vx [m/s]
%   psi_dot    — 实际横摆角速度 [rad/s]
%   omega_w    — 四轮转速 (4×1) [rad/s]
%   delta_eff  — 有效转向角 (4×1) [rad]（已含ramp平滑）
%   Fz_mf      — 准静态垂向载荷 (4×1) [N]
%   cos_g      — cos(合成坡度角)
%   sin_g_lon  — sin(纵向坡度角)
%   p          — 车辆参数结构体（含 p.ctrl）
%
% Output:
%   T_m_out — 分配后的四轮电机扭矩命令 (4×1) [N·m]
%
% 状态持久化: 使用persistent变量存储横摆积分和滤波状态，
% 自动检测 t < t_last 时重置（支持连续多工况仿真）。

%% ===== 持久化状态（跨ODE步保持）=====
persistent yaw_int t_last T_tvc_prev
if isempty(t_last) || t <= eps
    % 新仿真开始或首次调用时重置所有状态
    yaw_int = 0;
    t_last = t;
    T_tvc_prev = zeros(4, 1);
end

% 防重复积分: ode15s 可能在同一步多次调用或短暂回退，
% 仅当 t 前进时才累积积分，避免回退时误清控制器状态。
dt = t - t_last;
if dt > 0
    t_last = t;
else
    dt = 0;  % 时间未前进，跳过积分累积
end

ctrl = p.ctrl;  % 控制参数（简写）

%% ===== Stage 0: 计算总扭矩需求 =====
% 前馈阻力估算（与原始速度控制器一致）
F_resist = p.M * p.g * (p.f_roll * cos_g + sin_g_lon) ...
         + 0.5 * p.rho_air * p.C_d * p.A_f * vx * abs(vx);
T_resist_ff = F_resist * p.r_w;  % 总阻力矩 [N·m]

% 速度P+前馈总修正力矩
T_m_speed = ctrl.Kp_speed * dvx_err + T_resist_ff;

% 总需求扭矩 = 驾驶员基础扭矩之和 + 速度修正
T_total = sum(T_driver) + T_m_speed;

%% ===== Stage A: 载荷比例分配（mode 1-3）=====
Fz_sum = sum(Fz_mf);
if Fz_sum < 1
    Fz_sum = 1;  % 防零除
end

% 按垂向载荷比例分配总扭矩
T_base = T_total * (Fz_mf ./ Fz_sum);

%% ===== Stage B: ASR/TCS 驱动防滑控制（mode 2-3）=====
T_asr = T_base;  % 默认与载荷分配一致

if ctrl.mode >= 2
    % 各轮轮心纵向速度（车辆坐标系，含横摆角速度效应）
    % 弯道中内/外侧轮心速度差可达 ±(B/2)·ψ_dot
    vx_hub_asr = zeros(4, 1);
    vx_hub_asr(1) = vx - p.B_f/2 * psi_dot;   % FL
    vx_hub_asr(2) = vx + p.B_f/2 * psi_dot;   % FR
    vx_hub_asr(3) = vx - p.B_r/2 * psi_dot;   % RL
    vx_hub_asr(4) = vx + p.B_r/2 * psi_dot;   % RR
    % 转换至车轮坐标系（忽略vy_hub，小侧偏角下近似成立）
    vx_w_asr = vx_hub_asr .* cos(delta_eff);
    % 纵向滑移率估算: κ ≈ (ω_w * r_w - vx_w) / max(|ω_w * r_w|, eps_v)
    denom = max(abs(omega_w * p.r_w), p.eps_v);  % 防零除
    kappa_est = (omega_w * p.r_w - vx_w_asr) ./ denom;

    for i = 1:4
        kappa_abs = abs(kappa_est(i));
        kappa_err = kappa_abs - ctrl.kappa_thr;

        if kappa_err > 0
            % Sigmoid光滑削减: reduction ∈ [0, 1]
            % κ 超过阈值越多，削减越接近 100%
            reduction = 1 / (1 + exp(-ctrl.K_asr_blend * kappa_err));
            % 从1（无削减）过渡到0（完全削减）
            T_asr(i) = T_base(i) * (1 - reduction);
        end
        % κ 未超阈值: 不变
    end
end

%% ===== Stage C: TVC/DYC 横摆力矩控制（mode 3）=====
T_tvc = T_asr;  % 默认与ASR一致

if ctrl.mode >= 3
    % C1: 计算目标横摆角速度（线性自行车模型）
    delta_f = mean(delta_eff(1:2));  % 前轴等效转向角
    psi_dot_ref = yaw_rate_reference(vx, delta_f, p, Fz_mf);  % 坡度感知：传入动态轴荷

    % C2: 横摆角速度PI控制器
    psi_dot_err = psi_dot_ref - psi_dot;

    % P项
    M_z_p = ctrl.Kp_yaw * psi_dot_err;

    % I项（含抗饱和）
    if dt > 0
        yaw_int = yaw_int + psi_dot_err * dt;
        % 积分抗饱和限幅
        yaw_int = min(max(yaw_int, -ctrl.yaw_int_max), ctrl.yaw_int_max);
    end
    M_z_i = ctrl.Ki_yaw * yaw_int;

    % 总横摆力矩需求 [N·m]
    M_z_demand = M_z_p + M_z_i;

    % C3: 横摆力矩 → 左右轮扭矩差
    % M_z 由左右轮纵向力差产生:
    %   M_z ≈ (B_f/2) * (Fx_FR - Fx_FL)/r_w + (B_r/2) * (Fx_RR - Fx_RL)/r_w
    % 近似: 前后轴按 ctrl.yaw_split_front 比例分担横摆力矩
    M_z_front = M_z_demand * ctrl.yaw_split_front;
    M_z_rear  = M_z_demand * (1 - ctrl.yaw_split_front);

    % 各轴所需左右扭矩差
    delta_T_front = M_z_front * p.r_w / p.B_f;
    delta_T_rear  = M_z_rear  * p.r_w / p.B_r;

    % 应用扭矩差（左减右加，产生正横摆力矩）
    % 车轮编号: 1=FL(左), 2=FR(右), 3=RL(左), 4=RR(右)
    T_tvc(1) = T_asr(1) - delta_T_front;   % FL 左前: 减少
    T_tvc(2) = T_asr(2) + delta_T_front;   % FR 右前: 增加
    T_tvc(3) = T_asr(3) - delta_T_rear;    % RL 左后: 减少
    T_tvc(4) = T_asr(4) + delta_T_rear;    % RR 右后: 增加

    % C4: 一阶低通滤波（防止扭矩突变导致ODE求解器失败）
    if dt > 0 && ctrl.tau_tvc > 0
        alpha_filt = dt / (ctrl.tau_tvc + dt);  % 一阶滤波系数
        T_tvc = T_tvc_prev + alpha_filt * (T_tvc - T_tvc_prev);
        T_tvc_prev = T_tvc;
    end
end

%% ===== Stage D: Sigmoid 光滑过渡（regen/accel 模式切换）=====
% 与原始 vehicle_dynamics.m 第89-95行逻辑一致
K_blend = ctrl.K_blend;

% 加速/减速模式光滑切换
s_blend = 1 / (1 + exp(-K_blend * dvx_err));
% 加速时 dvx_err>0 → s_blend≈1 → T_m_min≈0（不允许回馈制动）
% 减速时 dvx_err<0 → s_blend≈0 → T_m_min≈-T_m_regen_max（允许回馈制动）
T_m_min = (1 - s_blend) * (-p.T_m_regen_max);
T_m_out = max(T_tvc, T_m_min);

% 电机扭矩上限保护
T_m_out = min(T_m_out, p.T_m_max);

% 电机扭矩下限保护（回馈制动上限）
T_m_out = max(T_m_out, -p.T_m_regen_max);

end
