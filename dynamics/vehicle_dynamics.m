function [dx, aux] = vehicle_dynamics(t, x, p, u)
%% vehicle_dynamics.m — 14-DOF 四轮独立驱动/转向 动力学主ODE函数
% 组装全部29个状态导数
%
% 状态向量 x (29):
%   x(1)=vx, x(2)=vy, x(3)=z, x(4)=vz
%   x(5)=phi, x(6)=phi_dot, x(7)=theta, x(8)=theta_dot, x(9)=psi_dot
%   x(10:13)=z_w(1:4), x(14:17)=z_w_dot(1:4), x(18:21)=omega_w(1:4)
%   x(22:25)=Fx_relax(1:4), x(26:29)=Fy_relax(1:4)
%
% 控制输入 u (11):
%   u(1:4)  = delta(1:4)   — 四轮独立转向角 [rad]
%   u(5:8)  = T_m(1:4)     — 四轮独立电机扭矩 [N·m]
%   u(9)    = beta_brk      — 制动踏板开度 [0~1]
%   u(10)   = i_h           — 横向坡度比 [-]
%   u(11)   = i_c           — 纵向坡度比 [-]
%
% 车轮编号: 1=FL(左前), 2=FR(右前), 3=RL(左后), 4=RR(右后)

%% ===== 支持时变输入: u 可以是函数句柄 u(t) =====
	if isa(u, 'function_handle')
		u = u(t);
	end

%% ===== 强制列向量（防御性编程，兼容行向量调用）=====
	x = x(:);
	u = u(:);

%% ===== 提取状态 =====
vx = x(1);  vy = x(2);  z = x(3);  vz = x(4);
phi = x(5);  phi_dot = x(6);
theta = x(7);  theta_dot = x(8);
psi_dot = x(9);
z_w     = x(10:13);
z_w_dot = x(14:17);
omega_w = x(18:21);
Fx_relax = x(22:25);
Fy_relax = x(26:29);

%% ===== 提取控制输入 =====
delta    = u(1:4);    % 四轮转向角
T_m      = u(5:8);    % 四轮电机扭矩
beta_brk = u(9);      % 制动踏板
i_h      = u(10);     % 横向坡度比
i_c      = u(11);     % 纵向坡度比

%% ===== 输入验证与限幅 =====
% 转向角限幅 ±45°（防止非物理大转角导致运动学奇异）
delta = min(max(delta, -pi/4), pi/4);
% 电机扭矩限幅（含回馈制动负扭矩）
T_m = min(max(T_m, -p.T_m_regen_max), p.T_m_max);
% 制动踏板限幅 [0, 1]
beta_brk = min(max(beta_brk, 0), 1);
% 坡度比限幅
i_h = min(max(i_h, -p.slope_max), p.slope_max);
i_c = min(max(i_c, -p.slope_max), p.slope_max);


%% ===== 输入平滑过渡 =====
% C⁴连续九阶多项式，过渡时间 p.T_ramp（可配置）
tau = min(t / p.T_ramp, 1);
ramp = tau^5 * (126 - 420*tau + 540*tau^2 - 315*tau^3 + 70*tau^4);

% 四轮转向角平滑
delta_eff = delta * ramp;

% 坡度平滑
i_h_eff = i_h * ramp;
i_c_eff = i_c * ramp;

%% ===== 坡度角及准静态垂向载荷 =====
[cos_g, sin_g_lat, sin_g_lon, Fz_mf] = slope_load(i_h_eff, i_c_eff, p);

%% ===== 1. 非线性悬架力（模型D）=====
[Fs, z_b, vz_b] = suspension(z, vz, theta, theta_dot, phi, phi_dot, z_w, z_w_dot, cos_g, p);

%% ===== 车速P+前馈 + 扭矩分配控制器 =====
% 根据 p.ctrl.mode 选择分配策略:
%   mode=0: 原始四轮均摊（向后兼容，逐字保留原逻辑）
%   mode=1: 载荷比例分配 (T_i ∝ Fz_mf(i))
%   mode=2: 载荷比例 + ASR/TCS驱动防滑
%   mode=3: 载荷比例 + ASR/TCS + TVC/DYC横摆力矩控制
% 全部使用光滑sigmoid过渡，避免if/else导致导数不连续使ODE求解器失败

dvx_err = p.vx0 - vx;
if isfield(p, 'ctrl') && isfield(p.ctrl, 'K_blend')
    K_blend = p.ctrl.K_blend;
else
    K_blend = 5.0;
end

if isfield(p, 'ctrl') && p.ctrl.mode > 0
    %% === 新控制策略：调用统一扭矩分配模块 ===
    T_m = torque_distribution(t, u(5:8), vx, dvx_err, psi_dot, ...
        omega_w, delta_eff, Fz_mf, cos_g, sin_g_lon, p);
else
    %% === 原始四轮均摊（逐字保留，确保向后兼容）===
    % 前馈阻力估算: 滚动阻力 + 空气阻力 + 坡度阻力
    % F_resist = M*g*f_roll*cos(α) + (1/2)*ρ*Cd*A*v² + M*g*sin(α_lon)
    F_resist = p.M * p.g * (p.f_roll * cos_g + sin_g_lon) ...
             + 0.5 * p.rho_air * p.C_d * p.A_f * vx * abs(vx);
    T_resist_ff = F_resist * p.r_w;  % 总阻力矩 [N·m]（不预先除4，统一在下方均摊）

    % P项 + 前馈项: 总修正力矩（统一在下方四轮均摊，避免重复除以4）
    T_m_speed = p.Kp_speed * dvx_err + T_resist_ff;
    T_m = T_m + T_m_speed / 4;  % 四轮均摊（P项与FF项各仅除一次）

    % 光滑过渡: 加速工况T_m下限→0, 减速工况T_m下限→-T_m_regen_max
    % sigmoid: s=1/(1+exp(-k*dvx_err)), s≈1加速, s≈0减速
    s_blend = 1 / (1 + exp(-K_blend * dvx_err));
    T_m_min = (1 - s_blend) * (-p.T_m_regen_max);  % 加速→0, 减速→-T_m_regen_max
    T_m = max(T_m, T_m_min);
    T_m = min(T_m, p.T_m_max);  % 上限保护: 不超过电机峰值扭矩
end

% 光滑制动补充: 仅当实际车速高于目标并超过死区时自动补充摩擦制动。
% dvx_err = v_target - v_actual，负值表示超速；死区/上限在参数文件中标定。
if isfield(p, 'brake_deadband')
    brake_downhill_deadband = p.brake_deadband;
else
    brake_downhill_deadband = 1.0;
end
if isfield(p, 'brake_cruise_deadband')
    brake_cruise_deadband = p.brake_cruise_deadband;
else
    brake_cruise_deadband = 1.0;
end
if isfield(p, 'beta_brk_auto_max')
    beta_brk_auto_max = p.beta_brk_auto_max;
else
    beta_brk_auto_max = 0.3;
end
% 下坡时使用更小死区，平路/上坡保留巡航死区，避免正常小超调就摩擦制动。
s_downhill_brake = 1 / (1 + exp(40 * (sin_g_lon + 0.01)));
brake_deadband = s_downhill_brake * brake_downhill_deadband ...
    + (1 - s_downhill_brake) * brake_cruise_deadband;
speed_over = max(-dvx_err - brake_deadband, 0);
s_brk = 1 / (1 + exp(K_blend * (dvx_err + brake_deadband)));
beta_brk_auto = s_brk * min(speed_over * p.Kp_brake, beta_brk_auto_max);
beta_brk = max(beta_brk, beta_brk_auto);

%% ===== 2. 轮胎垂向力 =====
% 轮胎-悬架串联弹簧系统: Fz_tire 由轮胎压入地面深度决定（独立于悬架力）
% 车轮垂向动力学 z_w_ddot = (Fz_tire - Fs - m_u*g*cos_g) / m_u 求解力平衡
% 注意: Fz_mf(准静态)用于Magic Formula摩擦椭圆缩放，
%       Fz_tire(动态)仅用于车轮垂向运动方程，两者各司其职，非过约束
Fz_tire = tire_vertical(z_w, z_w_dot, p);
%% ===== 动态载荷转移混合（可选）=====
% 将悬架动态力偏差叠加到准静态Fz_mf，提高Magic Formula垂向力估算精度
% 悬架力相对静态预载的偏差反映了动态载荷转移
if isfield(p, 'use_dyn_load_transfer') && p.use_dyn_load_transfer
    %% 静态预载（与suspension.m中F_s0一致）
    F_s0_dyn = zeros(4,1);
    F_s0_dyn(1) = p.m_s * p.g * p.b * cos_g / (2 * p.L);
    F_s0_dyn(2) = p.m_s * p.g * p.b * cos_g / (2 * p.L);
    F_s0_dyn(3) = p.m_s * p.g * p.a * cos_g / (2 * p.L);
    F_s0_dyn(4) = p.m_s * p.g * p.a * cos_g / (2 * p.L);
    %% 动态载荷转移 = 悬架力 - 静态预载（一阶低通滤波抑制高频振荡）
    dFz_dyn = Fs - F_s0_dyn;
    %% 混合: 80%%准静态 + 20%%动态（可调混合比）
    alpha_blend = 0.2;
    Fz_mf = Fz_mf + alpha_blend * dFz_dyn;
    Fz_mf = max(Fz_mf, 100);  %% 防止负值
end

%% ===== 3. 轮胎运动学（四轮独立转向）=====
[alpha, kappa, vx_w] = tire_kinematics(vx, vy, psi_dot, omega_w, delta_eff, p);

%% ===== 4. 轮胎力（Magic Formula + 摩擦椭圆）=====
[Fx0, Fy0] = tire_forces(alpha, kappa, Fz_mf, p);

% 轮胎松弛长度动力学（一阶滞后）
% dF/dt = (F_ss - F) * |vx_w| / σ,  时间常数 τ = σ / |vx_w|
% 各轮使用各自的轮心纵向速度 vx_w(i)（已在 tire_kinematics 中计算）
% 低速时增加最小松弛频率，防止响应过慢导致不稳定
% 纵滑与侧偏松弛长度独立（侧偏松弛长度通常为纵滑的2~3倍）
inv_tau_x = zeros(4, 1);
inv_tau_y = zeros(4, 1);
for i_relax = 1:4
    vx_w_abs = max(abs(vx_w(i_relax)), p.eps_v);
    inv_tau_x(i_relax) = max(vx_w_abs / p.sigma_x, p.inv_tau_min);
    inv_tau_y(i_relax) = max(vx_w_abs / p.sigma_y, p.inv_tau_min);
end
Fx = Fx_relax;
Fy = Fy_relax;

%% ===== 5. 滚动阻力矩 =====
T_roll = Fz_mf * p.f_roll * p.r_w;

%% ===== 6. 制动力矩 =====
T_brake = brake(beta_brk, p);

%% ===== 7. 驱动力矩（四轮独立电机）=====
T_drive = motor_drive(T_m, omega_w, p);

% 单轮扭矩附着极限（正驱动/回馈制动对称限制）
T_max_per_wheel = p.mu_x * Fz_mf * p.r_w;
for i_tq = 1:4
    T_limit_i = max(T_max_per_wheel(i_tq), 1e-6);
    T_sign_i = sign(T_drive(i_tq));
    T_abs_i = abs(T_drive(i_tq));
    ratio = T_abs_i / T_limit_i;
    ex = min(max(-(ratio - 1) * 60, -40), 40);
    w_tq = 1 / (1 + exp(ex));
    T_drive(i_tq) = T_drive(i_tq) * (1 - w_tq) + T_sign_i * T_limit_i * w_tq;
end

%% ===== 8. 空气阻力 =====
F_aero = aero(vx, p);

%% ===================== 车身运动方程 =====================

% --- 车身坐标系下各轮轮胎力 ---
% Fx_body_i = Fx_i * cos(δ_i) - Fy_i * sin(δ_i)
% Fy_body_i = Fx_i * sin(δ_i) + Fy_i * cos(δ_i)
Fx_body = Fx .* cos(delta_eff) - Fy .* sin(delta_eff);
Fy_body = Fx .* sin(delta_eff) + Fy .* cos(delta_eff);

% --- 纵向运动 ---
% 车身坐标系加速度: ax_body = ΣFx / M (不含科氏力)
% 惯性系导数: dvx/dt = ax_body + vy*ψ_dot
Fx_total = sum(Fx_body) - F_aero;
ax_body = (Fx_total - p.M * p.g * sin_g_lon) / p.M;
ax = ax_body + vy * psi_dot;  % dvx/dt

% --- 横向运动 ---
% 车身坐标系加速度: ay_body = ΣFy / M (不含科氏力)
% 惯性系导数: dvy/dt = ay_body - vx*ψ_dot
Fy_total = sum(Fy_body);
ay_body = (Fy_total - p.M * p.g * sin_g_lat) / p.M;
ay = ay_body - vx * psi_dot;  % dvy/dt

% --- 横摆运动 ---
% Mz = Σ(r_i × F_body_i)_z
% 各轮位置: r_FL=[a, +B_f/2], r_FR=[a, -B_f/2], r_RL=[-b, +B_r/2], r_RR=[-b, -B_r/2]
% 叉积 z 分量: r_x * Fy_body - r_y * Fx_body
Mz = p.a * (Fy_body(1) + Fy_body(2)) ...
   - p.B_f/2 * (Fx_body(1) - Fx_body(2)) ...
   - p.b * (Fy_body(3) + Fy_body(4)) ...
   - p.B_r/2 * (Fx_body(3) - Fx_body(4));
psi_ddot = Mz / p.I_z;

%% --- 侧倾运动 ---
%% 侧倾力矩 = 簧上质量横向惯性力力矩 + 悬架力差力矩 + 重力分量
%% (悬架stroke=z_w-z_b, 正=压缩. 右倾→右侧Fs↑→Mx_susp<0→恢复)
Mx_susp = (Fs(1) - Fs(2)) * p.B_sf / 2 + (Fs(3) - Fs(4)) * p.B_sr / 2;
phi_ddot = (p.m_s * p.d_roll * ay_body ...
           + Mx_susp ...
           + p.m_s * p.g * (p.d_roll * cos_g * sin(phi) ...
                          + p.d_roll * sin_g_lat * cos(phi))) / p.I_x;

%% --- 俯仰运动 ---
%% 俯仰力矩 = 簧上质量纵向惯性力力矩 + 前后悬架力差力矩 + 重力分量
%% (悬架stroke=z_w-z_b, 正=压缩. 点头→前Fs↑→My_susp↑→恢复)
My_susp = (Fs(1) + Fs(2)) * p.a - (Fs(3) + Fs(4)) * p.b;
theta_ddot = (+p.m_s * p.d_pitch * ax_body ...
             + My_susp ...
             + p.m_s * p.g * (p.d_pitch * cos_g * sin(theta) ...
                            + p.d_pitch * sin_g_lon * cos(theta))) / p.I_y;

% --- 垂向运动 ---
% m_s * az = ΣFs(向上) - m_s*g*cos(α_grad)(向下)
az = (sum(Fs) - p.m_s * p.g * cos_g) / p.m_s;

%% ===================== 车轮垂向动力学 =====================
% 四个车轮独立垂向运动（串联弹簧-质量系统）
% m_u_i * z_w_ddot_i = Fz_tire_i - Fs_i - m_u_i * g * cos(α_grad)
% Fz_tire 来自轮胎压地深度, Fs 来自悬架相对位移, 二者独立计算
% ODE求解器同步求解，保证力-运动自洽，非过约束系统
m_u = [p.m_uf; p.m_uf; p.m_ur; p.m_ur];  % 四轮非簧载质量
z_w_ddot = zeros(4, 1);
for i = 1:4
    z_w_ddot(i) = (Fz_tire(i) - Fs(i) - m_u(i) * p.g * cos_g) / m_u(i);
end

%% ===================== 车轮旋转动力学 =====================
% 四轮独立旋转
% I_w * omega_w_dot_i = T_drive_i - Fx_i * r_w - T_roll_i - T_brake_i
omega_w_dot = zeros(4, 1);
for i = 1:4
    omega_w_dot(i) = (T_drive(i) - Fx(i) * p.r_w - T_roll(i) - T_brake(i)) / p.I_w;
end

%% ===================== 轮胎松弛动力学 =====================
% dFx/dt = (Fx0 - Fx) * inv_tau_i,  inv_tau_i = max(|vx_w_i|/σ, inv_tau_min)
% dFy/dt = (Fy0 - Fy) * inv_tau_i
% inv_tau_min 保证低速时仍有合理响应速度，防止松弛力"冻结"
dFx_relax = (Fx0 - Fx_relax) .* inv_tau_x;
dFy_relax = (Fy0 - Fy_relax) .* inv_tau_y;

%% ===================== 组装状态导数 =====================
dx = zeros(29, 1);
dx(1)  = ax;                     % dvx/dt
dx(2)  = ay;                     % dvy/dt
dx(3)  = vz;                     % dz/dt
dx(4)  = az;                     % dvz/dt
dx(5)  = phi_dot;                % dφ/dt
dx(6)  = phi_ddot;               % d²φ/dt²
dx(7)  = theta_dot;              % dθ/dt
dx(8)  = theta_ddot;             % d²θ/dt²
dx(9)  = psi_ddot;               % dωz/dt
dx(10:13) = z_w_dot;             % dz_w/dt
dx(14:17) = z_w_ddot;            % d²z_w/dt²
dx(18:21) = omega_w_dot;         % dω_w/dt
dx(22:25) = dFx_relax;           % 纵向松弛力导数
dx(26:29) = dFy_relax;           % 侧向松弛力导数

if nargout > 1
    aux.u_actual = u;
    aux.delta_eff = delta_eff;
    aux.i_h_eff = i_h_eff;
    aux.i_c_eff = i_c_eff;
    aux.cos_g = cos_g;
    aux.sin_g_lat = sin_g_lat;
    aux.sin_g_lon = sin_g_lon;
    aux.Fz_mf = Fz_mf;
    aux.Fz_tire = Fz_tire;
    aux.Fs = Fs;
    aux.z_b = z_b;
    aux.vz_b = vz_b;
    aux.alpha = alpha;
    aux.kappa = kappa;
    aux.vx_w = vx_w;
    aux.Fx0 = Fx0;
    aux.Fy0 = Fy0;
    aux.Fx = Fx;
    aux.Fy = Fy;
    aux.Fx_body = Fx_body;
    aux.Fy_body = Fy_body;
    aux.T_roll = T_roll;
    aux.T_brake = T_brake;
    aux.T_drive = T_drive;
    aux.T_m = T_m;
    aux.beta_brk = beta_brk;
    aux.F_aero = F_aero;
    aux.ax_body = ax_body;
    aux.ay_body = ay_body;
    aux.Mz = Mz;
    aux.phi_ddot = phi_ddot;
    aux.theta_ddot = theta_ddot;
    aux.z_w_ddot = z_w_ddot;
    aux.omega_w_dot = omega_w_dot;
    aux.dFx_relax = dFx_relax;
    aux.dFy_relax = dFy_relax;
    aux.beta_brk_auto = beta_brk_auto;
end
end
