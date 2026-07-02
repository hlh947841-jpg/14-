function R = simulate(p, u, x0, T_sim, solver_opts)
%% simulate.m — 14-DOF 模型 ODE 仿真封装
% 使用 ode15s 求解 (刚性系统，悬架+轮胎动力学跨多个时间尺度)
%
% Input:
%   p          — 参数结构体（来自 vehicle_params）
%   u          — 控制输入向量 (11元素)，或函数句柄 u(t) 返回11元素向量:
%                  u(1:4)=delta(1:4) [rad], u(5:8)=T_m(1:4) [N·m],
%                  u(9)=beta_brk [0~1], u(10)=i_h, u(11)=i_c
%   x0         — 初始状态 (29元素)，若为空则自动计算静平衡
%   T_sim      — 仿真时长 [s]
%   solver_opts— ode15s 选项结构体 (可选)
%
% Output:
%   R — 结构体，包含:
%       R.t, R.x           — 时间向量与状态矩阵
%       R.vx, R.vy, R.vz   — 车身速度
%       R.phi, R.theta     — 侧倾/俯仰角
%       R.psi_dot          — 横摆角速度
%       R.z_w, R.omega_w   — 车轮垂向位移/转速
%       R.Fx, R.Fy, R.Fz   — 轮胎力
%       R.Fs              — 悬架力
%       R.alpha, R.kappa   — 侧偏角/滑移率

%% 默认输入处理
if nargin < 5 || isempty(solver_opts)
    solver_opts = odeset('RelTol', 1e-4, 'AbsTol', 1e-6, ...
                         'MaxStep', 0.005, 'Stats', 'off');
end
if nargin < 4 || isempty(T_sim)
    T_sim = 5;
end
if nargin < 3 || isempty(x0)
    % 自动计算静平衡初始状态
    if isa(u, 'function_handle')
        x0 = initial_state_14dof(p, u(0));
    else
        x0 = initial_state_14dof(p, u);
    end
end

%% ODE 求解
fprintf('开始 14-DOF ODE 仿真 (T=%.1f s)...\n', T_sim);
tic;

clear torque_distribution;
[t, x] = ode15s(@(t, x) vehicle_dynamics(t, x, p, u), [0, T_sim], x0, solver_opts);

elapsed = toc;
fprintf('仿真完成: %.1f 秒, %d 步, vx终点=%.2f km/h\n', ...
        elapsed, length(t), x(end, 1) * 3.6);

%% 提取状态时间历程
R.t = t;
R.x = x;

% 车身状态
R.vx = x(:, 1);
R.vy = x(:, 2);
R.z  = x(:, 3);
R.vz = x(:, 4);
R.phi     = x(:, 5);
R.phi_dot = x(:, 6);
R.theta     = x(:, 7);
R.theta_dot = x(:, 8);
R.psi_dot   = x(:, 9);

% 车轮状态
R.z_w     = x(:, 10:13);
R.z_w_dot = x(:, 14:17);
R.omega_w = x(:, 18:21);

% 轮胎松弛力
R.Fx_relax = x(:, 22:25);
R.Fy_relax = x(:, 26:29);

% 计算各时刻派生量
n = length(t);
R.Fx = zeros(n, 4);
R.Fy = zeros(n, 4);
R.Fz = zeros(n, 4);
R.Fs = zeros(n, 4);
R.alpha  = zeros(n, 4);
R.kappa  = zeros(n, 4);
R.delta_eff = zeros(n, 4);
R.T_drive   = zeros(n, 4);
R.T_brake   = zeros(n, 4);
R.u_actual  = zeros(n, 11);  % 存储各时刻实际输入（含时变坡度）

clear torque_distribution;
for i = 1:n
    xi = x(i, :)';
    [~, aux_i] = vehicle_dynamics(t(i), xi, p, u);

    R.delta_eff(i, :) = aux_i.delta_eff';
    R.u_actual(i, :) = aux_i.u_actual';
    R.Fs(i, :) = aux_i.Fs';
    R.Fz(i, :) = aux_i.Fz_tire';
    R.alpha(i, :) = aux_i.alpha';
    R.kappa(i, :) = aux_i.kappa';
    R.Fx(i, :) = aux_i.Fx';
    R.Fy(i, :) = aux_i.Fy';
    R.T_drive(i, :) = aux_i.T_drive';
    R.T_brake(i, :) = aux_i.T_brake';
end

% 存储工况信息
R.info.speed_kmh = p.vx0 * 3.6;
if isa(u, 'function_handle')
    R.info.input_type = 'time-varying';
    R.info.i_h = 'see R.u_actual(:,10)';
    R.info.i_c = 'see R.u_actual(:,11)';
else
    R.info.i_h = u(10);
    R.info.i_c = u(11);
    R.info.delta_deg = u(1:4) * 180 / pi;
    R.info.T_m = u(5:8);
end

% 侧翻稳定性指标 — 横向载荷转移率 (LTR)
% LTR = (Fz_left - Fz_right) / (Fz_left + Fz_right)
% LTR ∈ [-1, 1], |LTR| > 0.9 表示一侧车轮即将离地（侧翻风险）
Fz_left  = R.Fz(:,1) + R.Fz(:,3);   % FL + RL
Fz_right = R.Fz(:,2) + R.Fz(:,4);   % FR + RR
R.LTR = (Fz_left - Fz_right) ./ max(Fz_left + Fz_right, 1e-3);
R.info.LTR_max = max(abs(R.LTR));
R.info.rollover_risk = R.info.LTR_max > 0.9;
if R.info.rollover_risk
    fprintf('⚠ 侧翻风险: LTR_max = %.2f (阈值 0.9)\n', R.info.LTR_max);
end

fprintf('仿真结果已打包到结构体 R。\n');
end

%% ===== 静平衡初始状态 =====
function x0 = compute_steady_state(p, u)
% 计算车辆在给定工况下的静平衡状态
% 平路静止或匀速时，车身姿态与悬架预载平衡

x0 = zeros(29, 1);

% 速度
x0(1) = p.vx0;    % vx = 初始车速
x0(2) = 0;        % vy = 0
x0(3) = 0;        % z = 0 (以静平衡位置为零点)
x0(4) = 0;        % vz = 0

% 姿态角（平路初始为零）
x0(5) = 0;        % phi = 0
x0(6) = 0;        % phi_dot = 0
x0(7) = 0;        % theta = 0
x0(8) = 0;        % theta_dot = 0
x0(9) = 0;        % psi_dot = 0

	% 车轮垂向位移 —— 正确静平衡计算（轮胎-悬架串联弹簧）
	% 静平衡条件: Fz_tire = F_s + m_u*g
	%   Fz_tire = K_t * (0 - z_w)    （线性近似，z_w<0 时轮胎压入地面）
	%   F_s = F_s0 + K_s1*(z - z_w)  （忽略三次项，取初值）
	% 令 z pprox z_w（悬架行程近似为零），则 F_s pprox F_s0
	% 解得: z_w = -(F_s0 + m_u*g) / K_t
		% 静平衡需考虑坡度对法向重力分量的影响
		alpha_grad_init = atan(sqrt(u(10)^2 + u(11)^2));
		cos_g_init = cos(alpha_grad_init);
		F_s0_FL = p.m_s * p.g * p.b * cos_g_init / (2 * p.L);
		F_s0_RL = p.m_s * p.g * p.a * cos_g_init / (2 * p.L);
		z_w0_f = -(F_s0_FL + p.m_uf * p.g * cos_g_init) / p.K_t;
		z_w0_r = -(F_s0_RL + p.m_ur * p.g * cos_g_init) / p.K_t;
	x0(3)  = mean([z_w0_f; z_w0_f; z_w0_r; z_w0_r]);	% 车身 z 取轮心平均高度
	x0(10:13) = [z_w0_f; z_w0_f; z_w0_r; z_w0_r];
x0(14:17) = 0;  % 车轮垂向速度 = 0

% 车轮旋转角速度（按初始车速计算）
omega_w0 = p.vx0 / p.r_w;
x0(18:21) = omega_w0 * ones(4, 1);

	% 轮胎松弛力初始值 —— 基于初始扭矩平衡估算，消除初始瞬态波动
	% 纵向力平衡: T_drive - Fx*r_w - T_roll - T_brake ≈ 0
	% 侧向力: 初始为零（转向角ramp从0开始）
	[~, ~, ~, Fz_mf_init] = slope_load(u(10), u(11), p);
	T_roll_init = Fz_mf_init * p.f_roll * p.r_w;
	T_m_init = u(5:8);
	T_drive_init = motor_drive(T_m_init, x0(18:21), p);
	T_brake_init = brake(u(9), p);
	Fx_est = (T_drive_init - T_roll_init - T_brake_init) / p.r_w;
	% 摩擦椭圆限幅
	Fx_max_init = p.mu_x * Fz_mf_init;
	Fx_est = min(max(Fx_est, -Fx_max_init), Fx_max_init);
	x0(22:25) = Fx_est';   % 纵向松弛力初始值
	x0(26:29) = 0;         % 侧向松弛力初始值 = 0 (t=0时delta=0)
end
