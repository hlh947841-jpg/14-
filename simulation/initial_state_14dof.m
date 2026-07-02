function x0 = initial_state_14dof(p, u)
%% initial_state_14dof.m - 14DOF 模型统一初始状态
% 根据车辆参数和 11 维控制输入计算 ODE/Simulink 共用的初始状态。
% u = [delta(1:4); T_m(1:4); beta_brk; i_h; i_c]

u = u(:);
x0 = zeros(29, 1);

% 车身初始状态
x0(1) = p.vx0;
x0(2) = 0;
x0(4) = 0;
x0(5) = 0;
x0(6) = 0;
x0(7) = 0;
x0(8) = 0;
x0(9) = 0;

% 坡度会改变法向重力分量，初始轮胎压缩量应同步修正。
alpha_grad_init = atan(sqrt(u(10)^2 + u(11)^2));
cos_g_init = cos(alpha_grad_init);
F_s0_FL = p.m_s * p.g * p.b * cos_g_init / (2 * p.L);
F_s0_RL = p.m_s * p.g * p.a * cos_g_init / (2 * p.L);
z_w0_f = -(F_s0_FL + p.m_uf * p.g * cos_g_init) / p.K_t;
z_w0_r = -(F_s0_RL + p.m_ur * p.g * cos_g_init) / p.K_t;
z_w0 = [z_w0_f; z_w0_f; z_w0_r; z_w0_r];

x0(3) = mean(z_w0);
x0(10:13) = z_w0;
x0(14:17) = 0;

% 车轮旋转初值
omega_w0 = p.vx0 / p.r_w;
x0(18:21) = omega_w0 * ones(4, 1);

% 纵向松弛力初值按轮端扭矩平衡估算，减少 t=0 瞬态跳变。
[~, ~, ~, Fz_mf_init] = slope_load(u(10), u(11), p);
T_roll_init = Fz_mf_init * p.f_roll * p.r_w;
T_m_init = u(5:8);
T_drive_init = motor_drive(T_m_init, x0(18:21), p);
T_brake_init = brake(u(9), p);
Fx_est = (T_drive_init - T_roll_init - T_brake_init) / p.r_w;

Fx_max_init = p.mu_x * Fz_mf_init;
Fx_est = min(max(Fx_est, -Fx_max_init), Fx_max_init);
x0(22:25) = Fx_est(:);
x0(26:29) = 0;
end
