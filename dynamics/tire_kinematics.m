function [alpha, kappa, vx_w] = tire_kinematics(vx, vy, psi_dot, omega_w, delta, p)
%% tire_kinematics.m — 轮胎运动学（四轮独立转向）
% 计算四轮侧偏角 α、纵向滑移率 κ、轮心纵向速度 vx_w
%
% 四轮独立转向: delta(1:4) 各轮可不同
%
% Input:
%   vx, vy       — 车身纵向/横向速度 [m/s]
%   psi_dot      — 横摆角速度 [rad/s]
%   omega_w(4)   — 四轮旋转角速度 [rad/s]
%   delta(4)     — 四轮独立转向角 [rad]
%   p            — 参数结构体
%
% Output:
%   alpha(4)     — 四轮侧偏角 [rad]
%   kappa(4)     — 四轮纵向滑移率 [-]
%   vx_w(4)      — 四轮轮心纵向速度（车轮坐标系）[m/s]
%
% 车轮编号: 1=FL(左前), 2=FR(右前), 3=RL(左后), 4=RR(右后)

%% ===== 各轮轮心在车辆坐标系下的速度分量 =====
% 纵向分量: vx_hub_i = vx ∓ (B/2) * ψ_dot  （左轮-, 右轮+）
vx_hub = zeros(4, 1);
vx_hub(1) = vx - p.B_f/2 * psi_dot;   % FL
vx_hub(2) = vx + p.B_f/2 * psi_dot;   % FR
vx_hub(3) = vx - p.B_r/2 * psi_dot;   % RL
vx_hub(4) = vx + p.B_r/2 * psi_dot;   % RR

% 横向分量: vy_hub_i = vy + a * ψ_dot  (前), vy - b * ψ_dot (后)
vy_hub = zeros(4, 1);
vy_hub(1) = vy + p.a * psi_dot;       % FL
vy_hub(2) = vy + p.a * psi_dot;       % FR
vy_hub(3) = vy - p.b * psi_dot;       % RL
vy_hub(4) = vy - p.b * psi_dot;       % RR

%% ===== 轮心纵向速度（车轮坐标系下）=====
% vx_w_i = vx_hub_i * cos(δ_i) + vy_hub_i * sin(δ_i)
vx_w = zeros(4, 1);
for i = 1:4
    vx_w(i) = vx_hub(i) * cos(delta(i)) + vy_hub(i) * sin(delta(i));
end

%% ===== 侧偏角 =====
% α_i = δ_i - atan2(vy_hub_i, vx_hub_i)
% 用 atan2 避免除零奇异，全程光滑
alpha = zeros(4, 1);
for i = 1:4
    alpha(i) = delta(i) - atan2(vy_hub(i), vx_hub(i));
end

%% ===== 纵向滑移率（光滑正则化）=====
% κ_i = (ω_i * r_w - vx_w_i) / sqrt((ω_i * r_w)² + ε_v²)
kappa = zeros(4, 1);
for i = 1:4
    denom = sqrt((omega_w(i) * p.r_w)^2 + p.eps_v^2);
    kappa(i) = (omega_w(i) * p.r_w - vx_w(i)) / denom;
end
end
