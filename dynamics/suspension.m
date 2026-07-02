function [Fs, z_b, vz_b] = suspension(z, vz, theta, theta_dot, phi, phi_dot, z_w, z_w_dot, cos_g, p)
%% suspension.m — 非线性悬架模型D
% 弹簧力: F_spring = K_s1 * stroke + K_s2 * stroke³
% 阻尼力: F_damper = C_s1 * stroke_dot + C_s2 * |stroke_dot| * stroke_dot
% 总悬架力: F_si = F_s0i + F_spring_i + F_damper_i
%
% Input:
%   z, vz          — 车身簧上质量垂向位移/速度 [m, m/s]
%   theta, theta_dot — 俯仰角/角速度 [rad, rad/s]
%   phi, phi_dot     — 侧倾角/角速度 [rad, rad/s]
%   z_w(4), z_w_dot(4) — 四轮垂向位移/速度 [m, m/s]
%   cos_g          — 合成坡度角余弦（坡度上法向重力分量修正）
%   p              — 参数结构体
%
% Output:
%   Fs(4)          — 四角悬架力 [N]（正=车身受向上力，即悬架压缩）
%   z_b(4)         — 四角悬架车身安装点垂向位移 [m]
%   vz_b(4)        — 四角悬架车身安装点垂向速度 [m/s]
%
% 车轮编号: 1=FL(左前), 2=FR(右前), 3=RL(左后), 4=RR(右后)
% 坐标系: z轴向上为正

%% ===== 悬架车身安装点位移 =====
% z_b = z + long_off·θ + lat_off·φ (几何关系, phi>0=右倾时左侧上升右侧下降)
z_b = zeros(4, 1);
z_b(1) = z + p.a * theta + (p.B_sf / 2) * phi;   % FL: 前+左, 右倾↑
z_b(2) = z + p.a * theta - (p.B_sf / 2) * phi;   % FR: 前+右, 右倾↓
z_b(3) = z - p.b * theta + (p.B_sr / 2) * phi;   % RL: 后+左, 右倾↑
z_b(4) = z - p.b * theta - (p.B_sr / 2) * phi;   % RR: 后+右, 右倾↓

%% ===== 悬架车身安装点速度 =====
vz_b = zeros(4, 1);
vz_b(1) = vz + p.a * theta_dot + (p.B_sf / 2) * phi_dot;
vz_b(2) = vz + p.a * theta_dot - (p.B_sf / 2) * phi_dot;
vz_b(3) = vz - p.b * theta_dot + (p.B_sr / 2) * phi_dot;
vz_b(4) = vz - p.b * theta_dot - (p.B_sr / 2) * phi_dot;

%% ===== 静态预载 =====
% 静平衡时悬架力承担簧上质量，前后轴按质心位置分配
% 坡度上法向分量 = m_s * g * cos_g
F_s0 = zeros(4, 1);
F_s0(1) = p.m_s * p.g * p.b * cos_g / (2 * p.L);   % 前轴左
F_s0(2) = p.m_s * p.g * p.b * cos_g / (2 * p.L);   % 前轴右
F_s0(3) = p.m_s * p.g * p.a * cos_g / (2 * p.L);   % 后轴左
F_s0(4) = p.m_s * p.g * p.a * cos_g / (2 * p.L);   % 后轴右

%% ===== 非线性弹簧-阻尼力（模型D）=====
% 组装各轮悬架参数
K_s1 = [p.K_s1_f; p.K_s1_f; p.K_s1_r; p.K_s1_r];   % 线性刚度
K_s2 = [p.K_s2_f; p.K_s2_f; p.K_s2_r; p.K_s2_r];   % 三次刚度
C_s1 = [p.C_s1_f; p.C_s1_f; p.C_s1_r; p.C_s1_r];   % 名义线性阻尼
C_s2 = [p.C_s2_f; p.C_s2_f; p.C_s2_r; p.C_s2_r];   % 平方阻尼

% 由名义阻尼和回弹/压缩比计算非对称阻尼系数
% C_comp + C_reb = 2*C_s1 (保持每周期能量耗散不变)
C_comp_arr = 2 * C_s1 / (1 + p.xi_reb_comp);  % 压缩阻尼（较软）
C_reb_arr  = p.xi_reb_comp * C_comp_arr;       % 回弹阻尼（较硬）

Fs = zeros(4, 1);
for i = 1:4
    stroke     = z_w(i) - z_b(i);
    stroke_dot = z_w_dot(i) - vz_b(i);

    %% 非线性弹簧力: F_spring = K_s1 * stroke + K_s2 * stroke^3
    F_spring = K_s1(i) * stroke + K_s2(i) * stroke^3;

    %% 非对称非线性阻尼力
    % sigmoid 光滑分离压缩/回弹系数
    % s_comp ≈ 1 (压缩), s_comp ≈ 0 (回弹)
    s_comp = 1 / (1 + exp(-p.K_d_susp * stroke_dot));
    C_eff = C_comp_arr(i) * s_comp + C_reb_arr(i) * (1 - s_comp);
    % 线性阻尼 + 平方阻尼（速度超线性增长，双向对称）
    F_damper = C_eff * stroke_dot + C_s2(i) * abs(stroke_dot) * stroke_dot;

    %% 总悬架力 = 静态预载 + 弹簧力 + 阻尼力 (正=车身受向上力)
    Fs(i) = F_s0(i) + F_spring + F_damper;
end
end
