function psi_dot_ref = yaw_rate_reference(vx, delta_f, p, Fz_mf)
%% yaw_rate_reference.m — 线性自行车模型稳态目标横摆角速度（坡度感知）
% 基于线性二自由度自行车模型计算参考横摆角速度
% 可选第四参数 Fz_mf(4) 提供动态轴荷（坡度感知），不传则用静态轴荷
%
% 公式:
%   ψ_dot_ref = vx / (L + K_us * vx²) * δ_f
%
% 其中不足转向梯度:
%   K_us = M/L * (b/C_αf - a/C_αr)
%
% 侧偏刚度从 Magic Formula 参数近似:
%   C_αf ≈ 2 * D_y * B_y * (Fz_f / Fz0)^n_Fz   [N/rad]（前轴，双轮）
%   C_αr ≈ 2 * D_y * B_y * (Fz_r / Fz0)^n_Fz   [N/rad]（后轴，双轮）
%
% Input:
%   vx      — 纵向车速 [m/s]
%   delta_f — 前轴等效转向角 [rad]（取左右前轮平均值）
%   p       — 车辆参数结构体
%   Fz_mf   — (可选) 四轮准静态垂向载荷 [N]，用于坡度感知轴荷
%
% Output:
%   psi_dot_ref — 目标横摆角速度 [rad/s]（受摩擦圆限幅）

%% 计算轴荷（坡度感知或静态回退）
if nargin >= 4 && ~isempty(Fz_mf)
    % 使用动态轴荷（来自 slope_load，已含坡度载荷转移）
    Fz_f = (Fz_mf(1) + Fz_mf(2)) / 2;   % 前轴单轮均值 [N]
    Fz_r = (Fz_mf(3) + Fz_mf(4)) / 2;   % 后轴单轮均值 [N]
else
    % 回退：静态平路轴荷
    Fz_f = p.M * p.g * p.b / (2 * p.L);   % 前轴单轮静载 [N]
    Fz_r = p.M * p.g * p.a / (2 * p.L);   % 后轴单轮静载 [N]
end

%% 侧偏刚度（单轴双轮合计，基于MF参数近似，使用动态轴荷）
C_alpha_f = 2 * p.D_y * p.B_y * (Fz_f / p.Fz0)^p.n_Fz;   % 前轴 [N/rad]
C_alpha_r = 2 * p.D_y * p.B_y * (Fz_r / p.Fz0)^p.n_Fz;   % 后轴 [N/rad]

%% 不足转向梯度
% K_us > 0 → 不足转向（稳定），K_us < 0 → 过度转向
K_us = p.M / p.L * (p.b / C_alpha_f - p.a / C_alpha_r);

%% 稳态横摆角速度
if abs(vx) < p.eps_v
    psi_dot_ref = 0;
else
    psi_dot_ref = vx / (p.L + K_us * vx^2) * delta_f;
end

%% 摩擦圆限幅（轮胎附着力物理极限）
% 最大横摆角速度受限于轮胎-路面摩擦系数
psi_dot_max = p.mu_y * p.g / max(abs(vx), p.eps_v);

% 光滑限幅（sigmoid过渡，避免硬截断导致导数不连续）
if abs(psi_dot_ref) > psi_dot_max
    % 使用 tanh 光滑限幅，在限幅边界附近连续可导
    ratio = psi_dot_ref / psi_dot_max;
    psi_dot_ref = psi_dot_max * tanh(ratio);
end

end
