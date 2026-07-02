function T_drive = motor_drive(T_m, omega_w, p)
%% motor_drive.m — 四轮独立轮毂电机驱动模型
% 每个车轮由独立电机直接驱动，无中央传动链
%
% 电机外特性（sigmoid 光滑过渡）:
%   恒转矩区 (ω_w <= ω_base):  T = T_m (命令值)
%   恒功率区 (ω_w >  ω_base):  T = min(T_m, P_rated / ω_w)
%   过渡区: sigmoid 光滑混合，避免硬切换导致扭矩导数不连续
%
% 四轮独立意味着:
%   - 各轮扭矩可任意分配（转矩矢量控制）
%   - 无机械差速器约束
%   - 可实现驱动防滑 (ASR)、直接横摆力矩控制 (DYC) 等
%
% Input:
%   T_m(4)     — 四轮电机命令扭矩 [N·m]
%   omega_w(4) — 四轮转速 [rad/s]
%   p          — 参数结构体
%
% Output:
%   T_drive(4) — 四轮实际驱动力矩 [N·m]

T_drive = zeros(4, 1);
omega_base = p.P_m_rated / p.T_m_max;  % 基速 [rad/s]

% 恒功率区过渡锐度（sigmoid 光滑切换，避免硬 if/else 造成扭矩不连续）
% 值越大过渡越陡，2.0 在基速附近约 ±3 rad/s 范围内完成过渡
K_pwr = 2.0;

for i = 1:4
    % 扭矩限制（不超过电机峰值扭矩）
    T_cmd = min(max(T_m(i), -p.T_m_max), p.T_m_max);

    % 恒功率区扭矩衰减（sigmoid 光滑过渡）
    % s_pwr ≈ 0 → 恒转矩区; s_pwr ≈ 1 → 恒功率区
    omega_abs = abs(omega_w(i));
    s_pwr = 1 / (1 + exp(-K_pwr * (omega_abs - omega_base)));
    T_power_limit = p.P_m_rated / max(omega_abs, 1e-3);
    % 混合: 恒转矩扭矩与恒功率限幅扭矩的 sigmoid 加权
    T_limit = T_cmd * (1 - s_pwr) + min(abs(T_cmd), T_power_limit) * sign(T_cmd) * s_pwr;

    T_drive(i) = T_limit;
end
end
