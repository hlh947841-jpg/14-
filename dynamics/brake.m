function T_brake = brake(beta_brk, p)
%% brake.m — 液压制动系统制动力矩计算
% 制动链: 踏板力 → 主缸压力 → 制动钳推力 → 制动力矩
%
% 力矩公式（全字母符号）:
%   F_p   = β_brk * F_p_max                          — 踏板力
%   P_b   = 4 * F_p * i_p * η₁ * k_boost / (π * d₁²) — 主缸压力
%   F_b,j = (π/4) * P_b * d₂,j² * η₂                  — 制动钳推力
%   T_b,j = 2 * μ_brake * F_b,j * r_b,j               — 单轮制动力矩
%
% Input:
%   beta_brk   — 制动踏板开度 [0~1]
%   p          — 参数结构体
%
% Output:
%   T_brake(4) — 四轮制动力矩 [N·m]
%
% 车轮编号: 1=FL, 2=FR, 3=RL, 4=RR

if beta_brk <= 0
    T_brake = zeros(4, 1);
    return;
end

% 制动踏板力
F_p = beta_brk * p.F_pedal_max;

% 制动主缸压力
P_b = 4 * F_p * p.i_brake_pedal * p.eta_brake_pedal * p.k_brake_boost ...
      / (pi * p.d_brake_master^2);

% 前/后制动钳推力
F_b_f = (pi / 4) * P_b * p.d_brake_caliper_f^2 * p.eta_brake_caliper;
F_b_r = (pi / 4) * P_b * p.d_brake_caliper_r^2 * p.eta_brake_caliper;

% 单轮制动力矩（制动盘两侧均有制动片，乘以2）
T_b_f = 2 * p.mu_brake_pad * F_b_f * p.r_brake_disc_f;
T_b_r = 2 * p.mu_brake_pad * F_b_r * p.r_brake_disc_r;

T_brake = [T_b_f; T_b_f; T_b_r; T_b_r];  % FL, FR, RL, RR
end
