function F_aero = aero(vx, p)
%% aero.m — 空气阻力计算
% F_aero = (1/2) * ρ * C_d * A_f * vx * |vx|
% 使用 vx * |vx| 确保倒车时力方向正确
%
% Input:
%   vx     — 车辆纵向速度 [m/s]
%   p      — 参数结构体
%
% Output:
%   F_aero — 空气阻力 [N]（方向始终与纵向运动方向相反）

F_aero = 0.5 * p.rho_air * p.C_d * p.A_f * vx * abs(vx);
end
