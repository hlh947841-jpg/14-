function Fz_tire = tire_vertical(z_w, z_w_dot, p)
%% tire_vertical.m — 轮胎垂向力计算
% 地面通过轮胎给车轮的垂向支撑力（光滑近似）
%
% Input:
%   z_w(4)     — 四轮垂向位移 [m]（地面 z=0，向上为正）
%   z_w_dot(4) — 四轮垂向速度 [m/s]
%   p          — 参数结构体
%
% Output:
%   Fz_tire(4) — 轮胎垂向力 [N]（地面法向支撑力，只能推不能拉）

Fz_tire = zeros(4, 1);
for i = 1:4
    % 轮胎压入地面深度（光滑近似: max(0 - z_w, 0)）
    % 当车轮低于地面 (z_w < 0) 时产生正向支撑力
    penetration = 0.5 * ((0 - z_w(i)) + sqrt((0 - z_w(i))^2 + p.eps_gnd^2));

    % 线性弹簧 + 线性阻尼
    Fz_tire(i) = p.K_t * penetration + p.C_t * (0 - z_w_dot(i));

    % 光滑 clamp：地面只能推不能拉（过渡宽度 = eps_gnd * K_t ≈ 210 N）
    eps_clamp = p.eps_gnd * p.K_t;
    Fz_tire(i) = 0.5 * (Fz_tire(i) + sqrt(Fz_tire(i)^2 + eps_clamp^2));
end
end
