function [Fx, Fy] = tire_forces(alpha, kappa, Fz_mf, p)
%% tire_forces.m — Magic Formula 轮胎力 + 摩擦椭圆联合工况修正
% 先计算纯侧偏 / 纯纵滑 MF 力，再用摩擦椭圆耦合
%
% 纯纵滑: Fx0 = D_x*sin(C_x*atan(B_x*κ - E_x*(B_x*κ - atan(B_x*κ))))
% 纯侧偏: Fy0 = D_y*sin(C_y*atan(B_y*α - E_y*(B_y*α - atan(B_y*α))))
%
% 联合工况摩擦椭圆修正:
%   若 (Fx0/Fx_max)² + (Fy0/Fy_max)² > 1，等比例缩放至椭圆内
%
% Input:
%   alpha(4)   — 四轮侧偏角 [rad]
%   kappa(4)   — 四轮纵向滑移率 [-]
%   Fz_mf(4)   — 四轮准静态垂向载荷 [N]
%   p          — 参数结构体
%
% Output:
%   Fx(4)      — 四轮纵向力 [N]
%   Fy(4)      — 四轮侧向力 [N]

Fx = zeros(4, 1);
Fy = zeros(4, 1);

for i = 1:4
    Fz = max(Fz_mf(i), 100);  % 防止零或负垂向力

    % --- 纯纵滑 Magic Formula ---
    % D_x 按垂向载荷线性缩放: D_x(Fz) = D_x_ref * (Fz / Fz0)
    Dx = p.D_x * (Fz / p.Fz0);
    Dx = max(Dx, 1);
    x_arg_x = p.B_x * kappa(i);
    Fx0 = Dx * sin(p.C_x * atan(x_arg_x - p.E_x * (x_arg_x - atan(x_arg_x))));

    % --- 纯侧偏 Magic Formula ---
    % D_y 按垂向载荷非线性缩放: D_y(Fz) = D_y_ref * (Fz / Fz0)^n_Fz
    Dy = p.D_y * (Fz / p.Fz0)^p.n_Fz;
    Dy = max(Dy, 1);
    x_arg_y = p.B_y * alpha(i);
    Fy0 = Dy * sin(p.C_y * atan(x_arg_y - p.E_y * (x_arg_y - atan(x_arg_y))));

    % --- 摩擦椭圆联合工况修正 ---
    % Fx_max = μ_x * Fz, Fy_max = μ_y * Fz
    Fx_max = p.mu_x * Fz;
    Fy_max = p.mu_y * Fz;

    gamma_x = abs(Fx0) / max(Fx_max, 1e-6);
    gamma_y = abs(Fy0) / max(Fy_max, 1e-6);

    r2 = gamma_x^2 + gamma_y^2;
    if r2 > 1
        % 等比例缩放使合成力落在摩擦椭圆内
        r = 1 / sqrt(r2);
        Fx(i) = Fx0 * r;
        Fy(i) = Fy0 * r;
    else
        Fx(i) = Fx0;
        Fy(i) = Fy0;
    end
end
end
