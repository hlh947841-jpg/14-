function [cos_g, sin_g_lat, sin_g_lon, Fz_mf] = slope_load(i_h_eff, i_c_eff, p)
%% slope_load.m — 坡度角计算 + 准静态垂向载荷分配
%
% Input:
%   i_h_eff — 有效横向坡度比 (经过 ramp 平滑)
%   i_c_eff — 有效纵向坡度比 (经过 ramp 平滑)
%   p       — 参数结构体
%
% Output:
%   cos_g      — 合成坡度角余弦
%   sin_g_lat  — 横向坡度角正弦
%   sin_g_lon  — 纵向坡度角正弦
%   Fz_mf(4)   — 四轮准静态垂向载荷 [N]（用于 Magic Formula 缩放）

%% ===== 坡度角 =====
alpha_grad = atan(sqrt(i_h_eff^2 + i_c_eff^2));   % 合成坡度角
alpha_lat  = atan(i_h_eff);                        % 横向坡度角
alpha_lon  = atan(i_c_eff);                        % 纵向坡度角

% 限幅至安全范围
alpha_grad = min(max(alpha_grad, -p.slope_max_rad), p.slope_max_rad);
alpha_lat  = min(max(alpha_lat,  -p.slope_max_rad), p.slope_max_rad);
alpha_lon  = min(max(alpha_lon,  -p.slope_max_rad), p.slope_max_rad);

cos_g     = cos(alpha_grad);
sin_g_lat = sin(alpha_lat);
sin_g_lon = sin(alpha_lon);

%% ===== 准静态垂向载荷（含坡度载荷转移）=====
% 纵向坡度引起的轴荷转移
% ΔFz_lon = M * g * h_g * sin(α_lon) / (2 * L)   —— 每轮变化量
dFz_lon = p.M * p.g * p.h_g * sin_g_lon / (2 * p.L);

% 横向坡度引起的轮荷转移（左右轮间，单轮变化量）
% 总倾覆力矩 M*g*h_g*sin(α_lat) 由前后轴共同承担
% 若前后均分（等轮距），每轮载荷变化 = M*g*h_g*sin(α_lat) / (2*B)
% 前轴单轮: ΔFz_lat_f = M * g * h_g * sin(α_lat) / (2 * B_f)
% 后轴单轮: ΔFz_lat_r = M * g * h_g * sin(α_lat) / (2 * B_r)
dFz_lat_f = p.M * p.g * p.h_g * sin_g_lat / (2 * p.B_f);
dFz_lat_r = p.M * p.g * p.h_g * sin_g_lat / (2 * p.B_r);

% 平坦地面各轮静载荷（考虑合成坡度对法向力的影响）
% Fz0_f = M * g * b * cos(α_grad) / (2 * L)
% Fz0_r = M * g * a * cos(α_grad) / (2 * L)
Fz0_f = p.M * p.g * p.b * cos_g / (2 * p.L);
Fz0_r = p.M * p.g * p.a * cos_g / (2 * p.L);

% 四轮准静态垂向载荷
Fz_mf = zeros(4, 1);
Fz_mf(1) = Fz0_f - dFz_lon - dFz_lat_f;   % FL: 前-纵移-左移
Fz_mf(2) = Fz0_f - dFz_lon + dFz_lat_f;   % FR: 前-纵移+左移
Fz_mf(3) = Fz0_r + dFz_lon - dFz_lat_r;   % RL: 后+纵移-左移
Fz_mf(4) = Fz0_r + dFz_lon + dFz_lat_r;   % RR: 后+纵移+左移

% 防止零或负值
Fz_mf = max(Fz_mf, 100);
end
