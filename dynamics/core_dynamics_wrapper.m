function [dx, aux_vec] = core_dynamics_wrapper(t, x, u_ctrl, p)
%% core_dynamics_wrapper.m — vehicle_dynamics 的 S-Function 兼容包装
% S-Function 调用约定: func_name(u{1}, u{2}, ..., u{n_in}, p)
% 即参数顺序为 (t, x, u_ctrl, p)
% vehicle_dynamics 原始签名: (t, x, p, u)
% 本包装函数完成参数顺序转换
%
% 输入:
%   t      — 当前仿真时间 [s]（标量）
%   x      — 29维状态向量
%   u_ctrl — 11维控制输入向量 [delta(4); T_m(4); beta_brk; i_h; i_c]
%   p      — 车辆参数结构体
%
% 输出:
%   dx      — 29维状态导数
%   aux_vec — 52维诊断向量（轮荷/轮胎力/滑移率/扭矩等）

[dx, aux] = vehicle_dynamics(t, x, p, u_ctrl);
if nargout > 1
    aux_vec = aux_vector_14dof(aux);
end
end
