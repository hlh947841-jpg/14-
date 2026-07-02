function configure_simulink_initial_conditions(model_name, x0)
%% configure_simulink_initial_conditions.m - 配置 Simulink 29 状态初值
% 将 ODE 使用的 29 维初始状态同步写入 Simulink 积分器。
% 只修改当前加载模型的运行参数，不保存 .slx 文件。

x0 = x0(:);
if numel(x0) ~= 29
    error('initial_state:SizeMismatch', 'x0 必须是 29 维列向量。');
end

subsys = [model_name, '/Vehicle_Dynamics'];
integrator_paths = {
    [subsys, '/Int_vx']
    [subsys, '/Int_vy']
    [subsys, '/Int_z']
    [subsys, '/Int_vz']
    [subsys, '/Int_phi']
    [subsys, '/Int_phi_dot']
    [subsys, '/Int_theta']
    [subsys, '/Int_theta_dot']
    [subsys, '/Int_psi_dot']
    [subsys, '/Int_z_w_1']
    [subsys, '/Int_z_w_2']
    [subsys, '/Int_z_w_3']
    [subsys, '/Int_z_w_4']
    [subsys, '/Int_z_w_dot_1']
    [subsys, '/Int_z_w_dot_2']
    [subsys, '/Int_z_w_dot_3']
    [subsys, '/Int_z_w_dot_4']
    [subsys, '/Int_omega_w_1']
    [subsys, '/Int_omega_w_2']
    [subsys, '/Int_omega_w_3']
    [subsys, '/Int_omega_w_4']
    [subsys, '/Int_Fx_relax_1']
    [subsys, '/Int_Fx_relax_2']
    [subsys, '/Int_Fx_relax_3']
    [subsys, '/Int_Fx_relax_4']
    [subsys, '/Int_Fy_relax_1']
    [subsys, '/Int_Fy_relax_2']
    [subsys, '/Int_Fy_relax_3']
    [subsys, '/Int_Fy_relax_4']
};

for i = 1:29
    if getSimulinkBlockHandle(integrator_paths{i}) < 0
        error('initial_state:MissingBlock', '找不到积分器块：%s', integrator_paths{i});
    end
    set_param(integrator_paths{i}, 'InitialCondition', sprintf('%.17g', x0(i)));
end
end
