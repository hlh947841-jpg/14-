%% run_sim_simulink.m - 14DOF Simulink 模型运行与 ODE 对照
% 混合结构：Simulink 管理 29 个积分器，核心动力学由 vehicle_dynamics.m 提供。
% 运行前会同步 ODE/Simulink 初始状态，并在结束后关闭模型但不保存 .slx。

clearvars -except scenario make_plot;
clc;

%% 路径初始化
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
root_dir = fileparts(script_dir);

addpath(root_dir);
addpath(fullfile(root_dir, 'config'));
addpath(fullfile(root_dir, 'control'));
addpath(fullfile(root_dir, 'dynamics'));
addpath(fullfile(root_dir, 'simulation'));
addpath(fullfile(root_dir, 'simulink'));

model_name = 'vehicle_14dof_4wis_4wid';
mdl_path = fullfile(root_dir, [model_name, '.slx']);
if ~exist(mdl_path, 'file')
    error('Simulink:MissingModel', ...
        ['找不到模型文件：%s\n' ...
         '为避免误删或覆盖，本脚本不会自动重建 .slx。确认需要重建后再手动运行 build_model。'], ...
        mdl_path);
end

p = vehicle_params();

%% 工况选择
if ~exist('scenario', 'var') || isempty(scenario)
    scenario = 2;
end
if ~exist('make_plot', 'var') || isempty(make_plot)
    make_plot = true;
end

u_ctrl = zeros(11, 1);
u_ctrl(5:8) = [80; 80; 50; 50];
stop_time = 5;

switch scenario
    case 1
        scenario_name = '直线行驶';
    case 2
        scenario_name = '前轮转向 2deg';
        u_ctrl(1:2) = deg2rad(2);
    case 3
        scenario_name = '四轮同向转向 2deg';
        u_ctrl(1:4) = deg2rad(2);
    case 4
        scenario_name = '纵坡直线';
        u_ctrl(11) = 0.1;
    case 5
        scenario_name = '横坡直线';
        u_ctrl(10) = 0.1;
    otherwise
        error('Simulink:UnknownScenario', '未知工况编号：%d', scenario);
end

%% 加载并配置模型
if ~bdIsLoaded(model_name)
    load_system(mdl_path);
end

input_names = {'delta_FL','delta_FR','delta_RL','delta_RR', ...
               'T_m_FL','T_m_FR','T_m_RL','T_m_RR', ...
               'beta_brk','i_h','i_c'};
for k = 1:numel(input_names)
    set_param([model_name, '/', input_names{k}], 'Value', sprintf('%.17g', u_ctrl(k)));
end

x0 = initial_state_14dof(p, u_ctrl);
configure_simulink_initial_conditions(model_name, x0);

set_param(model_name, ...
    'StopTime', sprintf('%.17g', stop_time), ...
    'Solver', 'ode15s', ...
    'MaxStep', '0.005', ...
    'RelTol', '1e-4', ...
    'AbsTol', '1e-6');

%% 运行 Simulink
fprintf('\n========== Simulink: %s ==========\n', scenario_name);
fprintf('目标车速: %.2f km/h, StopTime: %.1f s\n', p.vx0 * 3.6, stop_time);

clear torque_distribution;
tic;
simIn = Simulink.SimulationInput(model_name);
simIn = simIn.setVariable('p_14dof', p, 'Workspace', model_name);
simOut = sim(simIn);
elapsed = toc;

states = simOut.get('simout_states');
aux = simOut.get('simout_aux');
tout = simOut.get('tout_sim');

fprintf('耗时: %.2f s, 步数: %d\n', elapsed, length(tout));
fprintf('最终状态 t=%.1f s:\n', tout(end));
fprintf('  vx      = %.4f km/h\n', states(end, 1) * 3.6);
fprintf('  vy      = %.6f m/s\n', states(end, 2));
fprintf('  z       = %.6f m\n', states(end, 3));
fprintf('  phi     = %.6f deg\n', states(end, 5) * 180 / pi);
fprintf('  theta   = %.6f deg\n', states(end, 7) * 180 / pi);
fprintf('  psi_dot = %.6f deg/s\n', states(end, 9) * 180 / pi);
fprintf('  z_w     = [%.6f %.6f %.6f %.6f] m\n', states(end, 10:13));
if ~isempty(aux)
    fprintf('  LTR     = %.6f\n', aux(end, 49));
    fprintf('  beta_brk= %.6f (auto %.6f)\n', aux(end, 47), aux(end, 48));
end

%% ODE 对照
fprintf('\n=== ODE 对照 ===\n');
solver_opts = odeset('MaxStep', 0.005, 'RelTol', 1e-4, 'AbsTol', 1e-6, 'Stats', 'off');

clear torque_distribution;
[t_ode, x_ode] = ode15s(@(t, x) vehicle_dynamics(t, x, p, u_ctrl), ...
    [0 stop_time], x0, solver_opts);

x_interp = interp1(t_ode, x_ode, tout, 'linear', 'extrap');
vx_err = max(abs(x_interp(:, 1) - states(:, 1)));
state_abs_err = max(abs(x_interp - states), [], 1);
[all_state_err, worst_idx] = max(state_abs_err);
state_scale = max(max(abs(x_interp), [], 1), 1);
state_rel_err = state_abs_err ./ state_scale;
[max_rel_err, worst_rel_idx] = max(state_rel_err);
state_names = {'vx','vy','z','vz','phi','phi_dot','theta','theta_dot','psi_dot', ...
    'z_w_FL','z_w_FR','z_w_RL','z_w_RR', ...
    'z_w_dot_FL','z_w_dot_FR','z_w_dot_RL','z_w_dot_RR', ...
    'omega_FL','omega_FR','omega_RL','omega_RR', ...
    'Fx_relax_FL','Fx_relax_FR','Fx_relax_RL','Fx_relax_RR', ...
    'Fy_relax_FL','Fy_relax_FR','Fy_relax_RL','Fy_relax_RR'};

fprintf('vx 最大误差: %.3e m/s\n', vx_err);
fprintf('29 状态最大误差: %.3e\n', all_state_err);
fprintf('误差最大状态: %s\n', state_names{worst_idx});
fprintf('29 状态最大归一化误差: %.3e\n', max_rel_err);
fprintf('归一化误差最大状态: %s\n', state_names{worst_rel_idx});
if max_rel_err < 2e-3
    fprintf('Simulink 与 ODE 全状态一致性良好。\n');
else
    fprintf('注意：全状态误差偏大，请检查模型积分器初值或 S-Function 路径。\n');
end

%% 简单绘图
if make_plot
    figure('Name', ['14DOF Simulink - ', scenario_name], 'Position', [100 100 900 600]);
    subplot(2,3,1); plot(tout, states(:,1) * 3.6); ylabel('vx (km/h)'); grid on;
    subplot(2,3,2); plot(tout, states(:,2)); ylabel('vy (m/s)'); grid on;
    subplot(2,3,3); plot(tout, states(:,5) * 180 / pi); ylabel('phi (deg)'); grid on;
    subplot(2,3,4); plot(tout, states(:,7) * 180 / pi); ylabel('theta (deg)'); grid on;
    subplot(2,3,5); plot(tout, states(:,9) * 180 / pi); ylabel('psi dot (deg/s)'); grid on;
    subplot(2,3,6); plot(tout, states(:,10:13)); ylabel('z_w (m)'); grid on;
    sgtitle(sprintf('14DOF Simulink: %s', scenario_name));
end

set_param(model_name, 'Dirty', 'off');
fprintf('完成。模型保持加载但未保存。\n');
