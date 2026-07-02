function build_model(root_dir, p)
%% build_model.m — 混合架构构建 14-DOF Simulink 模型
% 架构设计:
%   - 顶层: 控制输入路由 + Scope 显示 + To Workspace 输出
%   - Vehicle_Dynamics 内部: 单个核心动力学 S-Function + 29 积分器 + 诊断输出
%   - 核心 S-Function 包装 vehicle_dynamics.m（已验证的 ODE 函数）
%   - 单一块计算消除模块间代数环，同时保持各状态分量可视
%
% 状态 (29):
%   车身9: vx, vy, z, vz, phi, phi_dot, theta, theta_dot, psi_dot
%   车轮垂向8: z_w(4), z_w_dot(4)
%   车轮旋转4: omega_w(4)
%   松弛力8: Fx_relax(4), Fy_relax(4)

%% 默认参数
if nargin < 1, root_dir = pwd; end
if nargin < 2, p = vehicle_params(); end

model_name = 'vehicle_14dof_4wis_4wid';
mdl_path = fullfile(root_dir, [model_name, '.slx']);

% 关闭已存在的模型
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
% 删除旧模型文件（避免 genpath 导致同名冲突）
if exist(mdl_path, 'file')
    delete(mdl_path);
end

%% 创建新模型
fprintf('========================================\n');
fprintf('  构建混合架构14-DOF Simulink模型\n');
fprintf('  核心: 单个动力学S-Function + 29积分器\n');
fprintf('========================================\n');
ws = warning('off', 'Simulink:Engine:ShadowingNameOnPath');
new_system(model_name);
warning(ws);
open_system(model_name);

set_param(model_name, 'Solver', 'ode15s', ...
    'MaxStep', '0.005', 'RelTol', '1e-4', 'AbsTol', '1e-6', ...
    'StopTime', '5', 'StartTime', '0');

%% 将参数写入模型工作区
mdl_wks = get_param(model_name, 'ModelWorkspace');
assignin(mdl_wks, 'p_14dof', p);

%% ===== 生成核心动力学 S-Function =====
fprintf('\n--- 生成核心动力学 S-Function ---\n');
sfun_dir = fullfile(root_dir, 'dynamics');
addpath(sfun_dir);

% 为 core_dynamics_wrapper 生成 S-Function
% 3 输入: t(标量), x(29维), u_ctrl(11维) → 2 输出: dx(29维), aux(52维)
create_sfun_subsystem(sfun_dir, 'core_dynamics_wrapper', 3, 2, [1 29 11], [29 52]);

fprintf('核心 S-Function 已生成。\n');

%% ===== 构建顶层模型 =====
fprintf('\n--- 构建模型拓扑 ---\n');

% --- 时间源 ---
add_block('simulink/Sources/Clock', [model_name, '/Clock']);
set_param([model_name, '/Clock'], 'Position', [30, 50, 70, 80]);

% --- 控制输入块 + Mux ---
input_names = {'delta_FL','delta_FR','delta_RL','delta_RR', ...
               'T_m_FL','T_m_FR','T_m_RL','T_m_RR', ...
               'beta_brk','i_h','i_c'};
input_defaults = [0, 0, 0, 0, 80, 80, 50, 50, 0, 0, 0];

add_block('simulink/Signal Routing/Mux', [model_name, '/Control_Mux']);
set_param([model_name, '/Control_Mux'], 'Inputs', '11', ...
    'Position', [300, 100, 325, 500]);

for k = 1:11
    blk = [model_name, '/', input_names{k}];
    add_block('simulink/Sources/Constant', blk);
    set_param(blk, 'Value', num2str(input_defaults(k)), ...
        'Position', [100, 60 + k*35, 180, 85 + k*35]);
    add_line(model_name, [input_names{k}, '/1'], ['Control_Mux/', num2str(k)]);
end

% --- 主动力学子系统 ---
create_vehicle_dynamics_subsystem(model_name, root_dir, p);

% 连接顶层信号
add_line(model_name, 'Clock/1', 'Vehicle_Dynamics/1');
add_line(model_name, 'Control_Mux/1', 'Vehicle_Dynamics/2');

% --- 输出 Scope ---
% 车身Scope: vx, vy, z, phi, theta, psi_dot
add_block('simulink/Sinks/Scope', [model_name, '/Scope_Body']);
set_param([model_name, '/Scope_Body'], 'NumInputPorts', '1', ...
    'Position', [750, 50, 880, 200]);

add_block('simulink/Signal Routing/Selector', [model_name, '/Sel_Body']);
set_param([model_name, '/Sel_Body'], ...
    'InputPortWidth', '29', 'IndexOptions', 'Index vector (dialog)', ...
    'Indices', '[1 2 3 5 7 9]', 'OutputSizes', '6', ...
    'Position', [620, 50, 660, 120]);
add_line(model_name, 'Vehicle_Dynamics/1', 'Sel_Body/1');
add_line(model_name, 'Sel_Body/1', 'Scope_Body/1');

% 车轮Scope: z_w(4), z_w_dot(4)
add_block('simulink/Sinks/Scope', [model_name, '/Scope_Wheels']);
set_param([model_name, '/Scope_Wheels'], 'NumInputPorts', '1', ...
    'Position', [750, 230, 880, 380]);

add_block('simulink/Signal Routing/Selector', [model_name, '/Sel_Wheels']);
set_param([model_name, '/Sel_Wheels'], ...
    'InputPortWidth', '29', 'IndexOptions', 'Index vector (dialog)', ...
    'Indices', '[10 11 12 13 14 15 16 17]', 'OutputSizes', '8', ...
    'Position', [620, 230, 660, 300]);
add_line(model_name, 'Vehicle_Dynamics/1', 'Sel_Wheels/1');
add_line(model_name, 'Sel_Wheels/1', 'Scope_Wheels/1');

% 导数Scope (加速度/加加速度)
add_block('simulink/Sinks/Scope', [model_name, '/Scope_Accel']);
set_param([model_name, '/Scope_Accel'], 'NumInputPorts', '1', ...
    'Position', [750, 410, 880, 560]);

add_block('simulink/Signal Routing/Selector', [model_name, '/Sel_Accel']);
set_param([model_name, '/Sel_Accel'], ...
    'InputPortWidth', '29', 'IndexOptions', 'Index vector (dialog)', ...
    'Indices', '[1 2 4 6 8 9]', 'OutputSizes', '6', ...
    'Position', [620, 410, 660, 480]);
add_line(model_name, 'Vehicle_Dynamics/2', 'Sel_Accel/1');
add_line(model_name, 'Sel_Accel/1', 'Scope_Accel/1');

% To Workspace 块
add_block('simulink/Sinks/To Workspace', [model_name, '/To_WS_States']);
set_param([model_name, '/To_WS_States'], ...
    'VariableName', 'simout_states', 'SaveFormat', 'Array', ...
    'Position', [750, 600, 870, 640]);
add_line(model_name, 'Vehicle_Dynamics/1', 'To_WS_States/1');

add_block('simulink/Sinks/To Workspace', [model_name, '/To_WS_Derivs']);
set_param([model_name, '/To_WS_Derivs'], ...
    'VariableName', 'simout_derivs', 'SaveFormat', 'Array', ...
    'Position', [750, 670, 870, 710]);
add_line(model_name, 'Vehicle_Dynamics/2', 'To_WS_Derivs/1');

add_block('simulink/Sinks/To Workspace', [model_name, '/To_WS_Aux']);
set_param([model_name, '/To_WS_Aux'], ...
    'VariableName', 'simout_aux', 'SaveFormat', 'Array', ...
    'Position', [750, 740, 870, 780]);
add_line(model_name, 'Vehicle_Dynamics/3', 'To_WS_Aux/1');

add_block('simulink/Sinks/To Workspace', [model_name, '/To_WS_Time']);
set_param([model_name, '/To_WS_Time'], ...
    'VariableName', 'tout_sim', 'SaveFormat', 'Array', ...
    'Position', [750, 810, 870, 850]);
add_line(model_name, 'Clock/1', 'To_WS_Time/1');

%% 自动布局
try
    Simulink.BlockDiagram.arrangeSystem(model_name);
catch
end

%% 保存
save_system(model_name, mdl_path);
fprintf('\n========================================\n');
fprintf('  模型已保存: %s\n', mdl_path);
fprintf('  架构: 单核心S-Function + 29积分器\n');
fprintf('  状态全部由Simulink积分器管理\n');
fprintf('  信号: Mux/Demux 直接路由\n');
fprintf('========================================\n');
end

%% ===== 主动力学子系统构建 =====
function create_vehicle_dynamics_subsystem(model_name, root_dir, p)
% 构建包含29个积分器和单个核心动力学S-Function的子系统
%
% 内部结构:
%   [Time, Ctrl] → Core_Dynamics_SFun → dx(29) → Demux_Derivs → Integrators
%                                                                    ↓
%                                          Mux_States(29) ← ← ← ← ←
%
% 位置分配:
%   x=100:  输入端口
%   x=250:  Core_Dynamics 子系统
%   x=450:  Demux_Derivs
%   x=700:  29 积分器（按功能分组排列）
%   x=950:  Mux_AllStates / Mux_AllDerivs
%   x=1100: 输出端口

sub_name = [model_name, '/Vehicle_Dynamics'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_name);
set_param(sub_name, 'Position', [400, 100, 550, 750]);
set_param(sub_name, 'BackgroundColor', 'lightBlue');

% 删除子系统内默认内容
Simulink.SubSystem.deleteContents(sub_name);

%% ===== 输入/输出端口 =====
add_block('simulink/Sources/In1', [sub_name, '/Time']);
set_param([sub_name, '/Time'], 'Position', [50, 80, 80, 100]);

add_block('simulink/Sources/In1', [sub_name, '/Ctrl']);
set_param([sub_name, '/Ctrl'], 'Position', [50, 180, 80, 200]);
set_param([sub_name, '/Ctrl'], 'PortDimensions', '11');

add_block('simulink/Sinks/Out1', [sub_name, '/States_Out']);
set_param([sub_name, '/States_Out'], 'Position', [1050, 200, 1080, 230]);
set_param([sub_name, '/States_Out'], 'PortDimensions', '29');

add_block('simulink/Sinks/Out1', [sub_name, '/Derivs_Out']);
set_param([sub_name, '/Derivs_Out'], 'Position', [1050, 350, 1080, 380]);
set_param([sub_name, '/Derivs_Out'], 'PortDimensions', '29');

add_block('simulink/Sinks/Out1', [sub_name, '/Aux_Out']);
set_param([sub_name, '/Aux_Out'], 'Position', [1050, 500, 1080, 530]);
set_param([sub_name, '/Aux_Out'], 'PortDimensions', '52');

%% ===== 计算静平衡初值（平路默认，坡度修正由仿真时ODE动态求解）=====
cos_g0 = 1;  % Simulink模型构建时默认平路，坡度由控制输入u给定
F_s0_FL = p.m_s * p.g * p.b * cos_g0 / (2 * p.L);
F_s0_RL = p.m_s * p.g * p.a * cos_g0 / (2 * p.L);
z_w0_f = -(F_s0_FL + p.m_uf * p.g * cos_g0) / p.K_t;  % 前轮静平衡压缩量（负值=压缩）
z_w0_r = -(F_s0_RL + p.m_ur * p.g * cos_g0) / p.K_t;  % 后轮静平衡压缩量
z_w0 = [z_w0_f; z_w0_f; z_w0_r; z_w0_r];
omega_w0 = p.vx0 / p.r_w;

%% ===== 29 个积分器 =====
% 分组排列，便于可视化
%
% 车身状态积分器 (9个) — 行1-9
%   注意: z, phi, theta 的导数由其速度积分器输出驱动（级联）
%         即 Int_z 由 Int_vz 输出驱动, Int_phi 由 Int_phi_dot 输出驱动,
%         Int_theta 由 Int_theta_dot 输出驱动
body_state_names = {'vx','vy','z','vz','phi','phi_dot','theta','theta_dot','psi_dot'};
body_init = [p.vx0, 0, mean(z_w0), 0, 0, 0, 0, 0, 0];

for i = 1:9
    blk = [sub_name, '/Int_', body_state_names{i}];
    add_block('simulink/Continuous/Integrator', blk);
    set_param(blk, 'InitialCondition', num2str(body_init(i)), ...
        'Position', [680, 40 + (i-1)*38, 715, 60 + (i-1)*38]);
end

% 车轮垂向位移积分器 (4个) — z_w(1:4), 行10-13
% 由 z_w_dot 积分器输出驱动（级联）
for i = 1:4
    blk = [sub_name, '/Int_z_w_', num2str(i)];
    add_block('simulink/Continuous/Integrator', blk);
    set_param(blk, 'InitialCondition', num2str(z_w0(i)), ...
        'Position', [680, 400 + (i-1)*38, 715, 420 + (i-1)*38]);
end

% 车轮垂向速度积分器 (4个) — z_w_dot(1:4), 行14-17
for i = 1:4
    blk = [sub_name, '/Int_z_w_dot_', num2str(i)];
    add_block('simulink/Continuous/Integrator', blk);
    set_param(blk, 'InitialCondition', '0', ...
        'Position', [680, 560 + (i-1)*38, 715, 580 + (i-1)*38]);
end

% 车轮旋转积分器 (4个) — omega_w(1:4), 行18-21
for i = 1:4
    blk = [sub_name, '/Int_omega_w_', num2str(i)];
    add_block('simulink/Continuous/Integrator', blk);
    set_param(blk, 'InitialCondition', num2str(omega_w0), ...
        'Position', [680, 720 + (i-1)*38, 715, 740 + (i-1)*38]);
end

% 松弛力积分器 (8个) — Fx_relax(1:4), Fy_relax(1:4), 行22-29
for i = 1:4
    blk = [sub_name, '/Int_Fx_relax_', num2str(i)];
    add_block('simulink/Continuous/Integrator', blk);
    set_param(blk, 'InitialCondition', '0', ...
        'Position', [680, 880 + (i-1)*38, 715, 900 + (i-1)*38]);
end
for i = 1:4
    blk = [sub_name, '/Int_Fy_relax_', num2str(i)];
    add_block('simulink/Continuous/Integrator', blk);
    set_param(blk, 'InitialCondition', '0', ...
        'Position', [680, 1040 + (i-1)*38, 715, 1060 + (i-1)*38]);
end

%% ===== 状态 Mux (29 积分器输出 → 29维向量，反馈给核心 S-Function) =====
add_block('simulink/Signal Routing/Mux', [sub_name, '/Mux_States']);
set_param([sub_name, '/Mux_States'], 'Inputs', '29', ...
    'Position', [500, 100, 520, 1000]);

% 按 vehicle_dynamics.m 中 x(1:29) 的顺序连接
mux_idx = 1;
% x(1:9): 车身状态
for i = 1:9
    add_line(sub_name, ['Int_', body_state_names{i}, '/1'], ...
        ['Mux_States/', num2str(mux_idx)]);
    mux_idx = mux_idx + 1;
end
% x(10:13): z_w(1:4)
for i = 1:4
    add_line(sub_name, ['Int_z_w_', num2str(i), '/1'], ...
        ['Mux_States/', num2str(mux_idx)]);
    mux_idx = mux_idx + 1;
end
% x(14:17): z_w_dot(1:4)
for i = 1:4
    add_line(sub_name, ['Int_z_w_dot_', num2str(i), '/1'], ...
        ['Mux_States/', num2str(mux_idx)]);
    mux_idx = mux_idx + 1;
end
% x(18:21): omega_w(1:4)
for i = 1:4
    add_line(sub_name, ['Int_omega_w_', num2str(i), '/1'], ...
        ['Mux_States/', num2str(mux_idx)]);
    mux_idx = mux_idx + 1;
end
% x(22:25): Fx_relax(1:4)
for i = 1:4
    add_line(sub_name, ['Int_Fx_relax_', num2str(i), '/1'], ...
        ['Mux_States/', num2str(mux_idx)]);
    mux_idx = mux_idx + 1;
end
% x(26:29): Fy_relax(1:4)
for i = 1:4
    add_line(sub_name, ['Int_Fy_relax_', num2str(i), '/1'], ...
        ['Mux_States/', num2str(mux_idx)]);
    mux_idx = mux_idx + 1;
end

%% ===== 核心动力学 S-Function 子系统 =====
% 3 输入: t(标量), x(29维), u_ctrl(11维) → 2 输出: dx(29维), aux(52维)
create_core_dynamics_block(sub_name);

% 连接外部信号
add_line(sub_name, 'Time/1', 'Core_Dynamics/1');
add_line(sub_name, 'Mux_States/1', 'Core_Dynamics/2');
add_line(sub_name, 'Ctrl/1', 'Core_Dynamics/3');

%% ===== 导数 Demux (29维 → 29个单独信号) =====
add_block('simulink/Signal Routing/Demux', [sub_name, '/Demux_Derivs']);
set_param([sub_name, '/Demux_Derivs'], 'Outputs', '29', ...
    'Position', [400, 100, 430, 1000]);
add_line(sub_name, 'Core_Dynamics/1', 'Demux_Derivs/1');

% 连接导数到各积分器
% 注意: dx 中某些分量是状态复制而非新计算的导数
%   dx(3)=vz  → Int_z 由 Int_vz 输出驱动（级联）
%   dx(5)=phi_dot → Int_phi 由 Int_phi_dot 输出驱动（级联）
%   dx(7)=theta_dot → Int_theta 由 Int_theta_dot 输出驱动（级联）
%   这些 Demux 端口不直接连接积分器

% dx(1)=ax → Int_vx
add_line(sub_name, 'Demux_Derivs/1', 'Int_vx/1');
% dx(2)=ay → Int_vy
add_line(sub_name, 'Demux_Derivs/2', 'Int_vy/1');
% dx(3)=vz → (级联: Int_vz 输出驱动 Int_z, 见下方)
% dx(4)=az → Int_vz
add_line(sub_name, 'Demux_Derivs/4', 'Int_vz/1');
% dx(5)=phi_dot → (级联: Int_phi_dot 输出驱动 Int_phi)
% dx(6)=phi_ddot → Int_phi_dot
add_line(sub_name, 'Demux_Derivs/6', 'Int_phi_dot/1');
% dx(7)=theta_dot → (级联)
% dx(8)=theta_ddot → Int_theta_dot
add_line(sub_name, 'Demux_Derivs/8', 'Int_theta_dot/1');
% dx(9)=psi_ddot → Int_psi_dot
add_line(sub_name, 'Demux_Derivs/9', 'Int_psi_dot/1');

% dx(10:13)=z_w_dot → (级联: Int_z_w_dot 输出驱动 Int_z_w, 见下方)
% dx(14:17)=z_w_ddot → Int_z_w_dot(1:4)
for i = 1:4
    add_line(sub_name, ['Demux_Derivs/', num2str(13+i)], ...
        ['Int_z_w_dot_', num2str(i), '/1']);
end
% dx(18:21)=omega_w_dot → Int_omega_w(1:4)
for i = 1:4
    add_line(sub_name, ['Demux_Derivs/', num2str(17+i)], ...
        ['Int_omega_w_', num2str(i), '/1']);
end
% dx(22:25)=dFx_relax → Int_Fx_relax(1:4)
for i = 1:4
    add_line(sub_name, ['Demux_Derivs/', num2str(21+i)], ...
        ['Int_Fx_relax_', num2str(i), '/1']);
end
% dx(26:29)=dFy_relax → Int_Fy_relax(1:4)
for i = 1:4
    add_line(sub_name, ['Demux_Derivs/', num2str(25+i)], ...
        ['Int_Fy_relax_', num2str(i), '/1']);
end

%% ===== 级联积分器连接（位置→速度 关系）=====
% Int_vz 输出(vz) → Int_z 输入 (dz/dt = vz)
add_line(sub_name, 'Int_vz/1', 'Int_z/1');
% Int_phi_dot 输出(phi_dot) → Int_phi 输入 (dφ/dt = φ_dot)
add_line(sub_name, 'Int_phi_dot/1', 'Int_phi/1');
% Int_theta_dot 输出(theta_dot) → Int_theta 输入 (dθ/dt = θ_dot)
add_line(sub_name, 'Int_theta_dot/1', 'Int_theta/1');
% Int_z_w_dot_i 输出 → Int_z_w_i 输入 (dz_w/dt = z_w_dot)
for i = 1:4
    add_line(sub_name, ['Int_z_w_dot_', num2str(i), '/1'], ...
        ['Int_z_w_', num2str(i), '/1']);
end

%% ===== 输出 Mux =====
% 状态输出 Mux
add_block('simulink/Signal Routing/Mux', [sub_name, '/Mux_AllStates']);
set_param([sub_name, '/Mux_AllStates'], 'Inputs', '29', ...
    'Position', [920, 100, 940, 1000]);

mux_idx = 1;
for i = 1:9
    add_line(sub_name, ['Int_', body_state_names{i}, '/1'], ...
        ['Mux_AllStates/', num2str(mux_idx)]);
    mux_idx = mux_idx + 1;
end
for i = 1:4
    add_line(sub_name, ['Int_z_w_', num2str(i), '/1'], ...
        ['Mux_AllStates/', num2str(mux_idx)]);
    mux_idx = mux_idx + 1;
end
for i = 1:4
    add_line(sub_name, ['Int_z_w_dot_', num2str(i), '/1'], ...
        ['Mux_AllStates/', num2str(mux_idx)]);
    mux_idx = mux_idx + 1;
end
for i = 1:4
    add_line(sub_name, ['Int_omega_w_', num2str(i), '/1'], ...
        ['Mux_AllStates/', num2str(mux_idx)]);
    mux_idx = mux_idx + 1;
end
for i = 1:4
    add_line(sub_name, ['Int_Fx_relax_', num2str(i), '/1'], ...
        ['Mux_AllStates/', num2str(mux_idx)]);
    mux_idx = mux_idx + 1;
end
for i = 1:4
    add_line(sub_name, ['Int_Fy_relax_', num2str(i), '/1'], ...
        ['Mux_AllStates/', num2str(mux_idx)]);
    mux_idx = mux_idx + 1;
end

add_line(sub_name, 'Mux_AllStates/1', 'States_Out/1');

% 导数输出: 直接从 Core_Dynamics 输出路由（无需再 Mux）
add_line(sub_name, 'Core_Dynamics/1', 'Derivs_Out/1');
add_line(sub_name, 'Core_Dynamics/2', 'Aux_Out/1');

fprintf('  Vehicle_Dynamics 子系统构建完成。\n');
fprintf('    29 个积分器 + 1 个核心 S-Function + 信号路由\n');
end

%% ===== 辅助函数: 创建核心动力学 S-Function 子系统块 =====
function create_core_dynamics_block(parent)
% 在 parent 下创建 Core_Dynamics 子系统
% 内部: 3个输入端口 + Level-2 S-Function + 2个输出端口
%   输入1: t (标量)
%   输入2: x (29维状态向量)
%   输入3: u_ctrl (11维控制向量)
%   输出1: dx (29维导数向量)

blk_path = [parent, '/Core_Dynamics'];
add_block('simulink/Ports & Subsystems/Subsystem', blk_path);
set_param(blk_path, 'Position', [180, 200, 320, 600]);
set_param(blk_path, 'BackgroundColor', 'cyan');
Simulink.SubSystem.deleteContents(blk_path);

% 输入端口
in_labels = {'t', 'x(29)', 'u(11)'};
in_dims = [1, 29, 11];
for i = 1:3
    in_blk = [blk_path, '/In', num2str(i)];
    add_block('simulink/Sources/In1', in_blk);
    if in_dims(i) > 1
        set_param(in_blk, 'PortDimensions', num2str(in_dims(i)));
    end
    set_param(in_blk, 'Position', [30, 40 + (i-1)*60, 60, 60 + (i-1)*60]);
end

% 文本标注（Simulink Annotation）
% 无法通过命令行添加，通过端口名足以识别

% S-Function 块
sfun_blk = [blk_path, '/VehicleDynamics_SFun'];
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', sfun_blk);
set_param(sfun_blk, 'FunctionName', 'core_dynamics_wrapper_sfun', ...
    'Parameters', 'p_14dof', ...
    'Position', [130, 30, 280, 230]);

% 输入 → S-Function
for i = 1:3
    add_line(blk_path, ['In', num2str(i), '/1'], ['VehicleDynamics_SFun/', num2str(i)]);
end

% 输出端口
out_blk = [blk_path, '/Out1'];
add_block('simulink/Sinks/Out1', out_blk);
set_param(out_blk, 'PortDimensions', '29', ...
    'Position', [350, 80, 380, 110]);
add_line(blk_path, 'VehicleDynamics_SFun/1', 'Out1/1');

out_blk = [blk_path, '/Out2'];
add_block('simulink/Sinks/Out1', out_blk);
set_param(out_blk, 'PortDimensions', '52', ...
    'Position', [350, 150, 380, 180]);
add_line(blk_path, 'VehicleDynamics_SFun/2', 'Out2/1');
end
