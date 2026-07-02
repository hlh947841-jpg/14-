%% run_sim.m — 14-DOF 四轮独立驱动/转向 底盘动力学模型 一键运行
% 流程: 参数加载 → ODE仿真 → 可视化 → 保存图片和结果
%
% 使用方法: 在 MATLAB 中运行此脚本
%   >> run_sim
%
% 模型特征:
%   - 14 DOF: 车身6 + 车轮垂向4 + 车轮旋转4
%   - 29 状态: 含8个轮胎松弛力状态
%   - 驱动: 四轮独立轮毂电机
%   - 转向: 四轮独立转向
%   - 悬架: 非线性模型D (三次弹簧 + 平方阻尼)
%   - 轮胎: Magic Formula + 摩擦椭圆 + 松弛长度

clear; clc; close all;

%% ===== 0. 路径设置 =====
root_dir = fileparts(mfilename('fullpath'));
addpath(root_dir);
addpath(fullfile(root_dir, 'config'));
addpath(fullfile(root_dir, 'control'));
addpath(fullfile(root_dir, 'dynamics'));
addpath(fullfile(root_dir, 'simulation'));
addpath(fullfile(root_dir, 'simulink'));
fprintf('========================================\n');
fprintf('  14-DOF 四轮独立驱动/转向 底盘动力学模型\n');
fprintf('========================================\n');
fprintf('工作目录: %s\n', root_dir);

%% ===== 1. 加载参数 =====
fprintf('\n========== 第1步: 加载车辆参数 ==========\n');
p = vehicle_params();

%% ===== 1.5 控制模式配置 =====
% p.ctrl.mode 默认=0（禁用），各工况单独设置:
%   mode=0: 原始四轮均摊（向后兼容）
%   mode=1: 载荷比例分配 (T_i ∝ Fz_i)
%   mode=2: 载荷比例 + ASR/TCS驱动防滑
%   mode=3: 载荷比例 + ASR/TCS + TVC/DYC横摆力矩控制
fprintf('\n========== 第1.5步: 控制模式配置 ==========\n');
fprintf('  控制模式可切换: 0=禁用 1=载荷 2=+ASR 3=+TVC/DYC\n');

%% ===== 2. 单工况 ODE 仿真验证 =====
fprintf('\n========== 第2步: ODE单工况验证 ==========\n');

% 工况定义: 平路匀速转向
% u = [delta(1:4), T_m(1:4), beta_brk, i_h, i_c]
delta_deg = [2, 2, 1, 1];        % 四轮转角 [deg] (前大后小)
u_case1 = [deg2rad(delta_deg), ...    % delta (1:4)
           80, 80, 50, 50, ...        % T_m (1:4) [N·m]
           0, ...                     % beta_brk
           0, 0];                     % i_h, i_c (平路)

fprintf('工况1 (平路转向, vx0=30 km/h):\n');
fprintf('  转向角: FL=%.1f° FR=%.1f° RL=%.1f° RR=%.1f°\n', delta_deg);
fprintf('  电机扭矩: FL=%.0f FR=%.0f RL=%.0f RR=%.0f N·m\n', u_case1(5:8));

T_sim = 5;
p.ctrl.mode = 3;  % TVC/DYC全功能（平路转向，验证横摆力矩控制）
R1 = simulate(p, u_case1, [], T_sim);

%% ===== 3. 多工况批量仿真 =====
fprintf('\n========== 第3步: 多工况对比仿真 ==========\n');

% 工况2: 纯纵向坡度 (i_c = 0.1, 约 5.7°) — 直线行驶，无转向
% 物理上纯纵坡不应有侧倾/横摆，必须delta=0保持直线
u_case2 = [zeros(1,4), ...           % delta = 0 (纯直线)
           80, 80, 50, 50, ...       % T_m
           0, ...                    % beta_brk
           0, 0.1];                  % i_h=0, i_c=0.1
fprintf('\n工况2 (纵向坡度 i_c=0.1, 直线行驶):\n');
p.ctrl.mode = 2;  % 载荷比例 + ASR（纵坡直线，验证载荷分配和防滑）
R2 = simulate(p, u_case2, [], T_sim);

% 工况3: 横向坡度 (i_h = 0.1) — 直线行驶
% 横坡上重力侧向分量使车辆下坡漂移，零转向可观察纯横坡效应
u_case3 = [zeros(1,4), ...           % delta = 0 (观察横坡漂移)
           80, 80, 50, 50, ...       % T_m
           0, ...                    % beta_brk
           0.1, 0];                  % i_h=0.1, i_c=0
fprintf('\n工况3 (横向坡度 i_h=0.1, 直线行驶):\n');
p.ctrl.mode = 2;  % 载荷比例 + ASR（横坡直线，验证横向载荷转移下的分配）
R3 = simulate(p, u_case3, [], T_sim);

% 工况4: 百公里匀速巡航（恒速100km/h，验证速度控制器稳态性能）
% 前馈阻力矩 + P项维持设定车速，基础扭矩设为零
u_case4 = [zeros(1,4), ...           % delta = 0
           0, 0, 0, 0, ...           % T_m = 0 (由速度P+前馈控制器全权驱动)
           0, ...                    % beta_brk
           0, 0];                    % 平路
p.vx0_save = p.vx0;
p.vx0 = 100 / 3.6;  % 目标/初始车速 100km/h（恒速巡航，非零起步）
fprintf('\n工况4 (百公里恒速巡航, 目标100km/h):\n');
p.ctrl.mode = 2;  % 载荷比例 + ASR
R4 = simulate(p, u_case4, [], 10);
p.vx0 = p.vx0_save;  % 恢复目标车速

%% ===== 4. 四轮独立转向能力展示 =====
fprintf('\n========== 第4步: 四轮独立转向展示 ==========\n');

% 工况5: 蟹行模式 (四轮同向偏转)
u_case5 = [deg2rad([5, 5, 5, 5]), ...  % 同向偏转
           80, 80, 80, 80, ...
           0, 0, 0];
fprintf('工况5 (蟹行模式, 四轮同向偏转5°):\n');
p.ctrl.mode = 1;  % 仅载荷比例分配（蟹行无横摆需求，无需ASR/TVC）
R5 = simulate(p, u_case5, [], 5);

% 工况6: 小半径转向模式 (前后轮反向偏转，减小转弯半径)
% 注意: 前后反向偏转大幅缩小转弯半径，转向角不宜过大以避免翻车
u_case6 = [deg2rad([3, 3, -3, -3]), ...  % 前后反向3°（≈0.3g侧向加速度）
           60, 60, 60, 60, ...
           0, 0, 0];
fprintf('工况6 (小半径转向, 前后反向偏转±3°):\n');
p.ctrl.mode = 3;  % TVC/DYC全功能（急转，大横摆需求，验证横摆力矩控制）
R6 = simulate(p, u_case6, [], 5);

%% ===== 4.5 丘陵山地复合工况（时变坡度） =====
fprintf('\n========== 第4.5步: 丘陵山地复合工况 ==========\n');

% 工况7: 连续正弦坡度 + 转向（模拟山路起伏弯道）
% 纵向坡度: 15%幅值正弦，周期4s（模拟连续上下坡）
% 横向坡度: 5%幅值正弦，周期6s（模拟路面横坡变化）
% 同时前轮转向2°
T_sim_hill = 20;
u_case7 = @(t) [deg2rad([2, 2, 0, 0]), ...   % delta: 前轮转向
                 80, 80, 50, 50, ...           % T_m
                 0, ...                        % beta_brk
                 0.05 * sin(2*pi*t/6), ...     % i_h: 横坡正弦 5%
                 0.15 * sin(2*pi*t/4)];        % i_c: 纵坡正弦 15%
fprintf('工况7 (连续正弦坡度+转向, T=%ds):\n', T_sim_hill);
fprintf('  纵坡: 15%%幅值正弦(周期4s), 横坡: 5%%幅值正弦(周期6s)\n');
fprintf('  前轮转向2°, ctrl.mode=3\n');
p.ctrl.mode = 3;
R7 = simulate(p, u_case7, [], T_sim_hill);

% 工况8: 长下坡回馈制动（模拟山区长下坡5km）
% 坡度: 10%恒定下坡 (i_c = -0.10, 约-5.7°)
% 目标车速: 40 km/h（下坡控速）
% 电机扭矩: 零（完全靠回馈制动+液压制动控速）
p.vx0_save2 = p.vx0;
p.vx0 = 40 / 3.6;  % 下坡目标车速 40km/h
u_case8 = [zeros(1,4), ...        % delta = 0 (直线)
           0, 0, 0, 0, ...        % T_m = 0 (无驱动，靠回馈制动)
           0, ...                 % beta_brk = 0 (观察自动制动协调)
           0, -0.10];             % i_h=0, i_c=-0.10 (10%下坡)
fprintf('\n工况8 (长下坡回馈制动, 10%%坡度, 目标40km/h):\n');
p.ctrl.mode = 2;  % 载荷比例 + ASR（下坡需要防滑）
R8 = simulate(p, u_case8, [], 30);  % 30秒模拟长下坡
p.vx0 = p.vx0_save2;  % 恢复

% 工况9: 山路综合工况（上坡→平路→下坡→弯道，模拟真实山路）
% 分段目标通过C4连续权重平滑过渡，避免阶跃输入引入非真实冲击峰值
T_sim_mountain = 30;
u_case9 = @(t) deal_u9(t);
fprintf('\n工况9 (山路综合, %ds: 上坡→平路→下坡→弯道):\n', T_sim_mountain);
p.ctrl.mode = 3;
R9 = simulate(p, u_case9, [], T_sim_mountain);

%% ===== 5. 可视化 =====
fprintf('\n========== 第5步: 结果可视化 ==========\n');

% 各工况详细曲线
fprintf('绘制工况1详细曲线...\n');
plot_results(R1, '工况1_平路转向');
fprintf('绘制工况2详细曲线...\n');
plot_results(R2, '工况2_纵坡直线');
fprintf('绘制工况3详细曲线...\n');
plot_results(R3, '工况3_横坡直线');
fprintf('绘制工况4详细曲线...\n');
plot_results(R4, '工况4_直线加速');
fprintf('绘制工况5详细曲线...\n');
plot_results(R5, '工况5_蟹行模式');
fprintf('绘制工况6详细曲线...\n');
plot_results(R6, '工况6_小半径转向');
fprintf('绘制工况7详细曲线...\n');
plot_results(R7, '工况7_连续变坡转向');
fprintf('绘制工况8详细曲线...\n');
plot_results(R8, '工况8_长下坡回馈制动');
fprintf('绘制工况9详细曲线...\n');
plot_results(R9, '工况9_山路综合');

% 多工况对比（选取代表性工况）
fprintf('绘制多工况对比...\n');
plot_multi_case(R1, R2, R3, R4, R5, R6, R7, R8, R9);

%% ===== 5.5 保存图片 =====
fig_dir = fullfile(root_dir, 'results_figures');
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end
fig_handles = findobj('Type', 'figure');
fprintf('保存图片...共%d个图窗\n', length(fig_handles));
for k = 1:length(fig_handles)
    fig_name = get(fig_handles(k), 'Name');
    if isempty(fig_name)
        fig_name = sprintf('fig_%02d', k);
    end
    % 清理文件名中的非法字符
    fig_name = regexprep(fig_name, '[\\/:*?"<>|]', '_');
    saveas(fig_handles(k), fullfile(fig_dir, [fig_name, '.png']));
end
fprintf('图片已保存至: %s\n', fig_dir);

%% ===== 6. Simulink 模型构建与仿真 =====
fprintf('\n========== 第6步: Simulink仿真 ==========\n');
try
    % 构建模型
    build_model(root_dir, p);

    % 运行 Simulink 仿真
    model_name = 'vehicle_14dof_4wis_4wid';
    if bdIsLoaded(model_name)
        % 设置工况参数
        set_param([model_name '/delta_FL'], 'Value', num2str(u_case1(1)));
        set_param([model_name '/delta_FR'], 'Value', num2str(u_case1(2)));
        set_param([model_name '/delta_RL'], 'Value', num2str(u_case1(3)));
        set_param([model_name '/delta_RR'], 'Value', num2str(u_case1(4)));
        set_param([model_name '/T_m_FL'], 'Value', num2str(u_case1(5)));
        set_param([model_name '/T_m_FR'], 'Value', num2str(u_case1(6)));
        set_param([model_name '/T_m_RL'], 'Value', num2str(u_case1(7)));
        set_param([model_name '/T_m_RR'], 'Value', num2str(u_case1(8)));
        set_param([model_name '/beta_brk'], 'Value', num2str(u_case1(9)));
        set_param([model_name '/i_h'], 'Value', num2str(u_case1(10)));
        set_param([model_name '/i_c'], 'Value', num2str(u_case1(11)));

        % 设置模型工作区参数
        mdl_wks = get_param(model_name, 'ModelWorkspace');
        assignin(mdl_wks, 'p_14dof', p);

        fprintf('运行 Simulink 仿真...\n');
        simOut = sim(model_name, 'StopTime', num2str(T_sim), ...
                     'Solver', 'ode15s', 'MaxStep', '0.005', ...
                     'RelTol', '1e-4', 'AbsTol', '1e-6');

        fprintf('Simulink 仿真完成。\n');
        set_param(model_name, 'Dirty', 'off');
        close_system(model_name, 0);
    end
catch ME
    fprintf('Simulink 仿真跳过: %s\n', ME.message);
    fprintf('（ODE仿真结果仍然有效，请手动检查Simulink模型）\n');
end

%% ===== 7. 保存结果 =====
fprintf('\n========== 第7步: 保存结果 ==========\n');
results.R1 = R1; results.R2 = R2; results.R3 = R3;
results.R4 = R4; results.R5 = R5; results.R6 = R6;
results.R7 = R7; results.R8 = R8; results.R9 = R9;

save(fullfile(root_dir, 'simulation_results.mat'), 'p', 'results');
fprintf('结果已保存至 simulation_results.mat\n');

%% ===== 总结 =====
fprintf('\n========================================\n');
fprintf('  14-DOF 四轮独立驱动/转向 仿真完成\n');
fprintf('========================================\n');
fprintf('  模型自由度:  14 (车身6 + 车轮垂向4 + 车轮旋转4)\n');
fprintf('  状态变量数:  29 (含8个轮胎松弛力状态)\n');
fprintf('  驱动形式:    四轮独立轮毂电机\n');
fprintf('  转向形式:    四轮独立转向\n');
fprintf('  悬架模型:    非线性模型D (三次弹簧 + 平方阻尼)\n');
fprintf('  轮胎模型:    Magic Formula + 摩擦椭圆 + 松弛长度\n');
fprintf('  仿真工况数:  9 (含丘陵山地复合工况)\n');
fprintf('========================================\n');

%% ===== 局部函数: 工况9输入生成 =====
function u = deal_u9(t)
% 山路综合工况: 上坡→平路→下坡→弯道
%   0~8s:   12%上坡直线
%   8~14s:  平路直线
%   14~22s: 10%下坡直线
%   22~30s: 平路 + 前轮转向3°
    trans = 1.0;
    w_flat_1 = smooth_transition(t, 8, trans);
    w_down = smooth_transition(t, 14, trans) - smooth_transition(t, 22, trans);
    w_turn = smooth_transition(t, 22, trans);

    i_c = 0.12 * (1 - w_flat_1) - 0.10 * w_down;
    delta = deg2rad([3, 3, 0, 0]) * w_turn;
    T_m = [80, 80, 50, 50] * (1 - w_down);
    u = [delta, T_m, 0, 0, i_c];
end

function s = smooth_transition(t, t0, T)
% C4连续九阶过渡，t0前为0，t0+T后为1。
    tau = min(max((t - t0) / T, 0), 1);
    s = tau^5 * (126 - 420*tau + 540*tau^2 - 315*tau^3 + 70*tau^4);
end
