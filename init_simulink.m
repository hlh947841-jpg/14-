%% init_simulink.m — Simulink 模型初始化脚本
% 在打开 vehicle_14dof_4wis_4wid.slx 之前运行此脚本
% 或直接运行此脚本（会自动加载参数、路径和模型）
%
% 使用方法:
%   >> init_simulink
%
% 功能:
%   1. 添加所有子目录到 MATLAB 路径
%   2. 加载车辆参数
%   3. 生成/更新核心动力学 S-Function
%   4. 打开 Simulink 模型

clear; clc;

%% 获取项目根目录
root_dir = fileparts(mfilename('fullpath'));
if isempty(root_dir)
    root_dir = pwd;
end

%% 添加路径（仅添加必要子目录，避免 genpath 引入 .slx 文件冲突）
addpath(root_dir);
addpath(fullfile(root_dir, 'config'));
addpath(fullfile(root_dir, 'dynamics'));
addpath(fullfile(root_dir, 'control'));
addpath(fullfile(root_dir, 'simulation'));
addpath(fullfile(root_dir, 'simulink'));

fprintf('路径已添加。\n');

%% 加载车辆参数
p = vehicle_params();
fprintf('车辆参数已加载。\n');

%% 生成核心动力学 S-Function
% 必须先运行此步骤，否则 Simulink 报告 "S-Function 不存在"
fprintf('生成 S-Function...\n');
try
    create_sfun_subsystem(fullfile(root_dir, 'dynamics'), ...
        'core_dynamics_wrapper', 3, 1, [1 29 11], [29]);
    fprintf('  S-Function 已生成: core_dynamics_wrapper_sfun.m\n');
catch ME
    fprintf('  S-Function 生成失败: %s\n', ME.message);
    fprintf('  尝试打开已有模型...\n');
end

%% 检查模型文件是否存在
mdl_path = fullfile(root_dir, 'vehicle_14dof_4wis_4wid.slx');
if ~exist(mdl_path, 'file')
    fprintf('模型文件不存在，正在构建...\n');
    build_model(root_dir, p);
end

%% 打开模型
if bdIsLoaded('vehicle_14dof_4wis_4wid')
    close_system('vehicle_14dof_4wis_4wid', 0);
end
open_system(mdl_path);

%% 将参数写入模型工作区
mdl_wks = get_param('vehicle_14dof_4wis_4wid', 'ModelWorkspace');
assignin(mdl_wks, 'p_14dof', p);

fprintf('========================================\n');
fprintf('  Simulink 模型已就绪\n');
fprintf('  模型: vehicle_14dof_4wis_4wid.slx\n');
fprintf('  点击 Run 按钮即可运行仿真\n');
fprintf('========================================\n');
