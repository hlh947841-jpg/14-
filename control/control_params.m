function ctrl = control_params()
%% control_params.m — 扭矩分配控制参数
% 所有控制参数集中管理，独立于 vehicle_params.m
%
% 控制模式 (ctrl.mode):
%   mode=0: 禁用控制模块 — 完全保留原始四轮均摊行为（向后兼容）
%   mode=1: 载荷比例分配 — T_i ∝ Fz_mf(i)，总扭矩守恒
%   mode=2: 载荷比例 + ASR/TCS — mode1 + 滑移率超限时sigmoid削减
%   mode=3: 载荷比例 + ASR/TCS + TVC/DYC — mode2 + 横摆角速度闭环PI + 左右差动
%
% 调用方式:
%   ctrl = control_params();
%   p.ctrl = ctrl;  % 挂载到车辆参数结构体

%% ===== 控制模式 =====
ctrl.mode = 0;              % 默认禁用（向后兼容），各工况单独配置

%% ===== 车速P+前馈控制器参数 =====
ctrl.Kp_speed = 1000;       % 车速P控制器增益 [N·m/(m/s)]
ctrl.Kp_brake = 0.05;       % 制动协调增益 [1/(m/s)]
ctrl.K_blend  = 5.0;        % Sigmoid过渡锐度（越大越接近硬切换）

%% ===== ASR/TCS 驱动防滑控制参数 =====
ctrl.kappa_thr   = 0.15;    % 滑移率阈值 [-]（超过此值开始削减扭矩）
ctrl.K_asr_blend = 10.0;    % ASR削减sigmoid过渡锐度

%% ===== TVC/DYC 横摆力矩控制参数 =====
ctrl.Kp_yaw      = 500;     % 横摆角速度PI控制器P增益 [N·m/(rad/s)]
ctrl.Ki_yaw      = 100;     % 横摆角速度PI控制器I增益 [N·m/rad]
ctrl.yaw_int_max = 200;     % 积分抗饱和上限 [N·m]
ctrl.tau_tvc     = 0.05;    % TVC力矩一阶低通滤波时间常数 [s]（0.05s避免ODE过刚）

%% ===== 横摆力矩分配参数 =====
ctrl.yaw_split_front = 0.5; % 前轴横摆力矩分担比例 [-]
% rear = 1 - front（自动计算）

fprintf('扭矩分配控制参数已加载。mode=%d (0=禁用,1=载荷比例,2=+ASR,3=+TVC/DYC)\n', ctrl.mode);
end
