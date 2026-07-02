function plot_results(R, case_name, varargin)
%% plot_results.m — 单工况仿真结果详细可视化
% 绘制车身运动、车轮状态、轮胎力、悬架力等时间历程
% Input:
%   R         — 仿真结果结构体
%   case_name — 工况名称（用于标题和文件名），如 '工况1_平路转向'

% 获取工况信息用于标题
if isfield(R.info, 'delta_deg')
    delta_str = sprintf('δ=[%.0f,%.0f,%.0f,%.0f]°', R.info.delta_deg);
    info_str = sprintf('%s — vx0=%.0fkm/h, %s', case_name, R.info.speed_kmh, delta_str);
elseif isfield(R.info, 'input_type') && strcmp(R.info.input_type, 'time-varying')
    info_str = sprintf('%s — vx0=%.0fkm/h, 时变坡度输入', case_name, R.info.speed_kmh);
else
    info_str = sprintf('%s — vx0=%.0fkm/h', case_name, R.info.speed_kmh);
end

%% 图1: 车身平动速度
figure('Name', [case_name '_车身平动速度'], 'Position', [50, 400, 900, 400]);
subplot(1,3,1);
plot(R.t, R.vx*3.6, 'b-', 'LineWidth', 1.5);
xlabel('时间 t (s)'); ylabel('v_x (km/h)'); grid on;
title(sprintf('纵向速度 (目标 %.0f km/h)', R.info.speed_kmh));

subplot(1,3,2);
plot(R.t, R.vy, 'r-', 'LineWidth', 1.5);
xlabel('时间 t (s)'); ylabel('v_y (m/s)'); grid on;
y_range = max(abs(ylim));  if y_range < 0.01, ylim([-0.01, 0.01]); end
title('横向速度');

subplot(1,3,3);
plot(R.t, R.vz, 'g-', 'LineWidth', 1.5);
xlabel('时间 t (s)'); ylabel('v_z (m/s)'); grid on;
y_range = max(abs(ylim));  if y_range < 0.01, ylim([-0.01, 0.01]); end
title('垂向速度');
sgtitle(['车身平动速度 — ', info_str]);

%% 图2: 车身姿态
figure('Name', [case_name '_车身姿态'], 'Position', [50, 50, 900, 400]);
subplot(1,3,1);
plot(R.t, R.phi*180/pi, 'b-', 'LineWidth', 1.5);
xlabel('时间 t (s)'); ylabel('\phi (deg)'); grid on;
y_range = max(abs(ylim));  if y_range < 0.1, ylim([-0.1, 0.1]); end
title('侧倾角');

subplot(1,3,2);
plot(R.t, R.theta*180/pi, 'r-', 'LineWidth', 1.5);
xlabel('时间 t (s)'); ylabel('\theta (deg)'); grid on;
y_range = max(abs(ylim));  if y_range < 0.1, ylim([-0.1, 0.1]); end
title('俯仰角');

subplot(1,3,3);
plot(R.t, R.psi_dot*180/pi, 'g-', 'LineWidth', 1.5);
xlabel('时间 t (s)'); ylabel('d\psi/dt (deg/s)'); grid on;
y_range = max(abs(ylim));  if y_range < 0.1, ylim([-0.1, 0.1]); end
title('横摆角速度');
sgtitle(['车身姿态 — ', info_str]);

%% 图3: 车轮垂向位移
figure('Name', [case_name '_车轮垂向位移'], 'Position', [100, 100, 700, 500]);
labels = {'FL','FR','RL','RR'};
colors = {'b','r','g','m'};
for i = 1:4
    subplot(2,2,i);
    plot(R.t, R.z_w(:,i)*1000, [colors{i} '-'], 'LineWidth', 1.5);
    xlabel('t (s)'); ylabel('z_w (mm)'); grid on;
    title(sprintf('车轮 %s', labels{i}));
end
sgtitle(['车轮垂向位移 — ', info_str]);

%% 图4: 车轮转速
figure('Name', [case_name '_车轮转速'], 'Position', [150, 150, 700, 500]);
for i = 1:4
    subplot(2,2,i);
    plot(R.t, R.omega_w(:,i), [colors{i} '-'], 'LineWidth', 1.5);
    xlabel('t (s)'); ylabel('\omega_w (rad/s)'); grid on;
    title(sprintf('车轮 %s', labels{i}));
end
sgtitle(['车轮转速 — ', info_str]);

%% 图5: 轮胎力
figure('Name', [case_name '_轮胎力'], 'Position', [200, 200, 900, 600]);
for i = 1:4
    subplot(2,2,i);
    plot(R.t, R.Fx(:,i), 'b-', 'LineWidth', 1); hold on;
    plot(R.t, R.Fy(:,i), 'r-', 'LineWidth', 1);
    xlabel('t (s)'); ylabel('Force (N)'); grid on;
    legend('F_x','F_y','Location','best');
    title(sprintf('轮胎力 %s', labels{i}));
end
sgtitle(['轮胎力 — ', info_str]);

%% 图6: 悬架力
figure('Name', [case_name '_悬架力'], 'Position', [250, 250, 700, 500]);
for i = 1:4
    subplot(2,2,i);
    plot(R.t, R.Fs(:,i), [colors{i} '-'], 'LineWidth', 1.5);
    xlabel('t (s)'); ylabel('F_s (N)'); grid on;
    title(sprintf('悬架 %s', labels{i}));
end
sgtitle(['悬架力 — ', info_str]);

%% 图7: 驱动力矩
figure('Name', [case_name '_驱动力矩'], 'Position', [300, 300, 700, 500]);
for i = 1:4
    subplot(2,2,i);
    plot(R.t, R.T_drive(:,i), [colors{i} '-'], 'LineWidth', 1.5); hold on;
    plot(R.t, R.T_brake(:,i), 'k--', 'LineWidth', 1);
    xlabel('t (s)'); ylabel('Torque (N·m)'); grid on;
    legend('T_{drive}','T_{brake}','Location','best');
    title(sprintf('驱动/制动 %s', labels{i}));
end
sgtitle(['驱动力矩 — ', info_str]);

fprintf('  已绘制7个图窗（车身平动速度/姿态/车轮位移/转速/轮胎力/悬架力/驱动力矩）\n');
end
