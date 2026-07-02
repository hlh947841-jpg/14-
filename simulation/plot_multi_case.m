function plot_multi_case(varargin)
%% plot_multi_case.m — 多工况对比可视化
% 将多个仿真工况的关键指标放在同一图中对比

cases = varargin;
n_case = numel(cases);
default_names = {'工况1:平路转向','工况2:纵坡直线','工况3:横坡直线', ...
                 '工况4:百公里巡航','工况5:蟹行模式','工况6:小半径转向', ...
                 '工况7:连续变坡转向','工况8:长下坡回馈制动','工况9:山路综合'};
case_names = cell(1, n_case);
for k = 1:n_case
    if k <= numel(default_names)
        case_names{k} = default_names{k};
    else
        case_names{k} = sprintf('工况%d', k);
    end
end
colors = lines(max(n_case, 1));
line_styles = {'-','--','-.',':'};

%% 图1: 纵向速度对比
figure('Name', '多工况对比_纵向速度', 'Position', [100, 100, 900, 500]);
for k = 1:n_case
    plot(cases{k}.t, cases{k}.vx*3.6, 'Color', colors(k, :), ...
        'LineStyle', line_styles{mod(k-1, numel(line_styles)) + 1}, 'LineWidth', 1.5); hold on;
end
xlabel('时间 t (s)'); ylabel('v_x (km/h)'); grid on;
legend(case_names, 'Location', 'best');
title('多工况纵向速度对比');

%% 图2: 车身姿态对比
figure('Name', '多工况对比_车身姿态', 'Position', [100, 100, 1000, 500]);
subplot(1,3,1);
for k = 1:n_case
    plot(cases{k}.t, cases{k}.phi*180/pi, 'Color', colors(k, :), ...
        'LineStyle', line_styles{mod(k-1, numel(line_styles)) + 1}, 'LineWidth', 1.5); hold on;
end
xlabel('t (s)'); ylabel('\phi (deg)'); grid on;
y_range = max(abs(ylim));  if y_range < 0.1, ylim([-0.1, 0.1]); end
legend(case_names, 'Location', 'best'); title('侧倾角');

subplot(1,3,2);
for k = 1:n_case
    plot(cases{k}.t, cases{k}.theta*180/pi, 'Color', colors(k, :), ...
        'LineStyle', line_styles{mod(k-1, numel(line_styles)) + 1}, 'LineWidth', 1.5); hold on;
end
xlabel('t (s)'); ylabel('\theta (deg)'); grid on;
y_range = max(abs(ylim));  if y_range < 0.1, ylim([-0.1, 0.1]); end
legend(case_names, 'Location', 'best'); title('俯仰角');

subplot(1,3,3);
for k = 1:n_case
    plot(cases{k}.t, cases{k}.psi_dot*180/pi, 'Color', colors(k, :), ...
        'LineStyle', line_styles{mod(k-1, numel(line_styles)) + 1}, 'LineWidth', 1.5); hold on;
end
xlabel('t (s)'); ylabel('d\psi/dt (deg/s)'); grid on;
y_range = max(abs(ylim));  if y_range < 0.1, ylim([-0.1, 0.1]); end
legend(case_names, 'Location', 'best'); title('横摆角速度');

%% 图3: 轮胎力对比 (前左轮FL)
figure('Name', '多工况对比_轮胎力', 'Position', [100, 100, 900, 500]);
subplot(1,2,1);
for k = 1:n_case
    plot(cases{k}.t, cases{k}.Fx(:,1), 'Color', colors(k, :), ...
        'LineStyle', line_styles{mod(k-1, numel(line_styles)) + 1}, 'LineWidth', 1.5); hold on;
end
xlabel('t (s)'); ylabel('F_x (N)'); grid on;
legend(case_names, 'Location', 'best'); title('纵向力 F_x (FL)');

subplot(1,2,2);
for k = 1:n_case
    plot(cases{k}.t, cases{k}.Fy(:,1), 'Color', colors(k, :), ...
        'LineStyle', line_styles{mod(k-1, numel(line_styles)) + 1}, 'LineWidth', 1.5); hold on;
end
xlabel('t (s)'); ylabel('F_y (N)'); grid on;
legend(case_names, 'Location', 'best'); title('侧向力 F_y (FL)');

fprintf('  已绘制3个多工况对比图窗（%d工况，纵向速度/车身姿态/轮胎力）\n', n_case);
end
