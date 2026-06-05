%% 麦克纳姆轮小车画8字运动仿真
% 基于正逆运动学模型，底盘不旋转，纯平移
% 坐标约定：x轴向右，y轴向前
% 轮组布局：O型（RF-B型, LF-A型, LB-B型, RB-A型）
clear; clc; close all;

%% ====================== 车辆参数 ======================
D_wheel = 74.78e-3;                % 车轮直径 (m)
R_wheel = D_wheel / 2;             % 车轮半径 (m)
L_half  = 173.0e-3 / 2;           % 半轴距 (m)
W_half  = 194.2e-3 / 2;           % 半轮距 (m)
d = L_half + W_half;               % 组合参数 (m)

% 动画需要的整车尺寸
track = 2 * W_half;                % 轮距 (m)
wheelbase = 2 * L_half;            % 轴距 (m)

fprintf('===== 小车参数 =====\n');
fprintf('R_wheel=%.4f m  L=%.4f m  W=%.4f m  d=%.4f m\n\n', R_wheel, L_half, W_half, d);

%% ====================== 8字轨迹（两个相切圆，向量化）======================
R_circ = 0.25;                      % 圆半径 (m)，8字总宽=4R=1m
T_cycle = 10;                       % 单周期 (s)
T_total = 20;                       % 总时长 (s)
dt = 0.02;
t = 0:dt:T_total;
omega = 4*pi / T_cycle;             % 参数角速度

tc = mod(t, T_cycle);               % 周期内时间
half = T_cycle / 2;
left = tc < half;                   % 左半周期掩码
right = ~left;

% 预分配数组（保证时间顺序正确）
X = zeros(size(t));
Y = zeros(size(t));
Vx = zeros(size(t));
Vy = zeros(size(t));

% 左圆（逆时针，圆心 -R_circ,0）
th = omega * tc(left);
X(left) = -R_circ + R_circ*cos(th);
Y(left) = R_circ*sin(th);
Vx(left) = -R_circ*omega*sin(th);
Vy(left) = R_circ*omega*cos(th);

% 右圆（逆时针，圆心 R_circ,0，速度连续）
th = omega * (tc(right) - half);
X(right) = R_circ - R_circ*cos(th);
Y(right) = R_circ*sin(th);
Vx(right) = R_circ*omega*sin(th);
Vy(right) = R_circ*omega*cos(th);

% 车身不旋转 → 体坐标系速度 = 世界坐标系速度
vx_body = Vx; vy_body = Vy;
omega_z = 0;

%% ====================== 逆运动学：四轮转速 ======================
wheel_omega = [
    (vx_body - vy_body - d*omega_z) / R_wheel;   % RF
    (vx_body + vy_body + d*omega_z) / R_wheel;   % LF
    (vx_body + vy_body - d*omega_z) / R_wheel;   % LB
    (vx_body - vy_body + d*omega_z) / R_wheel];  % RB
wheel_v = wheel_omega * R_wheel;

%% ====================== 正运动学验证 ======================
vx_chk = (R_wheel/4) * sum(wheel_omega([1 2 3 4],:));
vy_chk = (R_wheel/4) * (-wheel_omega(1,:) + wheel_omega(2,:) ...
                         + wheel_omega(3,:) - wheel_omega(4,:));
wz_chk = (R_wheel/(4*d)) * (-wheel_omega(1,:) + wheel_omega(2,:) ...
                             - wheel_omega(3,:) + wheel_omega(4,:));

fprintf('===== 正逆运动学验证 =====\n');
fprintf('Δv_x max: %.2e  Δv_y max: %.2e  Δω_z max: %.2e\n\n', ...
    max(abs(vx_chk-vx_body)), max(abs(vy_chk-vy_body)), max(abs(wz_chk-omega_z)));

%% ====================== 绘图 ======================
figure('Position', [50 50 1400 900]);

% 辅助：参考圆数据
th_circ = linspace(0, 2*pi, 100);
circ_x = R_circ * cos(th_circ);
circ_y = R_circ * sin(th_circ);
circle_centers = [-R_circ, 0; R_circ, 0];

subplot(2,3,1); hold on; grid on; axis equal;
plot(X, Y, 'b-', 'LineWidth', 1.5);
plot(X(1), Y(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(X(end), Y(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
for i = 1:2
    plot(circle_centers(i,1)+circ_x, circle_centers(i,2)+circ_y, ...
         '--', 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
end
plot(circle_centers(:,1), circle_centers(:,2), 'k+', 'MarkerSize', 8);
xlabel('X (m)'); ylabel('Y (m)');
title('8字轨迹（两个相切圆）');
legend('轨迹', '起点', '终点', '参考圆', '圆心', 'Location', 'best');

subplot(2,3,2);
plot(t, zeros(size(t)), 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('航向角 (deg)');
title('航向角（底盘不旋转）'); grid on; ylim([-5, 5]);

subplot(2,3,3);
plot(t, vx_body, 'r-', 'LineWidth', 1.5); hold on;
plot(t, vy_body, 'b-', 'LineWidth', 1.5);
plot(t, zeros(size(t)), 'g-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('速度 (m/s)');
title('底盘速度'); legend('v_x', 'v_y', '\omega_z', 'Location', 'best'); grid on;

subplot(2,3,4);
plot(t, wheel_omega'*60/(2*pi), 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('转速 (rpm)');
title('四轮转速'); legend('RF', 'LF', 'LB', 'RB', 'Location', 'best'); grid on;

subplot(2,3,5);
plot(t, wheel_v', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('线速度 (m/s)');
title('四轮线速度'); legend('RF', 'LF', 'LB', 'RB', 'Location', 'best'); grid on;

subplot(2,3,6);
plot(t, vx_chk-vx_body, 'r-', 'LineWidth', 1); hold on;
plot(t, vy_chk-vy_body, 'b-', 'LineWidth', 1);
plot(t, wz_chk-omega_z, 'g-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差');
title('正逆运动学验证'); legend('Δv_x', 'Δv_y', 'Δω_z', 'Location', 'best'); grid on;

sgtitle('麦克纳姆轮小车画8字运动仿真（底盘不旋转）', 'FontSize', 14, 'FontWeight', 'bold');

%% ====================== 动画 ======================
figure('Position', [100 100 1000 800]);
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('麦克纳姆轮小车画8字仿真动画');

% 参考圆 + 轨迹
for i = 1:2
    plot(circle_centers(i,1)+circ_x, circle_centers(i,2)+circ_y, ...
         '--', 'Color', [0.85 0.85 0.85], 'LineWidth', 1);
end
plot(X, Y, '--', 'LineWidth', 1, 'Color', [0.7 0.7 1]);

% 车身多边形
car_shape = [track/2, wheelbase/2; -track/2, wheelbase/2;
             -track/2, -wheelbase/2; track/2, -wheelbase/2; track/2, wheelbase/2]';
car_h = fill(nan(1,5), nan(1,5), 'c', 'FaceAlpha', 0.3, 'EdgeColor', 'b', 'LineWidth', 2);

% 轮子
wheel_colors = {'r', 'b', 'g', 'm'};
wheel_h = gobjects(4,1);
wp = [W_half, L_half; -W_half, L_half; -W_half, -L_half; W_half, -L_half];
r_vis = D_wheel/2;
for i = 1:4
    wheel_h(i) = rectangle('Position', [0 0 0 0], 'Curvature', [1 1], ...
                           'FaceColor', wheel_colors{i}, 'EdgeColor', 'k', 'LineWidth', 1.5);
end

trail_h = plot(nan, nan, 'r-', 'LineWidth', 2);
center_h = plot(nan, nan, 'k+', 'MarkerSize', 12, 'LineWidth', 2);
vel_h = quiver(nan, nan, nan, nan, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.3);
txt_h = text(0, 0, '', 'FontSize', 10, 'BackgroundColor', 'w');

skip = max(1, round(0.04/dt));
trail_len = 100;
N = length(t);
idx = 1:skip:N;

% 动画循环
for k = idx
    xc = X(k); yc = Y(k);

    % 车身平移
    set(car_h, 'XData', xc + car_shape(1,:), 'YData', yc + car_shape(2,:));

    % 轮子
    for i = 1:4
        set(wheel_h(i), 'Position', [xc+wp(i,1)-r_vis, yc+wp(i,2)-r_vis, 2*r_vis, 2*r_vis]);
    end

    set(center_h, 'XData', xc, 'YData', yc);
    set(trail_h, 'XData', X(max(1,k-trail_len):k), 'YData', Y(max(1,k-trail_len):k));
    set(vel_h, 'XData', xc, 'YData', yc, 'UData', Vx(k), 'VData', Vy(k));

    w_rpm = wheel_omega(:,k) * 60 / (2*pi);
    set(txt_h, 'Position', [xc-0.5, yc+0.35], ...
        'String', sprintf('RF:%.0f LF:%.0f LB:%.0f RB:%.0f rpm  t=%.1fs', w_rpm(1), w_rpm(2), w_rpm(3), w_rpm(4), t(k)));

    xlim([xc-0.8, xc+0.8]); ylim([yc-0.6, yc+0.6]);
    drawnow; pause(0.01);
end

fprintf('仿真完成！\n');
fprintf('半径 R_circ=%.2f m  切线速度 v≈%.2f m/s\n', R_circ, R_circ*omega);