%% 数据提取与作图脚本
% 假设数据已从 Simulink 导出至 out 对象
t = out.tout;
dataX = out.simDataX;
dataY = out.simDataY;

% --- 1. 从 simDataX (13列) 提取 ---
% 定义：Q1=5, dQ1=6; Q2=8, dQ2=9; Q3=11, dQ3=12
Q1_X = dataX(:, 5);  dQ1_X = dataX(:, 6);
Q2_X = dataX(:, 8);  dQ2_X = dataX(:, 9);
Q3_X = dataX(:, 11); dQ3_X = dataX(:, 12);

% --- 2. 从 simDataY (17列) 提取 ---
% 定义：Q1=5, Q2=6, Q3=7; dQ1=15, dQ2=16, dQ3=17
Q1_Y = dataY(:, 5);  Q2_Y = dataY(:, 6);  Q3_Y = dataY(:, 7);
dQ1_Y = dataY(:, 15); dQ2_Y = dataY(:, 16); dQ3_Y = dataY(:, 17);

% --- 3. 绘图：位置 Q (角度) ---
figure('Name', '角度对比 Q1, Q2, Q3', 'Color', 'w');

subplot(3, 1, 1);
plot(t, rad2deg(Q1_X), 'b', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(Q1_Y), 'r--', 'LineWidth', 1);
ylabel('Q1 (deg)'); title('角度对比：实线(X) vs 虚线(Y)');
legend('DataX', 'DataY'); grid on;

subplot(3, 1, 2);
plot(t, rad2deg(Q2_X), 'b', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(Q2_Y), 'r--', 'LineWidth', 1);
ylabel('Q2 (deg)'); grid on;

subplot(3, 1, 3);
plot(t, rad2deg(Q3_X), 'b', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(Q3_Y), 'r--', 'LineWidth', 1);
ylabel('Q3 (deg)'); xlabel('Time (s)'); grid on;

% --- 4. 绘图：速度 dQ (角速度) ---
figure('Name', '角速度对比 dQ1, dQ2, dQ3', 'Color', 'w');

subplot(3, 1, 1);
plot(t, dQ1_X, 'b', 'LineWidth', 1.5); hold on;
plot(t, dQ1_Y, 'r--', 'LineWidth', 1);
ylabel('dQ1 (rad/s)'); title('角速度对比：实线(X) vs 虚线(Y)');
legend('DataX', 'DataY'); grid on;

subplot(3, 1, 2);
plot(t, dQ2_X, 'b', 'LineWidth', 1.5); hold on;
plot(t, dQ2_Y, 'r--', 'LineWidth', 1);
ylabel('dQ2 (rad/s)'); grid on;

subplot(3, 1, 3);
plot(t, dQ3_X, 'b', 'LineWidth', 1.5); hold on;
plot(t, dQ3_Y, 'r--', 'LineWidth', 1);
ylabel('dQ3 (rad/s)'); xlabel('Time (s)'); grid on;