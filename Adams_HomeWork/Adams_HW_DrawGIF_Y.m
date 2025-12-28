%% Simulink数据转GIF脚本 (完全沿用例程几何逻辑)
filename = 'Adams_HW_Mechanism_Animation_y.gif'; % 输出文件名

% 1. 准备数据 (针对 simDataY，N*17 的矩阵)
data = out.simDataY; 
t = out.tout; % 时间向量

% 2. 创建画布
hFig = figure('Color', 'w', 'Name', '机构仿真动画');
% 设置固定坐标轴范围（根据你的机构尺寸调整）
axis_range = [-5, 5, -5, 5]; 

% 3. 循环绘图
count = 1;
step = 1000; % 步长：建议设为 1-10 之间。例程中的 1000 适合超高采样频率，通常设 5-10 画面较丝滑

for i = 1 : step : size(data, 1)
    % 提取当前时刻的 17 维向量 x
    current_x = data(i, :);
    
    % --- 按照例程逻辑进行解算 ---
    clf(hFig); % 清空当前画布
    hold on; grid on; axis equal;
    axis(axis_range); % 固定坐标轴
    
    % 变量索引映射 (根据你的 simDataY 定义)
    L1 = current_x(1); L2 = current_x(2); L3 = current_x(3); L4 = current_x(4);
    Q1 = current_x(5); Q3 = current_x(7); % 注意：在 simDataY 中 Q3 是第 7 位
    
    % 几何逻辑 (完全等同于例程)
    O_R1 = [0, 0];
    O_R4 = [L4, 0];
    P_R2 = O_R1 + [L1*cos(Q1), L1*sin(Q1)];
    P_R3 = O_R4 + [L3*cos(Q3), L3*sin(Q3)];

    % 绘制杆件 (样式、线宽、颜色完全遵循例程)
    plot([O_R1(1), O_R4(1)], [O_R1(2), O_R4(2)], 'k--', 'LineWidth', 2); % 机架
    plot([O_R1(1), P_R2(1)], [O_R1(2), P_R2(2)], 'LineWidth', 5, 'Color', [0 0.45 0.74]); % 曲柄
    plot([P_R2(1), P_R3(1)], [P_R2(2), P_R3(2)], 'LineWidth', 3, 'Color', [0.85 0.33 0.1]); % 连杆
    plot([O_R4(1), P_R3(1)], [O_R4(2), P_R3(2)], 'LineWidth', 3, 'Color', [0.47 0.67 0.19]); % 摇杆
    
    % 绘制关节 (大小、颜色遵循例程)
    scatter([O_R1(1), P_R2(1), P_R3(1), O_R4(1)], [O_R1(2), P_R2(2), P_R3(2), O_R4(2)], 60, 'k', 'filled');
    
    % 标题及刷新
    title(['Time: ', num2str(t(i), '%.2f'), 's, Q1 = ', num2str(rad2deg(Q1), '%.1f'), '°']);
    drawnow; 

    % --- 捕获并保存 GIF (完全遵循例程) ---
    frame = getframe(hFig);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    if count == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
    count = count + 1;
end

fprintf('GIF 动画已生成: %s\n', filename);
