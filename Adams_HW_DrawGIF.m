%% Simulink数据转GIF脚本
filename = 'Adams_HW_Mechanism_Animation.gif'; % 输出文件名

% 1. 准备数据 (假设 simDataX 是 N*13 的矩阵)
% 如果你的数据在 out.simDataX 里，请执行：data = out.simDataX;
data = out.simDataX; 
t = out.tout; % 时间向量

% 2. 创建画布
hFig = figure('Color', 'w', 'Name', '机构仿真动画');
% 设置固定坐标轴范围（根据你的机构尺寸调整，防止画面跳动）
axis_range = [-50, 200, -50, 150]; 

% 3. 循环绘图
count = 1;
step = 1000; % 步长：每隔5帧取一帧，防止GIF太大。如果想更平滑，设为1

for i = 1 : step : size(data, 1)
    % 提取当前时刻的 x 向量
    current_x = data(i, :);
    
    % --- 调用绘图逻辑 (内部不新建figure) ---
    clf(hFig); % 清空当前画布
    hold on; grid on; axis equal;
    axis(axis_range); % 固定坐标轴
    
    % 逻辑解算
    L1 = current_x(1); L2 = current_x(2); L3 = current_x(3); L4 = current_x(4);
    Q1 = current_x(5); Q3 = current_x(11);
    O_R1 = [0, 0];
    O_R4 = [L4, 0];
    P_R2 = O_R1 + [L1*cos(Q1), L1*sin(Q1)];
    P_R3 = O_R4 + [L3*cos(Q3), L3*sin(Q3)];

    % 绘制杆件
    plot([O_R1(1), O_R4(1)], [O_R1(2), O_R4(2)], 'k--', 'LineWidth', 2); % 机架
    plot([O_R1(1), P_R2(1)], [O_R1(2), P_R2(2)], 'LineWidth', 5, 'Color', [0 0.45 0.74]); % 曲柄
    plot([P_R2(1), P_R3(1)], [P_R2(2), P_R3(2)], 'LineWidth', 3, 'Color', [0.85 0.33 0.1]); % 连杆
    plot([O_R4(1), P_R3(1)], [O_R4(2), P_R3(2)], 'LineWidth', 3, 'Color', [0.47 0.67 0.19]); % 摇杆
    
    % 绘制关节
    scatter([O_R1(1), P_R2(1), P_R3(1), O_R4(1)], [O_R1(2), P_R2(2), P_R3(2), O_R4(2)], 60, 'k', 'filled');
    
    title(['Time: ', num2str(t(i), '%.2f'), 's, Q1 = ', num2str(rad2deg(Q1), '%.1f'), '°']);
    drawnow; % 立即刷新图像

    % --- 捕获并保存 GIF ---
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