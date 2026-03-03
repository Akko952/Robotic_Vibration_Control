% 笛卡尔空间轨迹规划：XZ 平面正弦轨迹
% - 在 XZ 平面生成一条正弦曲线（Y 保持常数）
% - 逐帧构造目标位姿 T_des(k)
% - 用 IK_3SPR（牛顿下山法）求出每帧的 (d1,d2,d3)
% - 可选：输出 GIF、绘制轨迹
%
% 依赖：Kinematic.m 产生 S1,S2,S3,T_01,T_12,T_13,p
%       IK_3SPR.m（本工程内）
%       make_robot_gif.m

%% 1) 选一个可达的“参考位姿”作为姿态基准
% 推荐用工作空间样本，保证可达；否则退化用 T_01
if exist('T_ok','var') && ~isempty(T_ok)
    T_ref = T_ok(:,:,1);
else
    T_ref = T_01;
end
R_ref = T_ref(1:3,1:3);
p_ref = T_ref(1:3,4);

%% 2) 轨迹参数（你可以按需要改）
N = 120;                 % 轨迹点数（越大越平滑，但 IK 越慢）
A = 0.3;                 % Z 向正弦幅值
numPeriods = 1;          % 周期数
x_span = 2.0;            % X 向总长度（从 x_start 到 x_end）

x_start = p_ref(1) - x_span/2;
x_end   = p_ref(1) + x_span/2;
y_const = p_ref(2);      % 保持在 XZ 平面：Y 常数
z0      = p_ref(3);      % Z 基准偏置

x_list = linspace(x_start, x_end, N);
phase = 2*pi*numPeriods * (x_list - x_start) / (x_end - x_start);
z_list = z0 + A * sin(phase);

%% 3) 构造目标位姿序列 T_des_seq（姿态固定 = R_ref）
T_des_seq = repmat(eye(4), 1, 1, N);
for k = 1:N
    T_des_seq(:,:,k) = [R_ref, [x_list(k); y_const; z_list(k)]; 0 0 0 1];
end

%% 4) 逐帧 IK 求解：得到 d1/d2/d3 轨迹 & 实际位姿序列
D_traj = nan(3, N);
T_sol_seq = nan(4,4,N);

x0 = zeros(1,15); % 初值（后续会滚动更新）

opts = struct();
opts.verbose = false;    % 需要看迭代过程就改 true
opts.maxIter = 80;

failCount = 0;
for k = 1:N
    T_des = T_des_seq(:,:,k);
    try
        [d_sol, x_sol, T1_sol, ~, ~, info] = IK_3SPR(T_des, S1, S2, S3, T_01, x0, opts);
        D_traj(:,k) = d_sol;
        T_sol_seq(:,:,k) = T1_sol;
        x0 = x_sol; % 关键：用上一帧解做下一帧初值（更快更稳）

        if info.exitflag == 0
            % 没达到收敛阈值但返回了结果：记录一下
            failCount = failCount + 1;
        end
    catch ME
        warning('Generate_CartesianSine_XZ:IKFail', '第 %d 帧 IK 失败：%s', k, ME.message);
        failCount = failCount + 1;

        % 失败策略：用上一帧位姿顶上，避免序列断掉（也便于继续 GIF）
        if k > 1
            D_traj(:,k) = D_traj(:,k-1);
            T_sol_seq(:,:,k) = T_sol_seq(:,:,k-1);
        end
    end
end

fprintf('XZ 正弦轨迹规划完成：N=%d, failCount=%d\n', N, failCount);

%% 5) 绘制 XZ 平面轨迹
figure('Name','Cartesian Sine Trajectory (XZ plane)','NumberTitle','off');
plot(x_list, z_list, 'r-', 'LineWidth', 2);
grid on; axis equal;
xlabel('X'); ylabel('Z');
title('目标轨迹：XZ 平面正弦曲线 (Y 常数)');

%% 6) 可选：输出 GIF（用 IK 得到的实际位姿序列）
% 注意：如果 failCount 很多，说明这条轨迹对当前机构/姿态基准不可达，
%       建议减小 A、缩短 x_span、或换一个 T_ref。
makeGif = true;
if makeGif
    delay = 0.03;
    make_robot_gif(T_sol_seq, p, T_12, T_13, 'robot_cartesian_sine_xz.gif', delay);
end

%% 7) 可选：查看中心轨迹（用你之前写的函数）
% plot_platform_center_trajectory(T_sol_seq, p, 'mean');
