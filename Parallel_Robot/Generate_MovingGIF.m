%关节空间内时间规划 
% 梯形速度规划 + 生成GIF（d1,d2,d3）

% 起点
d0 = [d1; d2; d3];

% 终点
dT = [D(132,1); D(132,2); D(132,3)];

vmax = 0.8;     % 最大速度
amax = 3.0;     % 最大加速度
dt = 0.02;      % 帧间隔
delay = dt;     % GIF 每帧延时

s = dT - d0;

% 计算每根杆的总时长，取最大值作为统一时长
[~,~,~,T1] = trap_traj_disp_signed(s(1), vmax, amax, 0);
[~,~,~,T2] = trap_traj_disp_signed(s(2), vmax, amax, 0);
[~,~,~,T3] = trap_traj_disp_signed(s(3), vmax, amax, 0);
T = max([T1 T2 T3]);

t = 0:dt:T;

% 生成三根杆的位移轨迹
[x1,~,~,~] = trap_traj_disp_signed(s(1), vmax, amax, t);
[x2,~,~,~] = trap_traj_disp_signed(s(2), vmax, amax, t);
[x3,~,~,~] = trap_traj_disp_signed(s(3), vmax, amax, t);

d1_traj = d0(1) + x1;
d2_traj = d0(2) + x2;
d3_traj = d0(3) + x3;

% 逐帧 FK，保存位姿序列
T_seq = nan(4,4,numel(t));
x0 = zeros(1,12);  % FK 初值

for k = 1:numel(t)
    [x0, T_limb1_sol] = FK_3SPR(S1,S2,S3, d1_traj(k), d2_traj(k), d3_traj(k), T_01, x0);
    T_seq(:,:,k) = T_limb1_sol;
end

% 输出 GIF
make_robot_gif(T_seq, p, T_12, T_13, 'robot.gif', delay);

% 单独绘制动平台中心轨迹
C_world = plot_platform_center_trajectory(T_seq, p, 'mean');
fprintf('最后一帧动平台中心点坐标 [x y z] = [%.6f  %.6f  %.6f]\n', C_world(end,1), C_world(end,2), C_world(end,3));