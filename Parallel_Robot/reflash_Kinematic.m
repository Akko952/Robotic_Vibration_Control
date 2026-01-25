 function [S1_new, S2_new, S3_new, frame] =reflash_Kinematic(T_des, p,T_12,T_13, doPlot)
        % ===== 为连续 S 副保存上一帧正交轴 =====
persistent s14_prev s15_prev
persistent s24_prev s25_prev
persistent s34_prev s35_prev
persistent fig_robot
persistent center_traj
     if nargin < 5 || isempty(doPlot)
         doPlot = true;
     end
     frame = [];
        %%
    % 用当前几何关系初始化（第一帧）
if isempty(s15_prev)
    % --- 第 1 条支链 ---
    s1_hat = (transform_Point_vector(T_des, p.b1) - p.s1);
    s1_hat = s1_hat / norm(s1_hat);

    s13_hat = s1_hat;% 也就是移动副方向

    tmp = rotate_vector(T_des,[0;1;0]);
    s14_prev = tmp - dot(tmp,s13_hat)*s13_hat;
    s14_prev = s14_prev/norm(s14_prev);

    % --- 第 2 条支链 ---
    s2_hat = (transform_Point_vector(T_des, p.b2) - p.s2);
    s2_hat = s2_hat / norm(s2_hat);

    s23_hat = s2_hat;% 也就是移动副方向

    tmp = rotate_vector(T_des*T_12,[0;1;0]);
    s24_prev = tmp - dot(tmp,s23_hat)*s23_hat;
    s24_prev = s24_prev/norm(s24_prev);


    % --- 第 3 条支链 ---
    s3_hat = (transform_Point_vector(T_des, p.b3) - p.s3);
    s3_hat = s3_hat / norm(s3_hat);

    s33_hat = s3_hat;% 也就是移动副方向
    
    tmp = rotate_vector(T_des*T_13,[0;1;0]);
    s34_prev = tmp - dot(tmp,s33_hat)*s33_hat;
    s34_prev = s34_prev/norm(s34_prev);

    [s15_prev] = find_orthogonal_vectors_inition(s13_hat,s14_prev);
    [ s25_prev] = find_orthogonal_vectors_inition(s23_hat,s24_prev);
    [ s35_prev] = find_orthogonal_vectors_inition(s33_hat,s34_prev);
end
    % 计算并更新新的铰点位置
    S1_new(1).ri= transform_Point_vector(T_des, p.b1);
    S1_new(1).s_hat =rotate_vector(T_des,[0;1;0])/norm(rotate_vector(T_des,[0;1;0]));
    S1_new(1).nu =-vector_to_skew_symmetric(S1_new(1).s_hat)*S1_new(1).ri ;
    S1_new(1).xi=[S1_new(1).s_hat;...
        S1_new(1).nu];%旋转副的旋量
    S1_new(2).s_hat = (S1_new(1).ri-p.s1)/norm(S1_new(1).ri-p.s1);
    S1_new(2).ri = p.s1;
    S1_new(2).nu = S1_new(2).s_hat;
    S1_new(2).xi=[0;0;0;S1_new(2).nu];%移动副的旋量
    %球铰
    S1_new(3).i = 3;
    S1_new(3).s_hat =S1_new(2).nu ;
    S1_new(3).ri =p.s1;
    S1_new(3).nu =-vector_to_skew_symmetric(S1_new(3).s_hat)*S1_new(3).ri ;%Z轴，沿着移动副
    S1_new(3).xi=[S1_new(3).s_hat;...
        S1_new(3).nu];%旋转副的旋量

    S1_new(4).i = 4; S1_new(4).s_hat = s14_prev;S1_new(4).ri =p.s1;
    S1_new(4).nu =-vector_to_skew_symmetric(S1_new(4).s_hat)*S1_new(4).ri ;
    S1_new(4).xi=[S1_new(4).s_hat;...
        S1_new(4).nu];%旋转副的旋量

s_hat_s15 = find_orthogonal_vectors_flash( ...
              S1_new(3).s_hat, ...
              s14_prev, ...
              s15_prev );
s15_prev = s_hat_s15;

    S1_new(5).i = 5; S1_new(5).s_hat = s_hat_s15; S1_new(5).ri =p.s1;
    S1_new(5).nu =-vector_to_skew_symmetric(S1_new(5).s_hat)*S1_new(5).ri ;
    S1_new(5).xi=[S1_new(5).s_hat;...
        S1_new(5).nu];%旋转副的旋量
    %% 

    S2_new(1).ri = transform_Point_vector(T_des, p.b2);
    S2_new(1).s_hat =rotate_vector(T_des*T_12,[0;1;0])/norm(rotate_vector(T_des*T_12,[0;1;0]));
    S2_new(1).nu =-vector_to_skew_symmetric(S2_new(1).s_hat)*S2_new(1).ri ;
    S2_new(1).xi=[S2_new(1).s_hat;...
    S2_new(1).nu];%旋转副的旋量

%移动副
S2_new(2).i = 2; 
S2_new(2).s_hat = (S2_new(1).ri-p.s2)/norm(S2_new(1).ri-p.s2);
S2_new(2).ri = p.s2;
S2_new(2).nu = S2_new(2).s_hat;
S2_new(2).xi=[0;0;0;S2_new(2).nu];%移动副的旋量

%球铰
S2_new(3).i = 3; 
S2_new(3).s_hat =S2_new(2).nu ; 
S2_new(3).ri =p.s2;
S2_new(3).nu =-vector_to_skew_symmetric(S2_new(3).s_hat)*S2_new(3).ri ;%Z轴，沿着移动副
S2_new(3).xi=[S2_new(3).s_hat;...
    S2_new(3).nu];%旋转副的旋量


S2_new(4).i = 4; S2_new(4).s_hat = s24_prev; S2_new(4).ri =p.s2;
S2_new(4).nu =-vector_to_skew_symmetric(S2_new(4).s_hat)*S2_new(4).ri ;
S2_new(4).xi=[S2_new(4).s_hat;...
    S2_new(4).nu];%旋转副的旋量

    s_hat_s25 = find_orthogonal_vectors_flash( ...
                S2_new(3).s_hat, ...
                s24_prev, ...
                s25_prev );
s25_prev = s_hat_s25;
S2_new(5).i = 5; S2_new(5).s_hat = s_hat_s25; S2_new(5).ri =p.s2;
S2_new(5).nu =-vector_to_skew_symmetric(S2_new(5).s_hat)*S2_new(5).ri ;
S2_new(5).xi=[S2_new(5).s_hat;...
    S2_new(5).nu];%旋转副的旋量
    %% 

    S3_new(1).ri = transform_Point_vector(T_des, p.b3);
S3_new(1).s_hat =rotate_vector(T_des*T_13,[0;1;0])/norm(rotate_vector(T_des*T_13,[0;1;0]));
S3_new(1).nu =-vector_to_skew_symmetric(S3_new(1).s_hat)*S3_new(1).ri ;
S3_new(1).xi=[S3_new(1).s_hat;...
    S3_new(1).nu];%旋转副的旋量
%移动副
S3_new(2).i = 2; 
S3_new(2).s_hat = (S3_new(1).ri-p.s3)/norm(S3_new(1).ri-p.s3);
S3_new(2).ri = p.s3;
S3_new(2).nu = S3_new(2).s_hat;
S3_new(2).xi=[0;0;0;S3_new(2).nu];%移动副的旋量

%球铰
S3_new(3).i = 3; 
S3_new(3).s_hat =S3_new(2).nu ; 
S3_new(3).ri =p.s3;
S3_new(3).nu =-vector_to_skew_symmetric(S3_new(3).s_hat)*S3_new(3).ri ;%Z轴，沿着移动副
S3_new(3).xi=[S3_new(3).s_hat;...
    S3_new(3).nu];%旋转副的旋量

S3_new(4).i = 4; S3_new(4).s_hat = s34_prev; S3_new(4).ri =p.s3;
S3_new(4).nu =-vector_to_skew_symmetric(S3_new(4).s_hat)*S3_new(4).ri ;
S3_new(4).xi=[S3_new(4).s_hat;...
    S3_new(4).nu];%旋转副的旋量

s_hat_s35 = find_orthogonal_vectors_flash( ...
              S3_new(3).s_hat, ...
              s34_prev, ...
              s35_prev );
s35_prev = s_hat_s35;

S3_new(5).i = 5; S3_new(5).s_hat = s_hat_s35; S3_new(5).ri =p.s3;
S3_new(5).nu =-vector_to_skew_symmetric(S3_new(5).s_hat)*S3_new(5).ri ;
S3_new(5).xi=[S3_new(5).s_hat;...
    S3_new(5).nu];%旋转副的旋量
    %%
    % --- 3. 静平台点 (始终不变) ---
    % p.s1;
    % p.s2;
    % p.s3;
    %%
    % --- 4. 绘图（可选） ---
    if doPlot
        % 动平台“中心”轨迹：默认取动平台三个连接点的几何中心
        center_local = (p.b1 + p.b2 + p.b3) / 3;
        center_world = transform_Point_vector(T_des, center_local);
        if isempty(center_traj)
            center_traj = center_world;
        else
            center_traj(:, end+1) = center_world;
        end

        % 使用独立图窗绘制，避免占用其它脚本（如蒙特卡洛）的 figure
        if isempty(fig_robot) || ~isvalid(fig_robot)
            fig_robot = figure('Name','3SPR Robot Pose', 'NumberTitle','off');
        else
            figure(fig_robot);
        end

        clf(fig_robot);
        hold off; % 清空上一帧

        % 做出示意图
        plot_screws_and_links(S1_new, false);
        plot_screws_and_links(S2_new, false);
        plot_screws_and_links(S3_new, false);

        S_leg(1).ri = S1_new(1).ri;
        S_leg(2).ri = S2_new(1).ri;
        S_leg(3).ri = S3_new(1).ri;
        plot_p_r_links(p, S_leg, false);

        % --- 叠加中心轨迹 ---
        hold on;
        plot3(center_traj(1, :), center_traj(2, :), center_traj(3, :), 'r-', 'LineWidth', 2);
        plot3(center_world(1), center_world(2), center_world(3), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
        hold off;

        % --- 装饰 ---
        grid on; axis equal;
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title(sprintf('Pose: T_{des}'));
        view(3); % 设置视角
        axis([-10 10 -10 10 -10 5]); % 根据您的机器人尺寸调整坐标轴范围

        drawnow;
        frame = getframe(fig_robot);
    end
 end