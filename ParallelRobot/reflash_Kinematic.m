function frame = reflash_Kinematic(T_des, p,T_12,T_13)
    
    % 计算并更新新的铰点位置
    S1(1).ri= transform_Point_vector(T_des, p.b1);
    S1(1).s_hat =rotate_vector(T_des,[0;1;0])/norm(rotate_vector(T_des,[0;1;0]));
    S1(1).nu =-vector_to_skew_symmetric(S1(1).s_hat)*S1(1).ri ;
    S1(1).xi=[S1(1).s_hat;...
        S1(1).nu];%旋转副的旋量
    S1(2).s_hat = (S1(1).ri-p.s1)/norm(S1(1).ri-p.s1);
    S1(2).ri = p.s1;
    S1(2).nu = S1(2).s_hat;
    S1(2).xi=[0;0;0;S1(2).nu];%移动副的旋量
    %球铰
    S1(3).i = 3;
    S1(3).s_hat =S1(2).nu ;
    S1(3).ri =p.s1;
    S1(3).nu =-vector_to_skew_symmetric(S1(3).s_hat)*S1(3).ri ;%Z轴，沿着移动副
    S1(3).xi=[S1(3).s_hat;...
        S1(3).nu];%旋转副的旋量

    [s_hat_s14, s_hat_s15]=find_orthogonal_vectors(S1(3).s_hat);

    S1(4).i = 4; S1(4).s_hat = s_hat_s14; S1(4).ri =p.s1;
    S1(4).nu =-vector_to_skew_symmetric(S1(4).s_hat)*S1(4).ri ;
    S1(4).xi=[S1(4).s_hat;...
        S1(4).nu];%旋转副的旋量


    S1(5).i = 5; S1(5).s_hat = s_hat_s15; S1(5).ri =p.s1;
    S1(5).nu =-vector_to_skew_symmetric(S1(5).s_hat)*S1(5).ri ;
    S1(5).xi=[S1(5).s_hat;...
        S1(5).nu];%旋转副的旋量
    %% 

    S2(1).ri = transform_Point_vector(T_des, p.b2);
S2(1).s_hat =rotate_vector(T_des*T_12,[0;1;0])/norm(rotate_vector(T_des*T_12,[0;1;0]));
S2(1).nu =-vector_to_skew_symmetric(S2(1).s_hat)*S2(1).ri ;
S2(1).xi=[S2(1).s_hat;...
    S2(1).nu];%旋转副的旋量

%移动副
S2(2).i = 2; 
S2(2).s_hat = (S2(1).ri-p.s2)/norm(S2(1).ri-p.s2);
S2(2).ri = p.s2;
S2(2).nu = S2(2).s_hat;
S2(2).xi=[0;0;0;S2(2).nu];%移动副的旋量

%球铰
S2(3).i = 3; 
S2(3).s_hat =S2(2).nu ; 
S2(3).ri =p.s2;
S2(3).nu =-vector_to_skew_symmetric(S2(3).s_hat)*S2(3).ri ;%Z轴，沿着移动副
S2(3).xi=[S2(3).s_hat;...
    S2(3).nu];%旋转副的旋量
[s_hat_S24, s_hat_S25]=find_orthogonal_vectors(S2(3).s_hat);

S2(4).i = 4; S2(4).s_hat = s_hat_S24; S2(4).ri =p.s2;
S2(4).nu =-vector_to_skew_symmetric(S2(4).s_hat)*S2(4).ri ;
S2(4).xi=[S2(4).s_hat;...
    S2(4).nu];%旋转副的旋量
S2(5).i = 5; S2(5).s_hat = s_hat_S25; S2(5).ri =p.s2;
S2(5).nu =-vector_to_skew_symmetric(S2(5).s_hat)*S2(5).ri ;
S2(5).xi=[S2(5).s_hat;...
    S2(5).nu];%旋转副的旋量
    %% 

    S3(1).ri = transform_Point_vector(T_des, p.b3);
S3(1).s_hat =rotate_vector(T_des*T_13,[0;1;0])/norm(rotate_vector(T_des*T_13,[0;1;0]));
S3(1).nu =-vector_to_skew_symmetric(S3(1).s_hat)*S3(1).ri ;
S3(1).xi=[S3(1).s_hat;...
    S3(1).nu];%旋转副的旋量
%移动副
S3(2).i = 2; 
S3(2).s_hat = (S3(1).ri-p.s3)/norm(S3(1).ri-p.s3);
S3(2).ri = p.s3;
S3(2).nu = S3(2).s_hat;
S3(2).xi=[0;0;0;S3(2).nu];%移动副的旋量

%球铰
S3(3).i = 3; 
S3(3).s_hat =S3(2).nu ; 
S3(3).ri =p.s3;
S3(3).nu =-vector_to_skew_symmetric(S3(3).s_hat)*S3(3).ri ;%Z轴，沿着移动副
S3(3).xi=[S3(3).s_hat;...
    S3(3).nu];%旋转副的旋量
[s_hat_S34, s_hat_S35]=find_orthogonal_vectors(S3(3).s_hat);

S3(4).i = 4; S3(4).s_hat = s_hat_S34; S3(4).ri =p.s3;
S3(4).nu =-vector_to_skew_symmetric(S3(4).s_hat)*S3(4).ri ;
S3(4).xi=[S3(4).s_hat;...
    S3(4).nu];%旋转副的旋量

S3(5).i = 5; S3(5).s_hat = s_hat_S35; S3(5).ri =p.s3;
S3(5).nu =-vector_to_skew_symmetric(S3(5).s_hat)*S3(5).ri ;
S3(5).xi=[S3(5).s_hat;...
    S3(5).nu];%旋转副的旋量
    % --- 3. 静平台点 (始终不变) ---
    % p.s1;
    % p.s2;
    % p.s3;

    % --- 4. 绘图 ---
    hold off; % 清空上一帧
    %做出初始示意图的帧
frame1 = plot_screws_and_links(S1, false);
frame2 = plot_screws_and_links(S2, false);
frame3 = plot_screws_and_links(S3, false);

S_leg(1).ri = S1(1).ri;
S_leg(2).ri = S2(1).ri;
S_leg(3).ri = S3(1).ri;

frame0 = plot_p_r_links(p, S_leg, false);
    
    % --- 装饰 ---
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(sprintf('Pose: T_{des}'));
    view(3); % 设置视角
    axis([-10 10 -10 10 -10 5]); % 根据您的机器人尺寸调整坐标轴范围

     drawnow;
     frame = getframe(gcf);
end