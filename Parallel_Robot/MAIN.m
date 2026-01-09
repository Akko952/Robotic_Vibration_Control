%%
% 系统初始化
clc; clear; close all;
Kinematic;%初始化运动学参数
    hold off; % 清空上一帧
    %做出初始示意图的帧
  plot_screws_and_links(S1, false);
  plot_screws_and_links(S2, false);
  plot_screws_and_links(S3, false);
  S_leg(1).ri = S1(1).ri;
  S_leg(2).ri = S2(1).ri;
  S_leg(3).ri = S3(1).ri;
  plot_p_r_links(p, S_leg, false);
    
    % --- 装饰 ---
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(sprintf('Pose: T_{des}'));
    view(3); % 设置视角
    axis([-10 10 -10 10 -10 5]); % 根据您的机器人尺寸调整坐标轴范围

    
%计算初始时支链的POE以及伴随变换矩阵
cal_POE_and_AdjointT;%debug完全
%求解出初始时刻的目标位姿
%通过虚位移法求解初始时刻的无外力下的力雅可比

 cal_platform_jacobian_by_forceScrew;%debug完全

%可以通过计算得到相关limb的空间雅可比
cal_limb_jacobian;%debug完全，方法更具有普遍性，解得初始状态下的雅可比


%%
% 求解合法的逆运动学
 %设置目标动平台的位姿,求主动关节的关节变量，然后再各个驱动器的关节变量，属于逆运动学内容
   % z: 目标高度
   % alpha: 绕X轴转角 (roll)
   % beta: 绕Y轴转角 (pitch)
   %三自由度EE平台的目标位姿
   Z=-5.1;
   alpha=deg2rad(0);%绕X轴转alpha度
   beta=deg2rad(0); %绕Y轴转beta度
   [d, T_p, residual] = IK_3SPR(Z, alpha, beta, p);
%  T_des=T*[1,0,0,0;...
%     0,1,0,0;...
%     0,0,1,-5;...
%     0,0,0,1];%目标位姿,由于选择表示位姿为固定角+平移矩阵的形式，因此这里能直接给出欧拉角下的齐次变换矩阵
%  d = inverse_kinematics(T_des, p);
% d1=d(1);d2=d(2);d3=d(3);
% %计算目标时的POE


%%
%刚体约束方程：di​=∥p+Rri​−si​∥，也就是说：
% ​∥p+Rr1​−s1​∥2=d1^2​
% ∥p+Rr2​−s2​∥2=d2^2​
% ∥p+Rr3​−s3​∥2=d3^2​​同时成立

% Phi_1=norm(p+R*r1 -s1 )^2 - d1^2==0;
% Phi_2=norm(p+R*r2 -s2 )^2 - d2^2==0;
% Phi_3=norm(p+R*r3 -s3 )^2 - d3^2==0;
% Phi=[Phi_1;Phi_2;Phi_3];
% dot_Phi_1=
% dot_Phi_2=
% dot_Phi_3=

%  [S1, S2, S3, frame] =reflash_Kinematic(T_des, p,T_12,T_13);
%%


