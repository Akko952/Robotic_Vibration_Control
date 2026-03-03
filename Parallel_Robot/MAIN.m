%%
% 系统初始化
clc; clear; close all;
Kinematic;%初始化运动学参数
% 计算初始时刻的雅可比矩阵
 L=5/(sqrt(3)/2);%初始时刻各个支链的长度
 d1=0;d2=0;d3=0;
 d0=[d1 d2 d3];
a1=[0; 0; 0;d0(1);  0];%支链1的初始关节变量
b1=[0;  0; 0;d0(2); 0];%支链2的初始关节变量
c1=[0; 0; 0;d0(3);  0]; %支链3的初始关节变量
%计算初始时支链的POE以及伴随变换矩阵
[T_limb1_init, Ad_all_1, T_limb2_init, Ad_all_2, T_limb3_init, Ad_all_3]=cal_POE_and_AdjointT(S1,S2,S3,a1,b1,c1,T_01);%debug完全
%求解出初始时刻的目标位姿
%通过虚位移法求解初始时刻的无外力下的力雅可比

 [Jacobian_spctial_platform,R0]=cal_platform_jacobian_by_forceScrew(S1,S2,S3,p,T_01,L,d1,d2,d3);%debug完全，方法稍微有局限性

%可以通过计算得到相关limb的空间雅可比
[H_p,H_a,Ja1,Ja2,Ja3,g,Jacobian_spctial_platform_by_limb,Jacobian_spctial_platform_active]=cal_limb_jacobian(S1,S2,S3,Ad_all_1,Ad_all_2,Ad_all_3);%debug完全，方法更具有普遍性，解得初始状态下的雅可比
load workspace_montecarlo_3SPR_20000.mat
%%
% FK的模板
% d1=D(777,1);d2=D(777,2);d3=D(777,3);%给定主动关节变量
% [vars_sol_double,T_limb1_sol, Ad_all_1_sol, T_limb2_sol, Ad_all_2_sol, T_limb3_sol, Ad_all_3_sol]=FK_3SPR(S1,S2,S3,d1,d2,d3,T_01);%求解目标位姿
%   [S1_new, S2_new, S3_new, frame] =reflash_Kinematic(T_limb1_sol, p,T_12,T_13);
  %% 
% MonteCarlo_Workspace_3SPR;

%蒙特卡洛法粗略绘制工作空间
% 已获得数据

%%
% 关节空间的轨迹规划
% Generate_MovingGIF;
%笛卡尔空间轨迹规划

% XZ 平面正弦轨迹规划
 Generate_CartesianSine_XZ;

%%
%常规MBD方法的计算方法

% 求解合法的逆运动学
 %设置目标动平台的位姿,求主动关节的关节变量，然后再各个驱动器的关节变量，属于逆运动学内容
   % z: 目标高度
   % alpha: 绕X轴转角 (roll)
   % beta: 绕Y轴转角 (pitch)
   %三自由度EE平台的目标位姿
  %  Z=-5.1;
  %  alpha=deg2rad(0);%绕X轴转alpha度
  %  beta=deg2rad(0); %绕Y轴转beta度
  %  [d, T_p, residual] = IK_3SPR(Z, alpha, beta, p);


%  T_des=T*[1,0,0,0;...
%     0,1,0,0;...
%     0,0,1,-5;...
%     0,0,0,1];%目标位姿,由于选择表示位姿为固定角+平移矩阵的形式，因此这里能直接给出欧拉角下的齐次变换矩阵
% %计算目标时的POE

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
%%
% 求解数值雅可比矩阵
d1=D(777,1);d2=D(777,2);d3=D(777,3);%给定主动关节变量
[vars_sol_double,T_limb1_sol, Ad_all_1_sol, T_limb2_sol, Ad_all_2_sol, T_limb3_sol, Ad_all_3_sol]=FK_3SPR(S1,S2,S3,d1,d2,d3,T_01);%求解目标位姿
[S1_new, S2_new, S3_new, frame] =reflash_Kinematic(T_limb1_sol, p,T_12,T_13);
[H_p_sol,H_a_sol,Ja1_sol,Ja2_sol,Ja3_sol,g_sol,Jacobian_spctial_platform_by_limb_sol,Jacobian_spctial_platform_active_sol]=cal_limb_jacobian(S1_new,S2_new,S3_new,Ad_all_1_sol,Ad_all_2_sol,Ad_all_3_sol);
[Jacobian_spctial_platform_sol,R0_sol]=cal_platform_jacobian_by_forceScrew(S1_new,S2_new,S3_new,p,T_01,L,d1,d2,d3);%计算目标位姿下的雅可比矩阵

%% 给定矩阵
T_des = [ ...
  0.9890   -0.0270    0.1451   -0.9778; ...
   -0.0270    0.9335    0.3576   -2.2483; ...
   -0.1451   -0.3576    0.9226   -5.5027; ...
     0         0         0    1.0000];

x0 = zeros(1,15); 
try
  opts = struct();
  opts.verbose = true;   % 打印每次迭代信息
  opts.maxIter = 80;     % 最大迭代次数
  % opts.lambda0 = 1e-3; % 初始阻尼
  % opts.fdEps   = 1e-6; % 有限差分步长

  [d_sol, x_sol, T1_sol, T2_sol, T3_sol, info] = IK_3SPR(T_des, S1, S2, S3, T_01, x0, opts);
  fprintf('IK 求解成功：d = [%.6f  %.6f  %.6f]\n', d_sol(1), d_sol(2), d_sol(3));
  fprintf('迭代信息：iter=%d, exitflag=%d, f=%.3e, ||r||=%.3e\n', info.iter, info.exitflag, info.f, info.norm_r);

  pos_err = norm(T1_sol(1:3,4) - T_des(1:3,4));
  R_rel = T_des(1:3,1:3).'*T1_sol(1:3,1:3);
  ang_err = acos(max(-1,min(1,(trace(R_rel)-1)/2)));
  fprintf('位姿误差：|dp|=%.3e, 角度误差=%.3e rad\n', pos_err, ang_err);
catch ME
    warning('IK_3SPR:Failed', 'IK 求解失败：%s', ME.message);
end
 %%
% Classify_Singularities_3SPR;%分类奇异位形
