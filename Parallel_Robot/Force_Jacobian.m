function F_s= Force_Jacobian(S_limb, q_limb,T_01)
%计算并联机器人的力雅可比矩阵
%输入：S 结构体数组，包含每条支链的旋量信息
%      q_limb 动平台力作用点的坐标
%      T_01 动平台零位位姿
%输出：F_s 力雅可比矩阵
%力的方向即为从s副指向r副的方向
n_limb=S_limb(4).s_hat; %力作用点的线速度部分
%力作用点的位置向量
%    q_limb= transform_Point_vector(T_01, q_limb);
%q_limb已经是固定坐标系下的描述
m_limb= cross(S_limb(3).ri,n_limb ); %力作用点的力矩旋量部分
F_s=[%力矩旋量
m_limb;
%力旋量
n_limb];
end