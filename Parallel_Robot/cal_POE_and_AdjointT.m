function [T_limb1_init, Ad_all_1, T_limb2_init, Ad_all_2, T_limb3_init, Ad_all_3]=cal_POE_and_AdjointT(S1,S2,S3,a1,b1,c1,T_01)
%计算初始时支链的POE以及伴随变换矩阵
[T_limb1_init, Ad_all_1]=branch_forward_kinematics(S1,a1,T_01);%初始时刻末端位姿
[T_limb2_init, Ad_all_2]=branch_forward_kinematics(S2,b1,T_01);%初始时刻末端位姿
[T_limb3_init, Ad_all_3]=branch_forward_kinematics(S3,c1,T_01);%初始时刻末端位姿
end