function [Jacobian_spctial_platform,R0]=cal_platform_jacobian_by_forceScrew(S1_in,S2_in,S3_in,p,T_01,L,d1,d2,d3)
%计算动平台的运动学雅可比矩阵，通过力雅可比的方法
%设定力的作用点相对于固定坐标系的位置
q1 = p.s1+(L+d1)*S1_in(4).s_hat;
p1 = p.s2+(L+d2)*S2_in(4).s_hat; 
r1 = p.s3+(L+d3)*S3_in(4).s_hat; %真实的空间位置
%计算初始时刻的雅可比，利用力雅可比
%如果去除所有的外力，则动平台受到的力为三个Limb传递的力
%对于limb来说，这个力以S副为起点，指向R副，也就是P的方向
%对于Limb来说，力矩的臂为 the vector from the {s}-frame origin to the point of application of the force
F_s1=Force_Jacobian(S1_in, q1,T_01);
F_s2=Force_Jacobian(S2_in, p1,T_01);
F_s3=Force_Jacobian(S3_in, r1,T_01);
%组装动平台的力雅可比
Jacpbian_F_platform=[F_s1, F_s2, F_s3];
Jacpbian_F_platform_inv=(Jacpbian_F_platform)';%平台雅可比的伪逆即为力雅可比的转置;
Jacobian_spctial_platform=pinv(Jacpbian_F_platform_inv);%平台运动学雅可比
R0=rank(Jacobian_spctial_platform);%检查自由度，得到为3
end