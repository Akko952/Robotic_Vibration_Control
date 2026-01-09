d1=0;d2=0;d3=0;
 d0=[d1 d2 d3];
a1=[0; d0(1); 0; 0; 0];%支链1的初始关节变量
b1=[0; d0(2); 0; 0; 0];%支链2的初始关节变量
c1=[0; d0(3); 0; 0; 0]; %支链3的初始关节变量
[T_limb1_init, Ad_all_1]=branch_forward_kinematics(S1,a1,T_01);%初始时刻末端位姿
[T_limb2_init, Ad_all_2]=branch_forward_kinematics(S2,b1,T_01);%初始时刻末端位姿
[T_limb3_init, Ad_all_3]=branch_forward_kinematics(S3,c1,T_01);%初始时刻末端位姿