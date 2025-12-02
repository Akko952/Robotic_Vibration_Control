clear;clc;
%定义输入变量
x = zeros(19,1);
%定义几何约束：
%方程组：
%L1*SinQ1-L2*SinQ2=0
%D+L1*CosQ1-L2*CosQ2=0
%L4*CosQ3-L3*SinQ2=s
%L4*SinQ3-L3*CosQ2=0

% GeoMetric_F_1=@(Q1,L2,Q2)(L1)*Sin(Q1)-(L2)*Sin(Q2);
% GeoMetric_F_2=@(Q1,L2,Q2)D+(L1)*Cos(Q1)-(L2)*Cos(Q2);
% GeoMetric_F_3=@(Q3,s,Q2)(L4)*Cos(Q3)-(L3)*Sin(Q2)-s;
% GeoMetric_F_4=@(Q3,Q2)(L4)*Sin(Q3)-(L3)*Cos(Q2);
% GeometricConstraintsMatrix=[GeoMetric_F_1;GeoMetric_F_2;GeoMetric_F_3;GeoMetric_F_4];
%定义初始值
x(1)=5;
%x(2)是一个时变信号，待求初始值
x(3)=3;
x(4)=5;
%Q1是一个时变信号，为广义坐标，但可以有初始值，定义为：
x(5)=10*pi/180;
%定义D初始值
x(19)=10;
% 根据此可以迭代算出初始条件下，
% 满足几何约束的初始情况值？
% 迭代求解满足几何约束的 L2, Q2, s, Q3
[L2,Q2,s,Q3] = TheIterativeMethodForSolvingTheRoots(x);

% 将结果写回 x，供后续运动学计算
x(2)  = L2;
x(8)  = Q2;
x(11) = Q3;
x(14) = s;

% MechanicalMechanics_Kinematics(x);