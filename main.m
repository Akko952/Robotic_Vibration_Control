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
%定义.初始值
x(1)=5;
%x(2)是一个时变信号，待求初始值
x(3)=3;
x(4)=5;
%Q1是一个时变信号，为广义坐标，但可以有初始值，定义为：
x(5)=60*pi/-180;
%定义D初始值
x(19)=10;
% 根据此可以迭代算出初始条件下，
% 满足几何约束的初始情况值？
% 迭代求解满足几何约束的 L2, Q2, s, Q3

[L2,Q2,s,Q3] = TheIterativeMethodForSolvingTheRoots(x);

% 将结果写回 x，供后续运动学计算
L1=x(1);L3=x(3);L4=x(4);Q1=x(5);D=x(19);
x(2)  = L2;
x(8)  = Q2;
x(11) = Q3;
x(14) = s;



    % % 验证结果是否正确 (代回方程)
check_val_1 = L1*sin(Q1) - L2*sin(Q2);
check_val_2 = D+L1*cos(Q1) - L2*cos(Q2);
check_val_3= L4*cos(Q3) - L3*sin(Q2)-s;
check_val_4 = L4*sin(Q3) - L3*cos(Q2);
fprintf('验证 f1 (应接近0): %.4e\n', check_val_1);
fprintf('验证 f2 (应接近0): %.4e\n', check_val_2);
fprintf('验证 f3 (应接近0): %.4e\n', check_val_3);
fprintf('验证 f4 (应接近0): %.4e\n', check_val_4);

GenerateSimplifiedMechanism(x);
%做出图像，验证结果
%完成初值求解
%开始运动学迭代
%设置初始角速度dQ1
dQ1=5*pi/-180;
ddQ1=5*pi/-180;
x(6)=dQ1;


% %调用simulink
% out= sim('Kinematic.slx', 'StopTime', '10');
% 
% %再绘图并验证
% Correct_new_data;

%调用simulink
out= sim('Dynamic.slx', 'StopTime', '10');

%再绘图并验证
Correct_new_data;