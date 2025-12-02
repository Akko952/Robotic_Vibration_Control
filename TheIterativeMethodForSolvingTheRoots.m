function [L2,Q2,s,Q3]=TheIterativeMethodForSolvingTheRoots(x)

L1=x(1);%L2=x(2);
L3=x(3);L4=x(4);
Q1=x(5);%dQ1=x(6);ddQ1=x(7);
%Q2=x(8);%dQ2=x(9);ddQ2=x(10);
%Q3=x(11);%dQ3=x(12);ddQ3=x(13);
%s=x(14);%ds=x(15);dds=x(16);
%dL2=x(17);ddL2=x(18);
D=x(19);

%猜测Q2、Q3、L2、s的初始值
Q2_guess=5*pi/180;
Q3_guess=20*pi/180;
L2_guess=15;
s_guess=4;
Inition=[L2_guess,Q2_guess,s_guess,Q3_guess];

%方程组解析式;
%方程组：
%L1*SinQ1-L2*SinQ2=0
%D+L1*CosQ1-L2*CosQ2=0
%L4*CosQ3-L3*SinQ2=s
%L4*SinQ3-L3*CosQ2=0

% GeoMetric_F_1=@(Q1,L2,Q2)(L1)*sin(Q1)-(L2)*sin(Q2);
% GeoMetric_F_2=@(Q1,L2,Q2)D+(L1)*cos(Q1)-(L2)*cos(Q2);
% GeoMetric_F_3=@(Q3,s,Q2)(L4)*cos(Q3)-(L3)*sin(Q2)-s;
% GeoMetric_F_4=@(Q3,Q2)(L4)*sin(Q3)-(L3)*cos(Q2);
% GeometricConstraintsMatrix=[GeoMetric_F_1;GeoMetric_F_2;GeoMetric_F_3;GeoMetric_F_4];

%检查敛散性,迭代参数设置
max_iter = 100;
tolerance=1e-6;


for iter = 1:max_iter

    % 提取当前迭代变量
    L2_curr = Inition(1);
    Q2_curr = Inition(2);
    s_curr=Inition(3);
    Q3_curr=Inition(4);


%建立数值解预测初始值矩阵
GeometricConstraintsMatrixIntial=[
(L1)*sin(Q1)-(L2_curr)*sin(Q2_curr);
D+(L1)*cos(Q1)-(L2_curr)*cos(Q2_curr);
(L4)*cos(Q3_curr)-(L3)*sin(Q2_curr)-s_curr;
(L4)*sin(Q3_curr)-(L3)*cos(Q2_curr)
];

%建立数值雅可比矩阵
% Jacob=[(L1)*cos(Q1),-(L2)*cos(Q2),0,0;
%         -(L1)*sin(Q1),(L2)*sin(Q2),0,0;
%         0,-(L3)*cos(Q2),-1,-(L4)*sin(Q3);
%         0,(L3)*sin(Q2),0,(L4)*cos(Q3)
% ];%Q1、Q2、s、Q3

Jacob=[-sin(Q2_curr),-(L2_curr)*cos(Q2_curr),0,0;
        sin(Q2_curr),(L2_curr)*sin(Q2_curr),0,0;
        0,-(L3)*cos(Q2_curr),-1,-(L4)*sin(Q3_curr);
        0,(L3)*sin(Q2_curr),0,(L4)*cos(Q3_curr)
];%L2、Q2、s、Q3


if norm(GeometricConstraintsMatrixIntial) < tolerance
    fprintf('在第 %d 次迭代收敛。\n', iter);
    L2 = Inition(1);
    Q2= Inition(2);
    s=Inition(3);
    Q3=Inition(4);
    % % 输出结果
    fprintf('结果: L2 = %.4f m\n , Q2=%.4f rad\n,s = %.4f\n,Q3= %.4f\n ', ...
         L2, Q2, s,Q3);

    % % 验证结果是否正确 (代回方程)
    % check_val = L1*sin(theta1) - L2*sin(theta2);
    % fprintf('验证 f2 (应接近0): %.4e\n', check_val);
    return;
end

delta = -Jacob \ GeometricConstraintsMatrixIntial;
Inition = Inition + delta;
end

warning('达到最大迭代次数，未收敛。请尝试调整初始猜测值。');
L2 = Inition(1);
Q2= Inition(2);
s=Inition(3);
Q3=Inition(4);
end