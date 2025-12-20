clear;clc;close all;
%定义输入变量
x = zeros(13,1);
%定义几何约束：
%方程组：
% l_1 \cos \theta_1 + l_2 \cos \theta_2 - l_3 \cos \theta_3 - l_4 = 0 
% l_1 \sin \theta_1 + l_2 \sin \theta_2 - l_3 \sin \theta_3 = 0

%n=2;%约束方程个数

%定义.初始值
x(1)=1;
x(2)=2;
x(3)=2.5;
x(4)=3;
%Q1是一个时变信号，为广义坐标，但可以有初始值，定义为：
x(5)=60*pi/180;

[Q2,Q3] = Adams_Newton_Laersen(x);

% 将结果写回 x，供后续运动学计算
L1=x(1);L2=x(2);
L3=x(3);L4=x(4);
Q1=x(5);%dQ1=x(6);ddQ1=x(7);
%Q2=x(8);%dQ2=x(9);ddQ2=x(10);
%Q3=x(11);%dQ3=x(12);ddQ3=x(13);
x(8)  = Q2;
x(11) = Q3;

    % % 验证结果是否正确 (代回方程)
check_val_1 = L1*cos(Q1) + L2*cos(Q2) - L3*cos(Q3) - L4;
check_val_2 = L1*sin(Q1) + L2*sin(Q2) - L3*sin(Q3);
fprintf('验证 f1 (应接近0): %.4e\n', check_val_1);
fprintf('验证 f2 (应接近0): %.4e\n', check_val_2);

DrawPic(x);
%做出图像，验证结果
%完成初值求解
%开始运动学迭代
%设置初始角加速度ddQ1===设置初始驱动力矩
M=-5;
%设置质量矩阵
m1=5;
m2=5;
m3=5;
%设置转动惯量
J1=(1/3)*m1*L1^2;
J2=(1/12)*m2*L2^2;
J3=(1/3)*m3*L3^2;

%调用simulink
out= sim('Adams_HW_Dynamic.slx', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'ode4', ...
    'FixedStep', '1e-4', ...
    'StopTime', '5');

% %再绘图并验证
% Adams_HW_DrawGIF;
% Adams_HW_DrawGIF_Y;
Adams_HW1_DetectTheData;
%% 
function [Q2,Q3]=Adams_Newton_Laersen(x)

L1=x(1);L2=x(2);
L3=x(3);L4=x(4);
Q1=x(5);%dQ1=x(6);ddQ1=x(7);
%Q2=x(8);%dQ2=x(9);ddQ2=x(10);
%Q3=x(11);%dQ3=x(12);ddQ3=x(13);

%猜测Q2、Q3、L2、s的初始值
Q2_guess=20*pi/180;
Q3_guess=90*pi/180;

Inition=[Q2_guess;Q3_guess];

%检查敛散性,迭代参数设置
max_iter = 100;
tolerance=1e-9;


for iter = 1:max_iter

    % 提取当前迭代变量
    Q2_curr = Inition(1);
    Q3_curr=Inition(2);


%建立数值解预测初始值矩阵
GeometricConstraintsMatrixIntial=[
L1*cos(Q1) + L2*cos(Q2_curr) - L3*cos(Q3_curr) - L4;
L1*sin(Q1) + L2*sin(Q2_curr) - L3*sin(Q3_curr);
];

Jacob=[-(L2)*sin(Q2_curr),L3*sin(Q3_curr);
        (L2)*cos(Q2_curr),-L3*cos(Q3_curr);
];%Q2、Q3


if norm(GeometricConstraintsMatrixIntial) < tolerance
    fprintf('在第 %d 次迭代收敛。\n', iter);
    Q2= Inition(1);
    Q3=Inition(2);
    % % 输出结果
    fprintf('结果: Q2=%.4f rad,Q3= %.4f\n ', ...
          Q2,Q3);
    return;
end

delta = -Jacob \ GeometricConstraintsMatrixIntial;
Inition = Inition + delta;
end

warning('达到最大迭代次数，未收敛。请尝试调整初始猜测值。');
Q2= Inition(1);
Q3=Inition(2);
end



%% 3. 确定关键点坐标 (基于矢量环方程)
% 定义机架左支座为原点 R1
function DrawPic(x)
L1 = x(1); L2 = x(2); L3 = x(3); L4 = x(4);
Q1 = x(5);Q3=x(11);
O_R1 = [0, 0];
% 定义机架右支座 R4 (长度为 L4，水平布置)
O_R4 = [L4, 0];

% 确定曲柄末端 R2
P_R2 = O_R1 + [L1*cos(Q1), L1*sin(Q1)];
% 确定连杆与摇杆连接点 R3 (由 Q3 确定)
P_R3 = O_R4 + [L3*cos(Q3), L3*sin(Q3)];

%绘图输出

figure; hold on; grid on; axis equal;
xlabel('x 轴 (m)'); ylabel('y 轴 (m)'); 
title(['曲柄摇杆机构位置分析 (Q1 = ', num2str(rad2deg(Q1)), '°)']);

% 绘制机架 L4 (虚线)
plot([O_R1(1), O_R4(1)], [O_R1(2), O_R4(2)], 'k--', 'LineWidth', 2);

% 绘制曲柄 L1 (深蓝色)
plot([O_R1(1), P_R2(1)], [O_R1(2), P_R2(2)], 'LineWidth', 5, 'Color', [0 0.45 0.74]);

% 绘制连杆 L2 (橙红色)
plot([P_R2(1), P_R3(1)], [P_R2(2), P_R3(2)], 'LineWidth', 3, 'Color', [0.85 0.33 0.1]);

% 绘制摇杆 L3 (黄绿色)
plot([O_R4(1), P_R3(1)], [O_R4(2), P_R3(2)], 'LineWidth', 3, 'Color', [0.47 0.67 0.19]);

% 关键点标注 (节点)
scatter(O_R1(1), O_R1(2), 80, 'k', 'filled'); text(O_R1(1), O_R1(2)-0.5, '  R1 (旋转副)');
scatter(P_R2(1), P_R2(2), 60, 'k', 'filled'); text(P_R2(1), P_R2(2), '  R2 (旋转副)');
scatter(P_R3(1), P_R3(2), 60, 'k', 'filled'); text(P_R3(1), P_R3(2), '  R3 (旋转副)');
scatter(O_R4(1), O_R4(2), 80, 'k', 'filled'); text(O_R4(1), O_R4(2)-0.5, '  R4 (旋转副)');

end
%%
% function [ddQ2,ddQ3]=Adams_HW_Dynamic(x)
% L1=x(1);L2=x(2);L3=x(3);L4=x(4);
% Q1=x(5);dQ1=x(6);ddQ1=x(7);
% Q2=x(8);dQ2=x(9);ddQ2=x(10);
% Q3=x(11);dQ3=x(12);ddQ3=x(13);
% 
%   % k=4;%定义约束数
%   % [alpha,beta]=BSM(x,k);
%  alpha=0.1;beta=sqrt(2)*0.1;
% 
% % % 防御性检查：防止 NaN 传入导致整个仿真崩溃
% % alpha = min(alpha, 50); 
% % beta  = min(beta, 50);
% 
% 
% %BSM法相关定义
% %Phi为位置约束列向量
% Phi=[ L1*cos(Q1) + L2*cos(Q2) - L3*cos(Q3) - L4;
%     L1*sin(Q1) + L2*sin(Q2) - L3*sin(Q3)];
% 
% %dot_Phi为速度约束列向量
% Jacob=[-L1*sin(Q1),-(L2)*sin(Q2),L3*sin(Q3);
%         L1*cos(Q1),(L2)*cos(Q2),-L3*cos(Q3);
% ];%Q1、Q2、Q3
% 
% Phi_t=zeros(3,1);
% dot_Phi=Jacob*[dQ1;dQ2;dQ3]+Phi_t;
% 
% 
% 
% A_1=[L2*cos(Q2), - L3*cos(Q3);
%      L2*sin(Q2) ,- L3*sin(Q3)]*[dQ2^2;dQ3^2];
% 
% 
% A_2=[-L2*sin(Q2),  L3*sin(Q3);
%       L2*cos(Q2), -L3*cos(Q3)];%*[ddQ2;ddQ3]
% 
% 
% b_1=[ L1*sin(Q1); 
%      -L1*cos(Q1)]*ddQ1;
% b_2=[L1*cos(Q1);L1*sin(Q1)]*((dQ1)^2);
% 
% b=((b_1+b_2)-A_1)+(2*alpha*dot_Phi+(beta^2)*Phi);
% 
% if abs(det(A_2)) < 1e-6
%     % 如果矩阵接近奇异，使用伪逆 (pinv) 或者输出 0 防止报错
%     y = pinv(A_2) * b; 
% else
%     y = A_2\b;
% end
% 
% ddQ2=y(1);
% ddQ3=y(2);
% end