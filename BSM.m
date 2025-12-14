%% Baumgarte 违约稳定法+自适应参数选择
% 由于数值截断误差不可避免，，即产生了速度约束违约与位移约束 违 约．为 了 减 小 约 束 违 约 对 系 统 的 影 响，
% １９７２ 年，Ｂａｕｍｇａｒｔｅ 提 出 了 约 束 违 约 稳 定 法
% （ＢＳＭ），该方法利用反馈控制理论，将位移约束和速度约束引入加速度约束方程，通过约束修正得到稳定化的动力学方程
function [alpha,beta]=BSM(x,n)
%定义相关变量
L1=x(1);L2=x(2);L3=x(3);L4=x(4);
Q1=x(5);dQ1=x(6);ddQ1=x(7);
Q2=x(8);dQ2=x(9);ddQ2=x(10);
Q3=x(11);dQ3=x(12);ddQ3=x(13);
s=x(14);ds=x(15);dds=x(16);
dL2=x(17);ddL2=x(18);D=x(19);


%BSM相关定义
%Phi为位置约束列向量
Phi=[ L1*sin(Q1) - L2*sin(Q2);
    D+L1*cos(Q1) - L2*cos(Q2);
    L4*cos(Q3) - L3*sin(Q2)-s;
    L4*sin(Q3) - L3*cos(Q2)];%=0

%dot_Phi为速度约束列向量
% Jacob=[(L1)*cos(Q1),-(L2)*cos(Q2),0,0;
%         -(L1)*sin(Q1),(L2)*cos(Q2),0,0;
%         0,-(L3)*cos(Q2),-1,-(L4)*sin(Q3);
%         0,(L3)*sin(Q2),0,(L4)*cos(Q3)
% ];%Q1、Q2、s、Q3

Jacob=[L1*cos(Q1),-sin(Q2),-(L2)*cos(Q2),0,0;
        -L1*sin(Q1),-cos(Q2),(L2)*sin(Q2),0,0;
        0,0,-(L3)*cos(Q2),-1,-(L4)*sin(Q3);
        0,0,(L3)*sin(Q2),0,(L4)*cos(Q3)
];%Q1、L2、Q2、s、Q3=0
%这里的雅可比为全部坐标的雅可比
%对位置约束求一阶导数，根据链式求导法则，为雅可比矩阵乘速度矢量

 % Phi_t=[L1*cos(Q1)*dQ1-dL2*sin(Q2)-L2*cos(Q2)*dQ2;
 %            -L1*sin(Q1)*dQ1-dL2*cos(Q2)+L2*sin(Q2)*dQ2;
 %            -L4*sin(Q3)*dQ3-L3*cos(Q2)*dQ2-ds;
 %            L4*cos(Q3)*dQ3+L3*sin(Q2)*dQ2];
Phi_t=zeros(4,1);
dot_Phi=Jacob*[dQ1;dL2;dQ2;ds;dQ3]+Phi_t;

%ddot_Phi为加速度约束列向量

% ddot_Phi = [
%     % 第 1 行
%     L1*(cos(Q1)*ddQ1 - sin(Q1)*dQ1^2) ...
%     - ddL2*sin(Q2) - 2*dL2*cos(Q2)*dQ2 ...
%     + L2*sin(Q2)*dQ2^2 - L2*cos(Q2)*ddQ2;
% 
%     % 第 2 行
%     -L1*(sin(Q1)*ddQ1 + cos(Q1)*dQ1^2) ...
%     - ddL2*cos(Q2) + 2*dL2*sin(Q2)*dQ2 ...
%     + L2*cos(Q2)*dQ2^2 + L2*sin(Q2)*ddQ2;
% 
%     % 第 3 行
%     -L4*(sin(Q3)*ddQ3 + cos(Q3)*dQ3^2) ...
%     - L3*(cos(Q2)*ddQ2 - sin(Q2)*dQ2^2) ...
%     - dds;
% 
%     % 第 4 行
%     L4*(cos(Q3)*ddQ3 - sin(Q3)*dQ3^2) ...
%     + L3*(sin(Q2)*ddQ2 + cos(Q2)*dQ2^2)
% ];

% 1. 防止除以0 (n如果没传或者是0)
if nargin < 2 || n == 0
    n = 4; % 默认约束方程个数
end

% 2. 计算模长，加上 eps 防止 log(0)
norm_Phi = sqrt(Phi'*Phi/n);
norm_dotPhi = sqrt(dot_Phi'*dot_Phi/n);

% 使用 log10 替换 lg，并加入 eps 防止负无穷
err_Phi = log10(norm_Phi +eps); 
err_dotPhi = log10(norm_dotPhi +eps); 

%定义计算误差、乘法函数
err=1;%定步长算法

% 定义分段函数
ec = @(z) (z >= -6) * 10 + (z <= -16) * 0 + ...
          (z > -16 & z < -6) .* (16 + z);

y=@(z) 10*err*ec(z);

%计算算子
alpha=y(err_dotPhi);
beta=y(err_Phi);
%做数据保护
alpha = min(max(alpha,0),100); beta = min(max(beta,0),100);
end