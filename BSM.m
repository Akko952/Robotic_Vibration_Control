%% Baumgarte 违约稳定法+自适应参数选择
% 由于数值截断误差不可避免，，即产生了速度约束违约与位移约束 违 约．为 了 减 小 约 束 违 约 对 系 统 的 影 响，
% １９７２ 年，Ｂａｕｍｇａｒｔｅ 提 出 了 约 束 违 约 稳 定 法
% （ＢＳＭ），该方法利用反馈控制理论，将位移约束和速度约束引入加速度约束方程，通过约束修正得到稳定化的动力学方程
function [alpha,beta]=BSM(x,n)
L1=x(1);L2=x(2);L3=x(3);L4=x(4);
Q1=x(5);dQ1=x(6);ddQ1=x(7);
Q2=x(8);dQ2=x(9);ddQ2=x(10);
Q3=x(11);dQ3=x(12);ddQ3=x(13);
s=x(14);ds=x(15);dds=x(16);
dL2=x(17);ddL2=x(18);D=x(19);


%相关定义
%Phi为位置约束列向量
Phi=[ L1*sin(Q1) - L2*sin(Q2);
    D+L1*cos(Q1) - L2*cos(Q2);
    L4*cos(Q3) - L3*sin(Q2)-s;
    L4*sin(Q3) - L3*cos(Q2);];

%dot_Phi为速度约束列向量

dot_Phi=[L1*cos(Q1)*dQ1-dL2*sin(Q2)-L2*cos(Q2)*dQ2;
           -L1*sin(Q1)*dQ1-dL2*cos(Q2)+L2*sin(Q2)*dQ2;
           -L4*sin(Q3)*dQ3-L3*cos(Q2)*dQ2-ds;
           L4*cos(Q3)*dQ3+L3*sin(Q2)*dQ2];


%ddot_Phi为加速度约束列向量

ddot_Phi = [
    % 第 1 行
    L1*(cos(Q1)*ddQ1 - sin(Q1)*dQ1^2) ...
    - ddL2*sin(Q2) - 2*dL2*cos(Q2)*dQ2 ...
    + L2*sin(Q2)*dQ2^2 - L2*cos(Q2)*ddQ2;
    
    % 第 2 行
    -L1*(sin(Q1)*ddQ1 + cos(Q1)*dQ1^2) ...
    - ddL2*cos(Q2) + 2*dL2*sin(Q2)*dQ2 ...
    + L2*cos(Q2)*dQ2^2 + L2*sin(Q2)*ddQ2;
    
    % 第 3 行
    -L4*(sin(Q3)*ddQ3 + cos(Q3)*dQ3^2) ...
    - L3*(cos(Q2)*ddQ2 - sin(Q2)*dQ2^2) ...
    - dds;
    
    % 第 4 行
    L4*(cos(Q3)*ddQ3 - sin(Q3)*dQ3^2) ...
    + L3*(sin(Q2)*ddQ2 + cos(Q2)*dQ2^2)
];

err_Phi=lg((sqrt(Phi'*Phi))/n);
err_dotPhi=lg((sqrt(dot_Phi'*dot_Phi))/n);

%定义计算误差、乘法函数
err=1;

% 定义分段函数
ec = @(z) (z >= -6) * 10 + (z <= -16) * 0 + ...
          (z > -16 & z < -6) .* (16 + z);

y=@(z) 10*err*ec(z);

%计算算子
alpha=y(err_Phi);
beta=y(err_dotPhi);

end