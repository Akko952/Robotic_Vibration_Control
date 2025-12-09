function [ddL2,ddQ2,dds,ddQ3]=MechanicalMechanics_Dynamic(x)
L1=x(1);L2=x(2);L3=x(3);L4=x(4);
Q1=x(5);dQ1=x(6);ddQ1=x(7);
Q2=x(8);dQ2=x(9);ddQ2=x(10);
Q3=x(11);dQ3=x(12);ddQ3=x(13);
s=x(14);ds=x(15);dds=x(16);
dL2=x(17);ddL2=x(18);D=x(19);

 % k=4;%定义约束数
 % [alpha,beta]=BSM(x,k);
 alpha=0.1;beta=sqrt(2)*0.1;
% k1=alpha;k2=beta;


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

A_1=[2*cos(Q2),-L2*sin(Q2),0,0;
    -2*sin(Q2),-L2*cos(Q2),0,0;
    0,-L3*sin(Q2),0,L4*cos(Q3);
    0,L3*cos(Q2),0,-L4*sin(Q3)
]*[dQ2*dL2;dQ2^2;ds;dQ3^2];


A_2=[sin(Q2),L2*cos(Q2),0,0;
    cos(Q2),-L2*sin(Q2),0,0;
    0,L3*cos(Q2),1,L4*sin(Q3);
    0,L3*sin(Q2),0,L4*cos(Q3)
];%[ddL2;ddQ2;dds;ddQ3]


b_1=[L1*cos(Q1);-L1*sin(Q1);0;0]*ddQ1;
b_2=[-L1*sin(Q1);-L1*cos(Q1);0;0]*((dQ1)^2);

b=((b_1+b_2)-A_1)-(2*alpha*dot_Phi+(beta^2)*Phi);

if abs(det(A_2)) < 1e-6
    % 如果矩阵接近奇异，使用伪逆 (pinv) 或者输出 0 防止报错
    y = pinv(A_2) * b; 
else
    y = A_2\b;
end

ddL2=y(1);
ddQ2=y(2);
dds=y(3);
ddQ3=y(4);
end

