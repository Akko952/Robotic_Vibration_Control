function [dL2,dQ2,ds,dQ3]=MechanicalMechanics_Kinematics(x)
L1=x(1);L2=x(2);L3=x(3);L4=x(4);
Q1=x(5);dQ1=x(6);ddQ1=x(7);
Q2=x(8);dQ2=x(9);ddQ2=x(10);
Q3=x(11);dQ3=x(12);ddQ3=x(13);
s=x(14);ds=x(15);dds=x(16);
dL2=x(17);ddL2=x(18);
D=x(19);

A=[sin(Q2),L2*cos(Q2),0,0;
    cos(Q2),-L2*sin(Q2),0,0;
    0,L3*cos(Q2),1,L4*sin(Q3);
    0,L3*sin(Q2),0,L4*cos(Q3)
];
b=[L1*cos(Q1);-L1*sin(Q1);0;0]*dQ1;
%y=-A\b;%[dL2;dQ2;ds;dQ3]
if abs(det(A)) < 1e-6
    % 如果矩阵接近奇异，使用伪逆 (pinv) 或者输出 0 防止报错
    % 注意：pinv 在 MATLAB Function 模块中可能不支持代码生成，
    % 如果报错，可以改用简单的 A\b 但先给对角线加微小扰动
    y = -pinv(A) * b; 
    % 或者简单地保持上一时刻的值（需要状态记忆，较复杂），这里先尝试伪逆
else
    y = -A\b;
end

dL2=y(1);
dQ2=y(2);
ds=y(3);
dQ3=y(4);
end
