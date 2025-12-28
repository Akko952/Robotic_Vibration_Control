% 清空工作区和命令行
clear; clc;
syms Q1 Q2 Q3 L1 L2 L3 L4 dQ1 


symsA = [sin(Q2),   L2*cos(Q2),  0,  0;
     cos(Q2),  -L2*sin(Q2),  0,  0;
     0,         L3*cos(Q2),  1,  L4*sin(Q3);
     0,         L3*sin(Q2),  0,  L4*cos(Q3)];

symsb = [L1*cos(Q1); 
    -L1*sin(Q1); 
     0; 
     0] * dQ1;

% 求解线性方程组
solution = -symsA \ symsb;
solution = simplify(solution);


SYMS_dL2 = solution(1);
SYMS_dQ2 = solution(2);
SYMS_ds  = solution(3);
SYMS_dQ3 = solution(4);


