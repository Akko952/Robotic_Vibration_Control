function [vars_sol_double,T_limb1_sol, Ad_all_1_sol, T_limb2_sol, Ad_all_2_sol, T_limb3_sol, Ad_all_3_sol] = FK_3SPR(S1,S2,S3,d1, d2, d3,T_01, x0)
syms Q1  Q3 Q4 Q5
syms P1  P3 P4 P5
syms R1  R3 R4 R5

%计算目标下的POE，由此得到开环Limb的T_s以及伴随矩阵
[T_limb1_des, Ad_all_1_des]=branch_forward_kinematics(S1,[Q1; d1; Q3; Q4; Q5],T_01);%目标时刻末端位姿
[T_limb2_des, Ad_all_2_des]=branch_forward_kinematics(S2,[P1; d2; P3; P4; P5],T_01);%目标时刻末端位姿
[T_limb3_des, Ad_all_3_des]=branch_forward_kinematics(S3,[R1; d3; R3; R4; R5],T_01);%目标时刻末端位姿
p1_des=T_limb1_des(1:3,4);
p2_des=T_limb2_des(1:3,4);
p3_des=T_limb3_des(1:3,4);
%%得到未知数方程组
eq12 = (p1_des == p2_des);   % [p1x==p2x; p1y==p2y; p1z==p2z]
eq23 = (p2_des == p3_des);
eqs_p = [eq12; eq23];         % 一共 6 个标量方程
%%得到目标时刻的旋转矩阵
R1_des=T_limb1_des(1:3,1:3);
R2_des=T_limb2_des(1:3,1:3);
R3_des=T_limb3_des(1:3,1:3);
%% 构建旋转矩阵相等的约束方程（每个闭环3个独立方程）

% 对于第一个闭环：R1_des == R2_des
R_diff12 = R1_des - R2_des;
% 取反对称部分（上三角减下三角）对应的3个独立方程
eq_rot12 = [R_diff12(3,2) - R_diff12(2,3);   % ω_z 分量
            R_diff12(1,3) - R_diff12(3,1);   % ω_y 分量  
            R_diff12(2,1) - R_diff12(1,2)] == [0;0;0]; % ω_x 分量

% 对于第二个闭环：R2_des == R3_des  
R_diff23 = R2_des - R3_des;
eq_rot23 = [R_diff23(3,2) - R_diff23(2,3);
            R_diff23(1,3) - R_diff23(3,1);
            R_diff23(2,1) - R_diff23(1,2)] == [0;0;0];

% 合并所有方程：位置(6) + 旋转(6) = 12个独立方程
eqs = [eqs_p; eq_rot12; eq_rot23];

%% 求解方程组（零位作为初值）
vars = [Q1 Q3 Q4 Q5  P1 P3 P4 P5  R1 R3 R4 R5];

if nargin < 8 || isempty(x0)
  x0 = zeros(size(vars));
end

% 允许传入行/列向量
if isrow(x0)
  x0 = x0(:).';
end
if numel(x0) ~= numel(vars)
  error('FK_3SPR:InvalidInitialGuess', 'x0 的长度必须为 %d。', numel(vars));
end

S_sol = vpasolve(eqs, vars, x0);

if isempty(fieldnames(S_sol))
  error('vpasolve 未收敛：请尝试更换初值/给定范围，或检查方程是否一致。');
end

vars_sol = [S_sol.Q1; S_sol.Q3; S_sol.Q4; S_sol.Q5; ...
      S_sol.P1; S_sol.P3; S_sol.P4; S_sol.P5; ...
      S_sol.R1; S_sol.R3; S_sol.R4; S_sol.R5];

vars_sol_double = double(vars_sol);
%计算求解出后的T_limb
[T_limb1_sol, Ad_all_1_sol]=branch_forward_kinematics(S1,[vars_sol_double(1); d1; vars_sol_double(2); vars_sol_double(3); vars_sol_double(4)],T_01);%目标时刻末端位姿
[T_limb2_sol, Ad_all_2_sol]=branch_forward_kinematics(S2,[vars_sol_double(5); d2; vars_sol_double(6); vars_sol_double(7); vars_sol_double(8)],T_01);%目标时刻末端位姿
[T_limb3_sol, Ad_all_3_sol]=branch_forward_kinematics(S3,[vars_sol_double(9); d3; vars_sol_double(10); vars_sol_double(11); vars_sol_double(12)],T_01);%目标时刻末端位姿
end
