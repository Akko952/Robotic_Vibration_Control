
function [d_sol, vars_sol_double, T_limb1_sol, T_limb2_sol, T_limb3_sol, info] = IK_3SPR(T_des, S1, S2, S3, T_01, x0, opts)
%IK_3SPR 3SPR 并联机构逆运动学（牛顿下山法：阻尼牛顿/LM + 线搜索）
%
% 目标：同时约束“位置 + 姿态”与目标位姿一致。
% 实现方式：构造残差向量 r(x)（18x1）：
%   - 对 3 条支链：位置残差 p_i - p_des（共 9 个）
%   - 对 3 条支链：姿态残差用 R_rel = R_des^T * R_i 的反对称部分（共 9 个）
%     然后最小化 f(x) = 1/2 * ||r(x)||^2。
%
% 输入:
%   T_des : 4x4 目标平台齐次变换矩阵（世界系）
%   S1,S2,S3 : 三条支链的螺旋轴结构体数组
%   T_01  : 零位平台位姿
%   x0    : (可选) 15 维初值（建议用上一帧解做 warm start）
%   opts  : (可选) 结构体
%           .maxIter (默认 50)
%           .tolF    (默认 1e-10)  % 目标函数阈值
%           .tolG    (默认 1e-8)   % 梯度阈值
%           .tolStep (默认 1e-10)  % 步长阈值
%           .fdEps   (默认 1e-6)   % 有限差分步长
%           .lambda0 (默认 1e-3)   % 初始阻尼
%           .verbose (默认 false)
%
% 输出:
%   d_sol          : 3x1 主动关节变量 [d1; d2; d3]
%   vars_sol_double: 15x1 全部未知量的数值解（顺序见下方）
%   T_limb*_sol    : 4x4 各支链末端位姿
%   info           : 迭代信息

if nargin < 5
	error('IK_3SPR:NotEnoughInputs', '需要输入 T_des, S1, S2, S3, T_01。');
end
if ~isequal(size(T_des), [4,4])
	error('IK_3SPR:InvalidTdes', 'T_des 必须是 4x4 齐次矩阵。');
end

if nargin < 6 || isempty(x0)
	x0 = zeros(1,15);
end
if nargin < 7
	opts = struct();
end

% x = [Q1 Q3 Q4 Q5,  P1 P3 P4 P5,  R1 R3 R4 R5,  d1 d2 d3]
x = x0(:);
if numel(x) ~= 15
	error('IK_3SPR:InvalidInitialGuess', 'x0 的长度必须为 15。');
end

maxIter = get_opt(opts, 'maxIter', 50);
tolF    = get_opt(opts, 'tolF',    1e-10);
tolG    = get_opt(opts, 'tolG',    1e-8);
tolStep = get_opt(opts, 'tolStep', 1e-10);
fdEps   = get_opt(opts, 'fdEps',   1e-6);
lambda  = get_opt(opts, 'lambda0', 1e-3);
verbose = get_opt(opts, 'verbose', false);

T_des = double(T_des);
p_des = T_des(1:3,4);
R_des = T_des(1:3,1:3);

% 初始残差/目标函数
[r, T1, T2, T3] = residual_3spr(x, S1, S2, S3, T_01, p_des, R_des);
f = 0.5 * (r.'*r);

if verbose
	fprintf('[IK] iter=%d, f=%.3e, ||r||=%.3e, lambda=%.3e\n', 0, f, norm(r), lambda);
end

exitflag = 0;
iter = 0;

for k = 1:maxIter
	iter = k;
	% 有限差分雅可比 J (18x15)
	J = jacobian_fd(@(xx) residual_only(xx, S1, S2, S3, T_01, p_des, R_des), x, fdEps);

	g = J.' * r; % 梯度（近似）
	if norm(g, inf) < tolG
		exitflag = 1;
		break;
	end

	% LM / 阻尼高斯-牛顿步
	A = (J.'*J) + lambda * eye(15);
	dx = -A \ g;

	if norm(dx, inf) < tolStep
		exitflag = 2;
		break;
	end

	% 回溯线搜索（下山）
	alpha = 1.0;
	f0 = f;
	% Armijo 条件：f(x+alpha*dx) <= f0 + c1*alpha*g'*dx
	c1 = 1e-4;
	gd = g.' * dx;
	if gd > 0
		% 理论上 dx 应该是下降方向；若不是，增大阻尼并重来
		lambda = lambda * 10;
		continue;
	end

	accepted = false;
	for ls = 1:20
		x_try = x + alpha * dx;
		[r_try, T1_try, T2_try, T3_try] = residual_3spr(x_try, S1, S2, S3, T_01, p_des, R_des);
		f_try = 0.5 * (r_try.'*r_try);

		if f_try <= f0 + c1 * alpha * gd
			accepted = true;
			x = x_try;
			r = r_try;
			f = f_try;
			T1 = T1_try; T2 = T2_try; T3 = T3_try;
			break;
		end
		alpha = alpha * 0.5;
	end

	if ~accepted
		% 线搜索失败：增大阻尼，尝试更保守的步
		lambda = lambda * 10;
	else
		% 若接受步，适当减小阻尼，加速收敛
		lambda = max(lambda / 3, 1e-12);
	end

	if verbose
		fprintf('[IK] iter=%d, f=%.3e, ||r||=%.3e, alpha=%.3g, lambda=%.3e\n', k, f, norm(r), alpha, lambda);
	end

	if f < tolF
		exitflag = 3;
		break;
	end
end

vars_sol_double = x;
d_sol = x(13:15);

T_limb1_sol = T1;
T_limb2_sol = T2;
T_limb3_sol = T3;

info.iter = iter;
info.exitflag = exitflag;
info.f = f;
info.norm_r = norm(r);
info.lambda = lambda;
end

function val = get_opt(opts, name, defaultVal)
if isstruct(opts) && isfield(opts, name) && ~isempty(opts.(name))
	val = opts.(name);
else
	val = defaultVal;
end
end
%% 辅助函数
function r = residual_only(x, S1, S2, S3, T_01, p_des, R_des)
[r, ~, ~, ~] = residual_3spr(x, S1, S2, S3, T_01, p_des, R_des);
end

function [r, T1, T2, T3] = residual_3spr(x, S1, S2, S3, T_01, p_des, R_des)
% 根据当前未知量 x 计算 18x1 残差
% x = [Q1 Q3 Q4 Q5,  P1 P3 P4 P5,  R1 R3 R4 R5,  d1 d2 d3]

Q1 = x(1);  Q3 = x(2);  Q4 = x(3);  Q5 = x(4);
P1 = x(5);  P3 = x(6);  P4 = x(7);  P5 = x(8);
R1v= x(9);  R3 = x(10); R4 = x(11); R5 = x(12);
d1 = x(13); d2 = x(14); d3 = x(15);

q1 = [Q1; d1; Q3; Q4; Q5];
q2 = [P1; d2; P3; P4; P5];
q3 = [R1v; d3; R3; R4; R5];

[T1, ~] = branch_forward_kinematics(S1, q1, T_01);
[T2, ~] = branch_forward_kinematics(S2, q2, T_01);
[T3, ~] = branch_forward_kinematics(S3, q3, T_01);

p1 = T1(1:3,4);
p2 = T2(1:3,4);
p3 = T3(1:3,4);

R1m = T1(1:3,1:3);
R2m = T2(1:3,1:3);
R3m = T3(1:3,1:3);

rp = [p1 - p_des; p2 - p_des; p3 - p_des];

rr1 = rot_residual(R1m, R_des);
rr2 = rot_residual(R2m, R_des);
rr3 = rot_residual(R3m, R_des);
rr = [rr1; rr2; rr3];

r = [rp; rr];
end
%% 姿态残差计算
function rr = rot_residual(R_i, R_des)
% 姿态残差：R_rel = R_des^T * R_i；取反对称部分的 3 个独立分量
R_rel = R_des.' * R_i;
rr = [R_rel(3,2) - R_rel(2,3);
	  R_rel(1,3) - R_rel(3,1);
	  R_rel(2,1) - R_rel(1,2)];
end

function J = jacobian_fd(fun, x, eps0)
% 有限差分雅可比（前向差分）
r0 = fun(x);
m = numel(r0);
n = numel(x);
J = zeros(m, n);

for j = 1:n
	h = eps0 * (1 + abs(x(j)));
	x1 = x;
	x1(j) = x1(j) + h;
	r1 = fun(x1);
	J(:,j) = (r1 - r0) / h;
end
end

