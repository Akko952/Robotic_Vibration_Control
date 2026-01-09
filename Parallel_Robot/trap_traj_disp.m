function [x, v, a, T] = trap_traj_disp(s, vmax, amax, t)
%TRAP_TRAJ_DISP  梯形速度规划（位移型）
%
% 输入:
%   s     - 总位移（标量）
%   vmax  - 最大速度
%   amax  - 最大加速度
%   t     - 当前时间（可为标量或向量）
%
% 输出:
%   x     - 位移
%   v     - 速度
%   a     - 加速度
%   T     - 总运动时间

%% ---------- 1. 判断梯形 or 三角形 ----------
s_crit = vmax^2 / amax;

if s >= s_crit
    % 梯形速度
    ta = vmax / amax;          % 加速时间
    tc = (s - s_crit) / vmax;  % 匀速时间
    td = ta;                   % 减速时间
    T  = ta + tc + td;
    v_peak = vmax;
else
    % 三角形速度
    ta = sqrt(s / amax);
    tc = 0;
    td = ta;
    T  = ta + td;
    v_peak = amax * ta;
end

%% ---------- 2. 初始化 ----------
x = zeros(size(t));
v = zeros(size(t));
a = zeros(size(t));

%% ---------- 3. 分段计算 ----------
for i = 1:length(t)
    ti = t(i);

    if ti < 0
        x(i) = 0;
        v(i) = 0;
        a(i) = 0;

    elseif ti < ta
        % 加速段
        a(i) = amax;
        v(i) = amax * ti;
        x(i) = 0.5 * amax * ti^2;

    elseif ti < ta + tc
        % 匀速段
        a(i) = 0;
        v(i) = v_peak;
        x(i) = 0.5 * amax * ta^2 + v_peak * (ti - ta);

    elseif ti < T
        % 减速段
        t_dec = ti - ta - tc;
        a(i) = -amax;
        v(i) = v_peak - amax * t_dec;
        x(i) = 0.5 * amax * ta^2 ...
             + v_peak * tc ...
             + v_peak * t_dec ...
             - 0.5 * amax * t_dec^2;

    else
        % 结束
        x(i) = s;
        v(i) = 0;
        a(i) = 0;
    end
end
end
