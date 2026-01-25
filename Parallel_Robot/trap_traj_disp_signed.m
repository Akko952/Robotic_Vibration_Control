function [x, v, a, T] = trap_traj_disp_signed(s, vmax, amax, t)
%TRAP_TRAJ_DISP_SIGNED 梯形速度规划（支持正/负位移）
%
% 这是对 trap_traj_disp 的轻量封装：
% - 内部用 abs(s) 做规划，避免 s<0 导致 sqrt 出错
% - 输出 x/v/a 会自动带上方向（sign）
%
% 输入:
%   s     - 总位移（可正可负）
%   vmax  - 最大速度（正标量）
%   amax  - 最大加速度（正标量）
%   t     - 当前时间（标量或向量）
%
% 输出:
%   x,v,a - 位移/速度/加速度（与 s 同方向）
%   T     - 总运动时间

if ~isscalar(s)
    error('s 必须是标量。');
end

sgn = sign(s);
if sgn == 0
    x = zeros(size(t));
    v = zeros(size(t));
    a = zeros(size(t));
    T = 0;
    return;
end

[x0, v0, a0, T] = trap_traj_disp(abs(s), vmax, amax, t);

x = sgn * x0;
v = sgn * v0;
a = sgn * a0;
end
