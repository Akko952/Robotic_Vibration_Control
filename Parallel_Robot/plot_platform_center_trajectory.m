function [C_world] = plot_platform_center_trajectory(T_seq, p, method)
%PLOT_PLATFORM_CENTER_TRAJECTORY 单独绘制动平台中心轨迹
%
% C_world = plot_platform_center_trajectory(T_seq)
% C_world = plot_platform_center_trajectory(T_seq, p)
% C_world = plot_platform_center_trajectory(T_seq, p, method)
%
% 输入:
%   T_seq  : 4x4xN 平台位姿序列（齐次矩阵）
%   p      : (可选) 结构体，至少包含 p.b1/p.b2/p.b3（动平台坐标系下连接点）
%   method : (可选)
%            'T'    -> 直接取 T_seq(1:3,4,:) 作为中心（平台原点轨迹）
%            'mean' -> 取 (b1+b2+b3)/3 变换到世界系作为中心（默认，需要 p）
%
% 输出:
%   C_world: Nx3 轨迹点（世界坐标系）

if nargin < 3 || isempty(method)
    if nargin >= 2 && ~isempty(p)
        method = 'mean';
    else
        method = 'T';
    end
end

if ndims(T_seq) ~= 3 || size(T_seq,1) ~= 4 || size(T_seq,2) ~= 4
    error('T_seq 必须是 4x4xN 的齐次矩阵序列。');
end

N = size(T_seq, 3);

switch lower(method)
    case 't'
        C_world = squeeze(T_seq(1:3,4,:)).'; % Nx3

    case 'mean'
        if nargin < 2 || isempty(p) || ~isfield(p,'b1') || ~isfield(p,'b2') || ~isfield(p,'b3')
            error("method='mean' 需要传入 p，并包含字段 p.b1/p.b2/p.b3。");
        end
        center_local = (p.b1 + p.b2 + p.b3) / 3;
        C_world = zeros(N, 3);
        for k = 1:N
            C_world(k,:) = transform_Point_vector(T_seq(:,:,k), center_local).';
        end

    otherwise
        error("未知 method：%s（可选 'T' 或 'mean'）。", method);
end

figure('Name','Platform Center Trajectory', 'NumberTitle','off');
plot3(C_world(:,1), C_world(:,2), C_world(:,3), 'r-', 'LineWidth', 2);
hold on;
plot3(C_world(1,1), C_world(1,2), C_world(1,3), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 6);
plot3(C_world(end,1), C_world(end,2), C_world(end,3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
hold off;

grid on; axis equal; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('动平台中心轨迹');
legend({'Trajectory','Start','End'}, 'Location', 'best');
end
