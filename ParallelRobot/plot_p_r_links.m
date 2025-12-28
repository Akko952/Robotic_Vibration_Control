function frame = plot_p_r_links(p, S, if_clear, color_r, color_p, color_dash)
    if nargin < 3 || isempty(if_clear),   if_clear   = false;      end
    if nargin < 4 || isempty(color_r),    color_r    = [0 0 1];    end
    if nargin < 5 || isempty(color_p),    color_p    = [1 0 0];    end
    if nargin < 6 || isempty(color_dash), color_dash = [0 0 0];    end

    hold on;

    % --- 收集 p.si ---
    p_list = [];
    if isfield(p,'s1'), p_list(:,end+1) = p.s1; end
    if isfield(p,'s2'), p_list(:,end+1) = p.s2; end
    if isfield(p,'s3'), p_list(:,end+1) = p.s3; end

    % --- 收集 ri 列表（兼容结构体或 3xN 矩阵） ---
    if isstruct(S)
        nS = numel(S);
        r_list = zeros(3,nS);
        for i = 1:nS
            r_list(:,i) = S(i).ri;
        end
    else
        if size(S,1) ~= 3
            error('若传矩阵，S 必须是 3×N');
        end
        r_list = S;
        nS = size(S,2);
    end

    % --- 连接 ri，首尾闭合 ---
    if nS >= 2
        r_closed = [r_list, r_list(:,1)];
        plot3(r_closed(1,:), r_closed(2,:), r_closed(3,:), 'Color', color_r, 'LineWidth', 2);
    end
    scatter3(r_list(1,:), r_list(2,:), r_list(3,:), 50, color_r, 'filled');

    % --- 连接 p.si，首尾闭合 ---
    if size(p_list,2) >= 2
        p_closed = [p_list, p_list(:,1)];
        plot3(p_closed(1,:), p_closed(2,:), p_closed(3,:), 'Color', color_p, 'LineWidth', 2);
    end
    scatter3(p_list(1,:), p_list(2,:), p_list(3,:), 60, color_p, 'filled', 'MarkerEdgeColor', [0 0 0]);

    % --- 原点到各 p.si 的虚线 ---
    for k = 1:size(p_list,2)
        plot3([0, p_list(1,k)], [0, p_list(2,k)], [0, p_list(3,k)], '--', 'Color', color_dash, 'LineWidth', 1.5);
    end

    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(-35,25);
    drawnow;
    frame = getframe(gcf);
    if if_clear, cla; end
end