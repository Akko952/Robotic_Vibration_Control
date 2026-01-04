function frame = plot_screws_and_links(S, if_clear, radius, len, cylColor, lineColor)
%PLOT_SCREWS_AND_LINKS 在 ri 位置、沿 s_hat 方向绘制关节圆柱，并连接相邻 ri
%
% 输入:
%   S         : 结构体数组，每个元素包含字段 ri (3x1) 和 s_hat (3x1)，均为世界坐标系下描述
%   if_clear  : 布尔，true 则绘制后清空当前坐标轴
%   radius    : (可选) 圆柱半径，默认 10
%   len       : (可选) 圆柱长度，默认 30
%   cylColor  : (可选) 圆柱颜色，RGB，默认 [0 0 1]
%   lineColor : (可选) 连线颜色，RGB，默认 [0 0 0]
%
% 输出:
%   frame     : getframe 捕获的帧，可用于生成动画

    if nargin < 3 || isempty(radius),    radius   = 0.02;      end
    if nargin < 4 || isempty(len),       len      = 0.5;      end
    if nargin < 5 || isempty(cylColor),  cylColor = [0 0 1]; end
    if nargin < 6 || isempty(lineColor), lineColor= [0 0 0]; end

    hold on;
    n = numel(S);

    for i = 1:n
        ri    = S(i).ri;     % 世界系位置
        s_hat = S(i).s_hat;  % 世界系方向

        % 绘制关节圆柱
        draw_cylinder_at(ri, s_hat, radius, len, cylColor);

        % 连接相邻 ri
        if i < n
            rj = S(i+1).ri;
            draw_link_line(ri, rj, lineColor, 2);
        end
    end

    grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    axis equal;
    axis([-8, 8, -8, 8, -12, 12]); % 可按需调整
    view(-35, 25);

    drawnow;
    frame = getframe(gcf);
    if if_clear
        cla;
    end
end

%% 辅助函数：在 ri 位置、沿 dir 方向绘制圆柱
function draw_cylinder_at(p, dir, radius, len, col)
    % 归一化方向
    if norm(dir) < eps
        warning('s_hat 方向为零，跳过该圆柱绘制。');
        return;
    end
    az = dir / norm(dir);

    % 构造局部正交基 {ax, ay, az}
    az0 = [0;0;1];
    ax = cross(az0, az);
    if norm(ax) < eps
        % 若 az 与 z 轴平行，选用 x 轴做侧向
        ax = [1;0;0];
    else
        ax = ax / norm(ax);
    end
    ay = cross(az, ax);
    ay = ay / norm(ay);
    rot = [ax, ay, az];

    % 生成圆柱网格（局部坐标系）
    side_num   = 20;
    side_theta = linspace(0, 2*pi, side_num+1);
    x = [radius; radius] * cos(side_theta);
    y = [radius; radius] * sin(side_theta);
    z = [len/2; -len/2] * ones(1, side_num+1);

    % 旋转到世界系
    xyz_top    = rot * [x(1,:); y(1,:); z(1,:)];
    xyz_bottom = rot * [x(2,:); y(2,:); z(2,:)];

    x2 = [xyz_top(1,:); xyz_bottom(1,:)] + p(1);
    y2 = [xyz_top(2,:); xyz_bottom(2,:)] + p(2);
    z2 = [xyz_top(3,:); xyz_bottom(3,:)] + p(3);

    % 绘制侧面
    surf(x2, y2, z2, 'FaceColor', col, 'EdgeColor', 'none', 'FaceAlpha', 0.8);

    % 封顶/封底
    patch(x2(1,:), y2(1,:), z2(1,:), col, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
    patch(x2(2,:), y2(2,:), z2(2,:), col, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
end

%% 辅助函数：绘制连接线
function draw_link_line(p1, p2, col, lw)
    if nargin < 4, lw = 2; end
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
          'Color', col, 'LineWidth', lw);
end