function h = drawFrame(p, R, len, ax)
% drawFrame 在点 p 处绘制一个自定义坐标系（三轴箭头）
%  p   : 1x3 或 3x1，局部坐标系原点（全局坐标）
%  R   : 3x3，局部->全局的旋转矩阵（列向量为局部 x,y,z 轴在全局坐标下的方向）
%  len : 标尺长度（箭头长度，数据单位），默认 1
%  ax  : 目标坐标轴句柄，默认 gca
%
% 返回值 h 为包含三个 quiver3 句柄的向量（[hx, hy, hz]）

    if nargin < 4 || isempty(ax), ax = gca; end
    if nargin < 3 || isempty(len), len = 1; end

    p = p(:).';
    assert(all(size(R) == [3 3]), 'R 必须是 3x3 旋转矩阵');

    % 颜色与标签
    cols = {'r','g','b'};
    labs = {'x','y','z'};

    hold(ax, 'on');
    h = gobjects(1,3);
    for i = 1:3
        v = len * R(:,i).';                 % 该轴方向向量（全局）
        h(i) = quiver3(ax, p(1), p(2), p(3), v(1), v(2), v(3), ...
                       0, 'LineWidth', 2, 'Color', cols{i}, ...
                       'MaxHeadSize', 0.6); % AutoScale=0
        text(ax, p(1) + 1.05*v(1), p(2) + 1.05*v(2), p(3) + 1.05*v(3), ...
             labs{i}, 'Color', cols{i}, 'FontWeight','bold', ...
             'HorizontalAlignment','center', 'VerticalAlignment','middle');
    end

    grid(ax, 'on');
    axis(ax, 'equal');
end