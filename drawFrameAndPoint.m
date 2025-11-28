function h = drawFrameAndPoint(p1, R, p2, len, ax)
% drawFrameAndPoint 在点 p1 处绘制局部坐标系并显示局部坐标系下点(们) p2 的全局位置
%
%  p1  : 1x3 或 3x1，局部坐标系原点（全局坐标）
%  R   : 3x3，局部->全局的旋转矩阵（列为局部 x,y,z 轴在全局中的方向）
%  p2  : 局部坐标系下的点
%        - 1x3 / 3x1：单个点
%        - N x 3    ：多个点（每行一个局部坐标）
%  len : 坐标轴箭头长度（默认 1）
%  ax  : 目标坐标轴句柄（默认 gca）
%
% 返回：
%  h : 句柄向量
%      h(1:3)  = x,y,z 三个轴的 quiver3 句柄
%      h(4:end)= 绘制的点的句柄（可能为多个）
%
% 说明：
%  全局点坐标 = p1(:).' + (R * p_loc(:))
%  多点时：P_global(i,:) = p1 + (R * p2(i,:).')
%
% 示例：
%   R = eye(3);
%   p1 = [0 0 0];
%   p_local = [1 2 3];
%   figure; axis equal; grid on;
%   h = drawFrameAndPoint(p1, R, p_local);
%
% 批量点示例：
%   P_local = [1 0 0; 0.5 0.5 0.2; -0.2 1 0.3];
%   h = drawFrameAndPoint(p1, R, P_local, 1.5);
%
% 注意：
%  若需要自定义点样式，可在返回句柄后自行 set。
%

    if nargin < 5 || isempty(ax), ax = gca; end
    if nargin < 4 || isempty(len), len = 50; end

    % 规范 p1
    p1 = p1(:).';
    assert(numel(p1) == 3, 'p1 必须为 3 维坐标');
    assert(all(size(R) == [3 3]), 'R 必须是 3x3 旋转矩阵');

    % 调用已有的坐标系绘制函数
    hFrame = drawFrame(p1, R, len, ax);  % 需要已有 drawFrame.m 在路径中

    % 规范 p2
    if isvector(p2) && numel(p2) == 3
        P_local = p2(:).';          % 1x3
    else
        assert(size(p2,2) == 3, 'p2 必须是 1x3 / 3x1 或 N×3 的局部坐标');
        P_local = p2;               % N×3
    end

    nPts = size(P_local,1);
    P_global = (R * P_local.').';   % N×3 局部->全局方向变换（不含平移）
    P_global = P_global + p1;       % 加上原点平移

    hold(ax,'on');
    hPts = gobjects(1,nPts);

    % 为多点生成默认标签
    defaultLabels = arrayfun(@(i) sprintf('P%d', i), 1:nPts, 'UniformOutput', false);

    for i = 1:nPts
        pg = P_global(i,:);
        hPts(i) = plot3(ax, pg(1), pg(2), pg(3), 'ko', ...
                        'MarkerFaceColor','y', 'MarkerSize',8);
        % 连线：从局部原点到该点的实线
        randColor = rand(1,3);  % 生成 0~1 之间的 RGB
        line(ax, [p1(1) pg(1)], [p1(2) pg(2)], [p1(3) pg(3)], ...
             'LineStyle','-', 'Color', randColor, 'LineWidth',1.2)
        % 文字标注
        text(ax, pg(1), pg(2), pg(3), ['  ' defaultLabels{i}], ...
             'Color','k', 'FontWeight','bold', ...
             'HorizontalAlignment','left', 'VerticalAlignment','middle');
    end

    % 输出统一句柄
    h = [hFrame, hPts];

    % 视觉设定（避免被覆盖）
    grid(ax,'on');
    axis(ax,'equal');
end