function make_robot_gif(T_seq, p, T_12, T_13, gifFile, delayTime)
%MAKE_ROBOT_GIF 用 reflash_Kinematic 绘制并导出 GIF
%
% 输入:
%   T_seq     : 4x4xN 平台位姿序列（齐次矩阵）
%   p,T_12,T_13: 与 reflash_Kinematic 相同
%   gifFile   : 输出 GIF 文件名，例如 'robot.gif'
%   delayTime : 每帧间隔（秒），例如 0.05
%
% 用法示例：
%   % 已有 Kinematic; 得到 p,T_12,T_13
%   % 已有 T_seq(:,:,k)
%   make_robot_gif(T_seq, p, T_12, T_13, 'robot.gif', 0.05);

if nargin < 6 || isempty(delayTime)
    delayTime = 0.05;
end

if ndims(T_seq) ~= 3 || size(T_seq,1) ~= 4 || size(T_seq,2) ~= 4
    error('T_seq 必须是 4x4xN 的齐次矩阵序列。');
end

% 重新开始一段动画时，建议清空 persistent（避免沿用上一段的轴缓存）
clear reflash_Kinematic

N = size(T_seq, 3);
for k = 1:N
    T_des = T_seq(:, :, k);

    % reflash_Kinematic 内部已固定使用独立图窗绘制，并返回 frame
    [~, ~, ~, frame] = reflash_Kinematic(T_des, p, T_12, T_13);

    img = frame2im(frame);
    [A, map] = rgb2ind(img, 256);

    if k == 1
        imwrite(A, map, gifFile, 'gif', 'LoopCount', inf, 'DelayTime', delayTime);
    else
        imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
    end
end

fprintf('GIF 已生成：%s (帧数=%d)\n', gifFile, N);
end
