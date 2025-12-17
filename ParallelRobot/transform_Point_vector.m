function v_transformed = transform_Point_vector(T, v)
% TRANSFORM_VECTOR 对三维向量进行齐次坐标变换
%
% 输入:
%   T - 4x4 齐次变换矩阵 (Homogeneous Transformation Matrix)
%   v - 3x1 列向量 或 3xN 矩阵 (每一列是一个待变换的三维点)
%
% 输出:
%   v_transformed - 变换后的 3x1 向量 或 3xN 矩阵
%
% 使用示例:
%   T = [eye(3), [10;0;0]; 0 0 0 1]; % 沿x轴平移10
%   p = [1; 2; 3];
%   p_new = transform_vector(T, p);

    % 1. 检查输入 T 的维度
    if ~isequal(size(T), [4, 4])
        error('输入错误: T 必须是 4x4 的齐次变换矩阵。');
    end

    % 2. 检查输入 v 的维度并标准化
    [rows, cols] = size(v);
    
    % 如果输入是行向量 (1x3)，自动转置为列向量，方便计算
    if rows == 1 && cols == 3
        v = v';
        [rows, cols] = size(v);
        warning('检测到输入为行向量，已自动转置为列向量计算。');
    end

    if rows ~= 3
        error('输入错误: v 必须是 3x1 的向量或 3xN 的点集矩阵。');
    end

    % 3. 执行齐次变换
    % 方法：先扩充为齐次坐标 [x; y; z; 1]，乘完后再取前三行
    
    % 构建齐次坐标矩阵 (在底部加一行全为1的行向量)
    v_homogeneous = [v; ones(1, cols)];
    
    % 矩阵乘法 T * P
    v_transformed_homo = T * v_homogeneous;
    
    % 4. 提取结果 (取前3行)
    v_transformed = v_transformed_homo(1:3, :);

end