function v_rotated = rotate_vector(T, v)
% ROTATE_VECTOR 对三维向量仅进行旋转变换（忽略平移）
% 适用于变换方向向量、轴线、角速度等自由向量
%
% 输入:
%   T - 4x4 齐次变换矩阵 (提取左上角 3x3 旋转矩阵)
%   v - 3x1 列向量 或 3xN 矩阵 (待变换的方向向量)
%
% 输出:
%   v_rotated - 旋转后的向量 (长度保持不变)
%
% 原理:
%   v_new = R * v  (其中 R = T(1:3, 1:3))
%   注意：此函数不会应用 T 中的平移部分 T(1:3, 4)

    % 1. 检查输入 T
    if ~isequal(size(T), [4, 4])
        error('输入错误: T 必须是 4x4 的齐次变换矩阵。');
    end

    % 2. 提取旋转矩阵 R
    R = T(1:3, 1:3);

    % 3. 检查输入 v 的维度并标准化
    [rows, cols] = size(v);
    
    % 兼容行向量输入 (1x3 转 3x1)
    if rows == 1 && cols == 3
        v = v';
        [rows, cols] = size(v);
    end

    if rows ~= 3
        error('输入错误: v 必须是 3x1 的向量或 3xN 的矩阵。');
    end

    % 4. 执行旋转变换
    v_rotated = R * v;
    
    % 注意：如果 T 是标准旋转矩阵，理论上向量长度不变。
    % 只要输入 v 是单位向量，输出 v_rotated 也是单位向量，无需再次 norm。

end