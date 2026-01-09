function A = vector_to_skew_symmetric(v)
    % 逆反对称矩阵函数
    %
    % 输入:
    %   v - 一个三维向量 [x, y, z]
    %
    % 输出:
    %   A - 一个3x3的反对称矩阵
    % 检查输入是否为三维向量
    if length(v) ~= 3
        error('输入必须是一个三维向量');
    end

    % 提取向量的分量
    x = v(1);
    y = v(2);
    z = v(3);

    % 构造3x3的反对称矩阵
    A = [  0  -z   y;
           z   0  -x;
          -y   x   0 ];
end