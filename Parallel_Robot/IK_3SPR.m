function [d, T_p, residual] = IK_3SPR(Z, alpha, beta, p)
% --------------------------------------------------
% 3-SPR 并联机构逆运动学
% 输入:
%   Z, alpha, beta : 动平台自由度 (2R1T)
%   p : 动平台位置和姿态参数结构体，包含基座 S 副中心和平台 R 副中心信息
%
% 输出:
%   d : 3x1 主动 P 关节位移
%   T_p : 动平台位姿
%   residual : 约束残差
% --------------------------------------------------
    A = {p.s1, p.s2, p.s3};   % 基座点
    B = {p.b1, p.b2, p.b3};   % 平台点
    %% 1. 构造旋转矩阵（无绕 Z 轴）
    Rx = [1 0 0;
          0 cos(alpha) -sin(alpha);
          0 sin(alpha)  cos(alpha)];
      
    Ry = [ cos(beta) 0 sin(beta);
            0        1     0;
          -sin(beta) 0 cos(beta)];

    R = Rx * Ry; % Z-YX (2R1T) 顺序旋转矩阵

    %% 2. P 副方向（3SPR 通常竖直）
    ez = [0;0;1];

    %% 3. 求解 X,Y（线性约束）
    M = [];
    c = [];

    for i = 1:3
        bi = R * B{i};
        Ai = A{i};

        % 约束：水平分量必须相等
        M = [M;
             1 0;
             0 1];

        c = [c;
             Ai(1) - bi(1);
             Ai(2) - bi(2)];
    end

    XY = M \ c;
    X = XY(1);
    Y = XY(2);

    P = [X;Y;Z];

    %% 4. 计算 P 副位移
    d = zeros(3,1);
    residual = 0;

    for i = 1:3
        bi = R * B{i};   % 平台点（转到基座系）
        Ai = A{i};       % 基座点
        d(i) = ez' * (P + bi - Ai);
        residual = residual + norm(P + bi - Ai - d(i)*ez);
    end

    residual = residual / 3;

    %% 5. 平台位姿
    T_p = [R P;
           0 0 0 1];
end
