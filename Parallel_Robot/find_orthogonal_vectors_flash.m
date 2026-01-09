function omega_si5 = find_orthogonal_vectors_flash(s3, s4, s5_prev)
% 用于球铰第三轴的连续更新（Parallel Transport）
%
% 输入：
%   s3       - 当前帧主轴（移动副方向）
%   s4       - 仅用于【初始化】的参考轴（后续不再绑定）
%   s5_prev  - 上一帧的第三轴
%
% 输出：
%   omega_si5 - 当前帧连续更新后的第三轴

    % ---------- 单位化 ----------
    s3 = s3 / norm(s3);

    %% ===== 初始化阶段 =====
    if nargin < 3 || isempty(s5_prev)
                s4 = s4 / norm(s4);
        % 只在第一帧使用 s4 来确定唯一初始方向
                s4 = s4 - dot(s4, s3) * s3;
        if norm(s4) < 1e-8
            error('Initialization failed: s4 parallel to s3.');
        end
        s4 = s4 / norm(s4);

        omega_si5 = cross(s3, s4);
        if norm(omega_si5) < 1e-8
            error('Initialization failed: s3 and s4 are nearly parallel.');
        end

        omega_si5 = omega_si5 / norm(omega_si5);
        return;
    end

    %% ===== 连续更新阶段（Parallel Transport） =====
    % 上一帧的主轴（由正交关系反推）
    s3_prev = cross(s5_prev, cross(s3, s5_prev));
    s3_prev = s3_prev / norm(s3_prev);

    % 若主轴几乎未变化，直接继承
    if norm(cross(s3_prev, s3)) < 1e-10
        omega_si5 = s5_prev;
        return;
    end

    % ---------- 最小旋转 ----------
    k = cross(s3_prev, s3);
    k = k / norm(k);

    theta = acos(max(-1, min(1, dot(s3_prev, s3))));

    K = [   0    -k(3)  k(2);
          k(3)    0   -k(1);
         -k(2)  k(1)    0  ];

    R = eye(3) + sin(theta)*K + (1-cos(theta))*(K*K);

    % ---------- 并行移动 ----------
    omega_si5 = R * s5_prev;

    % ---------- 数值正交修正 ----------
    omega_si5 = omega_si5 - dot(omega_si5, s3) * s3;
    omega_si5 = omega_si5 / norm(omega_si5);

end