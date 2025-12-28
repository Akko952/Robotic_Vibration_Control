function T = trans_exp_screw(xi, theta)
    % 计算旋量ξ对应的指数映射
    % xi: 6×1旋量 [ω; v]（移动关节ω=0）
    % theta: 关节变量（角度或位移）
    
    Omega = xi(1:3);      % 角速度部分 (ω)
    Veloc = xi(4:6);      % 线速度部分 (v)
    
    if norm(Omega) < 1e-10
        % 移动关节（ω = 0）
        % 纯平移：exp(ξθ) = [I, vθ; 0, 0, 0, 1]
        T = [eye(3), Veloc * theta;
             0, 0, 0, 1];
    else
        % 转动关节（ω ≠ 0）
        % 确保ω是单位向量
        Omega_unit = Omega / norm(Omega);
        theta_actual = theta;  % θ已经是实际转角
        
        % 计算叉乘矩阵
        omega_hat = [0, -Omega_unit(3), Omega_unit(2);
                     Omega_unit(3), 0, -Omega_unit(1);
                     -Omega_unit(2), Omega_unit(1), 0];
        
        % 旋转部分：Rodrigues公式
        R = eye(3) + sin(theta_actual) * omega_hat + ...
            (1 - cos(theta_actual)) * (omega_hat * omega_hat);
        
        % 计算平移部分：p = (Iθ + (1-cosθ)ω̂ + (θ-sinθ)ω̂²) * v
        % 注意：这里的v = -ω × q，是旋量的线速度部分
        
        % 方法1：直接使用公式
        G_theta = eye(3) * theta_actual + ...
                  (1 - cos(theta_actual)) * omega_hat + ...
                  (theta_actual - sin(theta_actual)) * (omega_hat * omega_hat);
        p = G_theta * Veloc;
        
        
        T = [R, p;
             0, 0, 0, 1];
    end
end