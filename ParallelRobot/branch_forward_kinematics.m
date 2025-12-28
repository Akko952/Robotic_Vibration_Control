function T_branch = branch_forward_kinematics(S_chain, q_values, T_0_home)
    % branch_forward_kinematics 计算单条分支的正向运动学 (POE公式)
    %
    % 输入:
    %   S_chain  : 结构体数组 (如 S1)，包含该分支所有关节的旋量信息
    %   q_values : 向量，对应每个关节的运动量 [theta_r; d_p; theta_s1; theta_s2; theta_s3]
    %   T_0_home : 机械臂在零位时的末端位姿 (通常就是 T_01 或者是动平台相对于基座的变换)
    %
    % 输出:
    %   T_branch : 当前关节变量下的末端齐次变换矩阵
    
    % 检查输入维度
    num_joints = length(S_chain);
    if length(q_values) ~= num_joints
        error('关节变量数量与旋量数量不匹配');
    end

    % 累乘指数积: T = e^(xi1*q1) * ... * e^(xin*qn) * M
    T_acc = eye(4);
    
    for k = 1:num_joints
        % 获取第k个关节的旋量 xi (6x1)
        xi = S_chain(k).xi;
        theta = q_values(k);
        
        % 计算指数映射并累乘
        T_acc = T_acc * trans_exp_screw(xi, theta);
    end
    
    % 最后乘上零位位姿 M
    T_branch = T_acc * T_0_home;
end