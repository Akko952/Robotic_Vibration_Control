function [T_branch, Ad_all] = branch_forward_kinematics(S_chain, q_values, T_0_home)
    % branch_forward_kinematics 计算单条分支的正向运动学 (POE公式)
    %
    % 输入:
    %   S_chain  : 旋量结构体，包含该分支所有关节的旋量信息
    %   q_values : 向量，对应每个关节的关节变量 [theta_r; d_p; theta_s1; theta_s2; theta_s3]
    %   T_0_home : 零位位姿 
    %
    % 输出:
    %   T_branch : 当前关节变量下的末端齐次变换矩阵
    %   Ad_all   : 6x6xN，每一步累乘变换 T_acc 的伴随矩阵 Ad(T_acc)
    % Adjoint:伴随变化矩阵
    
    % 检查输入维度
    num_joints = length(S_chain);
    if length(q_values) ~= num_joints
        error('关节变量数量与旋量数量不匹配');
    end

    % 类型跟随
    use_sym = isa(q_values(1), 'sym');

    % 累乘指数积
    if use_sym
        T_acc = sym(eye(4));
        % 预分配伴随矩阵输出
        Ad_all = sym(zeros(6, 6, num_joints));
    else
        T_acc = eye(4);
        % 预分配伴随矩阵输出
        Ad_all = zeros(6, 6, num_joints);
    end
    
    for k = num_joints:-1:1
        % 获取第k个关节的旋量 xi (6x1)
        xi = S_chain(k).xi;
        theta = q_values(k);
        Ad_all(:, :, k) = adjoint_transformation(T_acc);% 记录当前累乘到此的伴随矩阵
        % 计算指数映射并累乘
        T_acc = T_acc*trans_exp_screw(xi, theta) ;%顺序为：S==p==R


    end
    
    % 最后乘上零位位姿 M
    T_branch = T_acc * T_0_home;
    
end
%% 计算齐次变换矩阵的伴随变化矩阵
function Ad_T = adjoint_transformation(T)
    % 计算齐次变换矩阵 T 的伴随变化矩阵 Ad(T)
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    p_skew = vector_to_skew_symmetric(p);

    if isa(R, 'sym')
        Z = sym(zeros(3, 3));
    else
        Z = zeros(3, 3);
    end

    Ad_T = [R, Z; p_skew * R, R];
end
%对应的变换
 % trans_exp_screw(S1(5).xi,0)*...
 % trans_exp_screw(S1(4).xi,0)*...
 % trans_exp_screw(S1(3).xi,0)*...
 % trans_exp_screw(S1(2).xi,0)*...
 % trans_exp_screw(S1(1).xi,0)*T_01;