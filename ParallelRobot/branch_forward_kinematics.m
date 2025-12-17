% 对于第i个分支，假设已定义旋量：
% xi_si1, xi_si2, xi_si3 - 球铰的三个旋量
% xi_pi - P副旋量（主动）
% xi_ri - R副旋量

% 分支运动链的正向运动学
function T_branch = branch_forward_kinematics(theta_s1, theta_s2, theta_s3, d_i, phi_i)
    % 计算分支末端的位姿（从静平台到动平台连接点）
    
    T = eye(4);  % 初始为单位矩阵
    
    % 注意：旋量的顺序应与实际运动链一致
    % 通常从静平台开始：S → P → R
    
    % 球铰的三个转动
    T = T * trans_exp_screw(xi_si1, theta_s1);
    T = T * trans_exp_screw(xi_si2, theta_s2);
    T = T * trans_exp_screw(xi_si3, theta_s3);
    
    % 移动关节（主动）
    T = T * trans_exp_screw(xi_pi, d_i);
    
    % 转动关节（被动）
    T = T * trans_exp_screw(xi_ri, phi_i);
    
    T_branch = T;
end