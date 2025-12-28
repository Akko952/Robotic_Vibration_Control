function [d, passive_angles] = inverse_kinematics(T_des, p)
    % inverse_kinematics 逆运动学求解
    %
    % 输入:
    %   T_des          : 期望的动平台齐次变换矩阵 (4x4)
    %   p              : 包含 p.s1, p.s2, p.s3 的结构体 (静平台球铰，世界坐标)
    %   r_b : 包含 r1, r2, r3 的结构体 (动平台转动副)
    %
    % 输出:
    %   d              : [d1; d2; d3] 主动关节伸缩长度
    
    % 1. 提取静平台固定点 (S副, 世界坐标系)
    s1_world = p.s1;
    s2_world = p.s2;
    s3_world = p.s3;
    
    % 2. 提取动平台连接点 (R副, 局部坐标系)
    % 这些点是相对于动平台中心的固定几何参数
    r1_local = p.b1;
    r2_local = p.b2;
    r3_local = p.b3;
    
    % 3. 计算动平台连接点在当前目标位姿下的世界坐标
    % Formula: P_world = T_des * P_local
    r1_current = transform_Point_vector(T_des, r1_local);
    r2_current = transform_Point_vector(T_des, r2_local);
    r3_current = transform_Point_vector(T_des, r3_local);
    
    % 4. 计算杆长 (距离公式)
    % Vector = r_current (动) - s_world (静)
    vec_1 = r1_current - s1_world;
    vec_2 = r2_current - s2_world;
    vec_3 = r3_current - s3_world;
    
    d1 = norm(vec_1);
    d2 = norm(vec_2);
    d3 = norm(vec_3);
    
    d = [d1; d2; d3];
    
    % (可选) 计算被动关节角度，需要根据具体R副和S副的轴线方向计算
    passive_angles = []; 
end