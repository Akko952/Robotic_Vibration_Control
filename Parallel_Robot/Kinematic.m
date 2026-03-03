clear ;clc;
%初始化运动学
%将主轴看作一个点（材料力学：长径比）
% Screw=[Pos_q,vec_s,h];
% 【twist】V=[Omega;......
%     velocity;
% Omega=(vec_s)*dot_Q;
% velocity=(-cross(vec_s,q)+h*vec_s)*dot_Q;
% %skew-symmetric(用于计算叉乘）
% ss_x=-x';cross(a,b)=ss_a*b

%建立基座标系，建立初始位置时，主轴抽象出来的点相对于其的齐次变换矩阵
T_01=[1,0,0,0;...
    0,1,0,0;...
    0,0,1,-5;...
    0,0,0,1];%基座坐标系到动平台坐标系的齐次变换矩阵
T_12=[-1/2,-sqrt(3)/2,0,0;...
    sqrt(3)/2,-1/2,0,0;...
    0,0,1,0;...
    0,0,0,1];%旋转120度的算子
T_13=[-1/2,sqrt(3)/2,0,0;...
    -sqrt(3)/2,-1/2,0,0;...
    0,0,1,0;...
    0,0,0,1];%旋转240度的算子
%% 

%建立链路的运动旋量，其结果全部归一化，并且转化到世界坐标系下
%得到的结果为关节旋量基

%定义端点坐标
%psi为世界坐标（静平台坐标系）
p.s1=[5*tand(30)+1;0;0];%链路1与静平台的连接点坐标
p.s2=transform_Point_vector(T_12,[5*tand(30)+1;0;0]);%链路2与静平台的连接点坐标，直接将链路1的坐标通过齐次变换矩阵转换得到
p.s3=transform_Point_vector(T_13,[5*tand(30)+1;0;0]);%链路3与静平台的连接点坐标
%pbi为动平台坐标系
p.b1=[1;0;0];%链路1与动平台的连接点坐标 
p.b2=transform_Point_vector(T_12,[1;0;0]);%链路2与动平台的连接点坐标
p.b3=transform_Point_vector(T_13,[1;0;0]);%链路3与动平台的连接点坐标
% function []=model_limb_jopint_screw()

% end
% 参数初始化-
S1(5).i = 5; 
S1(5).s_hat =rotate_vector(T_01,[0;1;0])/norm(rotate_vector(T_01,[0;1;0]));%与动平台连接处的旋转副轴为动坐标系的Y轴
% %通过旋转算子变成世界坐标系，由于不在意转轴的位置，只在意方向，因此只需要旋转【可能】
S1(5).ri=transform_Point_vector(T_01,p.b1); %转轴的位置为动平台与链路连接处，抽象出来一点,变换成世界坐标系
%关节旋量定义完成，开始组装TWIST
S1(5).nu =-vector_to_skew_symmetric(S1(5).s_hat)*S1(5).ri ;%叉乘算法，计算线速度部分
S1(5).xi=[S1(5).s_hat;...
    S1(5).nu];%旋转副的瞬时速度旋量

%移动副
S1(4).i = 4; 
S1(4).s_hat = (S1(5).ri-p.s1)/norm(S1(5).ri-p.s1);
S1(4).ri = p.s1;
S1(4).nu = S1(4).s_hat;
S1(4).xi=[0;0;0;S1(4).nu];%移动副的瞬时速度旋量

%球铰
S1(3).i = 3; 
S1(3).s_hat =S1(4).nu ; 
S1(3).ri =p.s1;
S1(3).nu =-vector_to_skew_symmetric(S1(3).s_hat)*S1(3).ri ;%Z轴，沿着移动副
S1(3).xi=[S1(3).s_hat;...
    S1(3).nu];%旋转副的TWIST旋量



S1(2).i = 2; S1(2).s_hat = S1(5).s_hat; S1(2).ri =p.s1;
S1(2).nu =-vector_to_skew_symmetric(S1(2).s_hat)*S1(2).ri ;
S1(2).xi=[S1(2).s_hat;...
    S1(2).nu];%旋转副的TWIST旋量

    % [s_hat_s15]=find_orthogonal_vectors_inition(S1(3).s_hat,S1(4).s_hat);

S1(1).i = 1; S1(1).s_hat = find_orthogonal_vectors_inition(S1(3).s_hat,S1(2).s_hat); S1(1).ri =p.s1;
S1(1).nu =-vector_to_skew_symmetric(S1(1).s_hat)*S1(1).ri ;
S1(1).xi=[S1(1).s_hat;...
    S1(1).nu];%旋转副的旋量
%% 


%链路2的关节
S2(5) = struct('i',[],'s_hat',[],'ri',[],'nu',[]);
%旋转副
S2(5).i = 5; 
S2(5).s_hat =rotate_vector(T_01*T_12,[0;1;0])/norm(rotate_vector(T_01*T_12,[0;1;0]));
S2(5).ri=transform_Point_vector(T_01,p.b2); 
S2(5).nu =-vector_to_skew_symmetric(S2(5).s_hat)*S2(5).ri ;
S2(5).xi=[S2(5).s_hat;...
    S2(5).nu];%旋转副的TWIST旋量

%移动副
S2(4).i = 4; 
S2(4).s_hat = (S2(5).ri-p.s2)/norm(S2(5).ri-p.s2);
S2(4).ri = p.s2;
S2(4).nu = S2(4).s_hat;
S2(4).xi=[0;0;0;S2(4).nu];%移动副的TWIST旋量

%球铰
S2(3).i = 3; 
S2(3).s_hat =S2(4).nu ; 
S2(3).ri =p.s2;
S2(3).nu =-vector_to_skew_symmetric(S2(3).s_hat)*S2(3).ri ;%Z轴，沿着移动副
S2(3).xi=[S2(3).s_hat;...
    S2(3).nu];%旋转副的TWIST旋量


S2(2).i = 2; S2(2).s_hat = S2(5).s_hat; S2(2).ri =p.s2;
S2(2).nu =-vector_to_skew_symmetric(S2(2).s_hat)*S2(2).ri ;
S2(2).xi=[S2(2).s_hat;...
    S2(2).nu];%旋转副的TWIST旋量

    %[s_hat_S25]=find_orthogonal_vectors_inition(S2(3).s_hat,S2(4).s_hat);

S2(1).i = 1; S2(1).s_hat = find_orthogonal_vectors_inition(S2(3).s_hat,S2(2).s_hat); S2(1).ri =p.s2;
S2(1).nu =-vector_to_skew_symmetric(S2(1).s_hat)*S2(1).ri ;
S2(1).xi=[S2(1).s_hat;...
    S2(1).nu];%旋转副的TWIST旋量
%% 


%链路3的关节
S3(5) = struct('i',[],'s_hat',[],'ri',[],'nu',[]);
%旋转副
S3(5).i = 5; 
S3(5).s_hat =rotate_vector(T_01*T_13,[0;1;0])/norm(rotate_vector(T_01*T_13,[0;1;0]));
S3(5).ri=transform_Point_vector(T_01,p.b3); 
S3(5).nu =-vector_to_skew_symmetric(S3(5).s_hat)*S3(5).ri ;
S3(5).xi=[S3(5).s_hat;...
    S3(5).nu];%旋转副的TWIST旋量
%移动副
S3(4).i = 4; 
S3(4).s_hat = (S3(5).ri-p.s3)/norm(S3(5).ri-p.s3);
S3(4).ri = p.s3;
S3(4).nu = S3(4).s_hat;
S3(4).xi=[0;0;0;S3(4).nu];%移动副的TWIST旋量

%球铰
S3(3).i = 3; 
S3(3).s_hat =S3(4).nu ; 
S3(3).ri =p.s3;
S3(3).nu =-vector_to_skew_symmetric(S3(3).s_hat)*S3(3).ri ;%Z轴，沿着移动副
S3(3).xi=[S3(3).s_hat;...
    S3(3).nu];%旋转副的TWIST旋量


S3(2).i = 2; S3(2).s_hat = S3(5).s_hat; S3(2).ri =p.s3;
S3(2).nu =-vector_to_skew_symmetric(S3(2).s_hat)*S3(2).ri ;
S3(2).xi=[S3(2).s_hat;...
    S3(2).nu];%旋转副的TWIST旋量

    % [s_hat_S35]=find_orthogonal_vectors_inition(S3(3).s_hat,S3(4).s_hat);

S3(1).i = 1; S3(1).s_hat = find_orthogonal_vectors_inition(S3(3).s_hat,S3(2).s_hat); S3(1).ri =p.s3;
S3(1).nu =-vector_to_skew_symmetric(S3(1).s_hat)*S3(1).ri ;
S3(1).xi=[S3(1).s_hat;...
    S3(1).nu];%旋转副的TWIST旋量