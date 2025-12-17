clear ;clc;
%将主轴看作一个点（材料力学：长径比）
% Screw=[Pos_q,vec_s,h];
% V=[Omega;......
%     velocity];
% Omega=(vec_s)*dot_Q;
% velocity=(-cross(vec_s,q)+h*vec_s)*dot_Q;
% %skew-symmetric(用于计算叉乘）
% ss_x=-x';%cross(a,b)=ss_a*b

%建立基座标系，建立初始位置时，主轴抽象出来的点相对于其的齐次变换矩阵
T_01=[1,0,0,0;...
    0,1,0,0;...
    0,0,1,-5;...
    0,0,0,1];
T_12=[-1/2,-sqrt(3)/2,0,0;...
    sqrt(3)/2,-1/2,0,0;...
    0,0,1,0;...
    0,0,0,1];%旋转120度
T_13=[-1/2,sqrt(3)/2,0,0;...
    -sqrt(3)/2,-1/2,0,0;...
    0,0,1,0;...
    0,0,0,1];%旋转240度
%% 

%建立链路的运动旋量
%定义端点坐标
p.s1=[4;0;0];
p.s2=transform_Point_vector(T_12,[4;0;0]);
p.s3=transform_Point_vector(T_13,[4;0;0]);
p.b1=[1;0;0];
p.b2=[1;0;0];
p.b3=[1;0;0];
% 参数

%链路1的关节
S1(5) = struct('i',[],'s_hat',[],'ri',[],'nu',[],'xi',[]);
%旋转副
S1(1).i = 1; 
S1(1).s_hat =rotate_vector(T_01,[0;1;0])/norm(rotate_vector(T_01,[0;1;0]));
S1(1).ri=transform_Point_vector(T_01,p.b1); 
S1(1).nu =-vector_to_skew_symmetric(S1(1).s_hat)*S1(1).ri ;
S1(1).xi=[S1(1).s_hat;...
    S1(1).nu];%旋转副的旋量

%移动副
S1(2).i = 2; 
S1(2).s_hat = (S1(1).ri-p.s1)/norm(S1(1).ri-p.s1);
S1(2).ri = p.s1;
S1(2).nu = S1(2).s_hat;
S1(2).xi=[0;0;0;S1(2).nu];%移动副的旋量

%球铰
S1(3).i = 3; 
S1(3).s_hat =S1(2).nu ; 
S1(3).ri =p.s1;
S1(3).nu =-vector_to_skew_symmetric(S1(3).s_hat)*S1(3).ri ;%Z轴，沿着移动副
S1(3).xi=[S1(3).s_hat;...
    S1(3).nu];%旋转副的旋量

[s_hat_s14, s_hat_s15]=find_orthogonal_vectors(S1(3).s_hat);

S1(4).i = 4; S1(4).s_hat = s_hat_s14; S1(4).ri =p.s1;
S1(4).nu =-vector_to_skew_symmetric(S1(4).s_hat)*S1(4).ri ;
S1(4).xi=[S1(4).s_hat;...
    S1(4).nu];%旋转副的旋量


S1(5).i = 5; S1(5).s_hat = s_hat_s15; S1(5).ri =p.s1;
S1(5).nu =-vector_to_skew_symmetric(S1(5).s_hat)*S1(5).ri ;
S1(5).xi=[S1(5).s_hat;...
    S1(5).nu];%旋转副的旋量
%% 


%链路2的关节
S2(5) = struct('i',[],'s_hat',[],'ri',[],'nu',[]);
%旋转副
S2(1).i = 1; 
S2(1).s_hat =rotate_vector(T_01*T_12,[0;1;0])/norm(rotate_vector(T_01*T_12,[0;1;0]));
S2(1).ri=transform_Point_vector(T_01*T_12,p.b2); 
S2(1).nu =-vector_to_skew_symmetric(S2(1).s_hat)*S2(1).ri ;
S2(1).xi=[S2(1).s_hat;...
    S2(1).nu];%旋转副的旋量

%移动副
S2(2).i = 2; 
S2(2).s_hat = (S2(1).ri-p.s2)/norm(S2(1).ri-p.s2);
S2(2).ri = p.s2;
S2(2).nu = S2(2).s_hat;
S2(2).xi=[0;0;0;S2(2).nu];%移动副的旋量

%球铰
S2(3).i = 3; 
S2(3).s_hat =S2(2).nu ; 
S2(3).ri =p.s2;
S2(3).nu =-vector_to_skew_symmetric(S2(3).s_hat)*S2(3).ri ;%Z轴，沿着移动副
S2(3).xi=[S2(3).s_hat;...
    S2(3).nu];%旋转副的旋量
[s_hat_S24, s_hat_S25]=find_orthogonal_vectors(S2(3).s_hat);

S2(4).i = 4; S2(4).s_hat = s_hat_S24; S2(4).ri =p.s2;
S2(4).nu =-vector_to_skew_symmetric(S2(4).s_hat)*S2(4).ri ;
S2(4).xi=[S2(4).s_hat;...
    S2(4).nu];%旋转副的旋量
S2(5).i = 5; S2(5).s_hat = s_hat_S25; S2(5).ri =p.s2;
S2(5).nu =-vector_to_skew_symmetric(S2(5).s_hat)*S2(5).ri ;
S2(5).xi=[S2(5).s_hat;...
    S2(5).nu];%旋转副的旋量
%% 


%链路3的关节
S3(5) = struct('i',[],'s_hat',[],'ri',[],'nu',[]);
%旋转副
S3(1).i = 1; 
S3(1).s_hat =rotate_vector(T_01*T_13,[0;1;0])/norm(rotate_vector(T_01*T_13,[0;1;0]));
S3(1).ri=transform_Point_vector(T_01*T_13,p.b3); 
S3(1).nu =-vector_to_skew_symmetric(S3(1).s_hat)*S3(1).ri ;
S3(1).xi=[S3(1).s_hat;...
    S3(1).nu];%旋转副的旋量
%移动副
S3(2).i = 2; 
S3(2).s_hat = (S3(1).ri-p.s3)/norm(S3(1).ri-p.s3);
S3(2).ri = p.s3;
S3(2).nu = S3(2).s_hat;
S3(2).xi=[0;0;0;S3(2).nu];%移动副的旋量

%球铰
S3(3).i = 3; 
S3(3).s_hat =S3(2).nu ; 
S3(3).ri =p.s3;
S3(3).nu =-vector_to_skew_symmetric(S3(3).s_hat)*S3(3).ri ;%Z轴，沿着移动副
S3(3).xi=[S3(3).s_hat;...
    S3(3).nu];%旋转副的旋量
[s_hat_S34, s_hat_S35]=find_orthogonal_vectors(S3(3).s_hat);

S3(4).i = 4; S3(4).s_hat = s_hat_S34; S3(4).ri =p.s3;
S3(4).nu =-vector_to_skew_symmetric(S3(4).s_hat)*S3(4).ri ;
S3(4).xi=[S3(4).s_hat;...
    S3(4).nu];%旋转副的旋量

S3(5).i = 5; S3(5).s_hat = s_hat_S35; S3(5).ri =p.s3;
S3(5).nu =-vector_to_skew_symmetric(S3(5).s_hat)*S3(5).ri ;
S3(5).xi=[S3(5).s_hat;...
    S3(5).nu];%旋转副的旋量
