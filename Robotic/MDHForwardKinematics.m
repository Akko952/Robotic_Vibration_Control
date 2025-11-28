% %Build Robot by D_H methods
function Link = MDHForwardKinematics(Posture)
% %global Link      %声明全局变量
theta1=Posture(1);
theta2=Posture(2);
theta3=Posture(3);
d4=Posture(4);
theta5=Posture(5);
theta6=Posture(6);
theta7=Posture(7);

ToRad = pi/180;
az = [0 0 1]';

% 创建（Body, Base, J1..J7）
Link = struct('Name','Body','alpha',0,'a',0,'d',0,'theta',0,'az',az);
Link(1) = struct('Name','Base', 'alpha', 0 * ToRad,    'a', 0,   'd', 0,   'theta', 0 * ToRad,  'az', az);  % 0,0,0,0
Link(2) = struct('Name','J1',   'alpha', 0 * ToRad,    'a', 0,   'd', 100,   'theta', 0 * ToRad,  'az', az);  % 0,0,0,0
Link(3) = struct('Name','J2',   'alpha', 90 * ToRad,   'a', 0,   'd', 0,   'theta', 90 * ToRad, 'az', az);  % 90,0,0,90
Link(4) = struct('Name','J3',   'alpha', 0 * ToRad,    'a', 260, 'd', 0,   'theta', 0 * ToRad,  'az', az);  % 0,260,0,0
Link(5) = struct('Name','J4',   'alpha', 90 * ToRad,   'a', 0,   'd', 480, 'theta', 0 * ToRad,  'az', az);  % 90,0,480,0
Link(6) = struct('Name','J5',   'alpha', 0 * ToRad,    'a', 0,   'd', 520, 'theta', 90 * ToRad, 'az', az);  % 0,0,520,90
Link(7) = struct('Name','J6',   'alpha', 90 * ToRad,   'a', 30,   'd', 0,   'theta', 90 * ToRad, 'az', az);  % 90,0,0,90
Link(8) = struct('Name','J7',   'alpha', 90 * ToRad,   'a', 0,   'd', 0,   'theta', 0 * ToRad,  'az', az);  % 90,0,0,0
Link(9) = struct('Name','End',  'alpha', 0 * ToRad,    'a', 0,   'd', 0,   'theta', 0 * ToRad,  'az', az);  % 0,0,0,0

% 打印确认
fprintf('Created Link struct array with %d elements:\n', numel(Link));
for k = 1:numel(Link)
    fprintf('  %2d: %s  alpha=%.3f deg  a=%.1f  d=%.1f  theta=%.3f deg\n', ...
        k, Link(k).Name, Link(k).alpha/ToRad, Link(k).a, Link(k).d, Link(k).theta/ToRad);
end



Link(2).theta = (theta1)*pi/180;
Link(3).theta = (theta2+90)*pi/180;
Link(4).theta = (theta3)*pi/180;
Link(5).d = d4+480;
Link(6).theta = (theta5+90)*pi/180;
Link(7).theta = (theta6+90)*pi/180;
Link(8).theta = (theta7)*pi/180;


for i = 1:9
    Ct = cos(Link(i).theta);
    St = sin(Link(i).theta);
    Ca = cos(Link(i).alpha);
    Sa = sin(Link(i).alpha);
    a = Link(i).a;
    d = Link(i).d;

    Link(i).n=[ Ct;    St*Ca;     St*Sa;      0];
    Link(i).o=[-St;    Ct*Ca;     Ct*Sa;      0];
    Link(i).a=[  0;   -Sa;        Ca;         0];
    Link(i).p=[  a;   -Sa*d;      Ca*d;       1];

    Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
    Link(i).A=[Link(i).n,Link(i).o,Link(i).a,Link(i).p];
end

for i=2:9
    Link(i).A = Link(i-1).A*Link(i).A;
    Link(i).p = Link(i).A(:,4);
    Link(i).n = Link(i).A(:,1);
    Link(i).o = Link(i).A(:,2);
    Link(i).a = Link(i).A(:,3);
    Link(i).R = [Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
end

end
