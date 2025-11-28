function frame = Generate_Cyl_and_Linkline(Link,if_clear)
for i=2:9
    DrawLinkLine(Link(i-1).p,Link(i).p);
    hold on;
    DrawCylinder(Link(i-1).p, Link(i-1).R * Link(i).az);
    hold on;
end

grid on;
xlabel('x');
ylabel('y');
zlabel('z');


% 设置坐标范围与视角
axis([-800, 800, -800, 800, -1200, 1200]);
view(-35, 25);

drawnow;
frame = getframe();
if if_clear
    cla;
end
end
%% 

function DrawCylinder(p,az)
% 关节圆柱参数
radius = 10;
len = 30;
col = 0;
side_num = 20;
side_theta = (0:side_num)/side_num * 2*pi;

az0 = [0;0;1];
ax = cross(az0,az);

if norm(ax) < eps
    rot = eye(3);
else
    ax = ax/norm(ax);
    ay = cross(az,ax);
    ay = ay/norm(ay);
    rot = [ax,ay,az];
end

% x=rcost,y=rsint,z=z
x = [radius; radius]* cos(side_theta);
y = [radius; radius] * sin(side_theta);
z = [len/2; -len/2] * ones(1,side_num+1);
cc = col*ones(size(x));

for i=1:size(x,1)
    xyz = [x(i,:);y(i,:);z(i,:)];
    xyz2 = rot * xyz;
    x2(i,:) = xyz2(1,:);
    y2(i,:) = xyz2(2,:);
    z2(i,:) = xyz2(3,:);
end

surf(x2+p(1),y2+p(2),z2+p(3),cc);

for i=1:2
    patch(x2(i,:)+p(1),y2(i,:)+p(2),z2(i,:)+p(3),cc(i,:));
end

end

%% 

function DrawLinkLine(p1,p2)
plot3([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],'Color','b','LineWidth',2);
end