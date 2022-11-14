clear
rosinit
node_sub = robotics.ros.Node('Node_sub');
start_point = [445.28 -257.41 -1221.45];
% subscribe the sorted and interpolated data here
sub_test = rossubscriber("/cpp_test","geometry_msgs/PoseArray");
figure(1)
set(gcf,'Position',[1300 300 1000 900])

while true
input = receive(sub_test,10);
count = size(input.Poses,1);
p = zeros(count,3);
for k = 1:count
    poseMsg = input.Poses(k);
    p(k,1) = poseMsg.Position.X;
    p(k,2) = poseMsg.Position.Y;
    p(k,3) = poseMsg.Position.Z;
end

Inter_p= linspace(1,size(p,1),1000); %create a array of points within the length of the data
xx=spline(1:size(p,1),p(:,1),Inter_p); % use points created above to interpolate x,y,and z
yy=spline(1:size(p,1),p(:,2),Inter_p);
zz=spline(1:size(p,1),p(:,3),Inter_p);

plot3(xx,yy,zz,'LineWidth',2)
hold on
plot3(start_point(1), start_point(2), start_point(3),...
    "Marker","o","MarkerSize",5,...
    "LineWidth",2)
hold off
xlim([-1500 1500])
ylim([-1400 1100])
zlim([-2000 -1000])
title("Interpolated Curve")
view(135,25)
pause(0.1);
end