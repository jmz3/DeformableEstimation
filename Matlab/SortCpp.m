clear sub_raw
pause(1)
sub_raw = rossubscriber("/cpp_test","geometry_msgs/PoseArray");
disp("Listening...");
input = receive(sub_raw,10);
for k = 1:size(input.Poses)
    poseMsg = input.Poses(k);
    p(k,1) = poseMsg.Position.X;
    p(k,2) = poseMsg.Position.Y;
    p(k,3) = poseMsg.Position.Z;
end

open = p;


Inter_p= linspace(1,size(open,1),1000); %create a array of points within the length of the data
xx=spline(1:size(open,1),open(:,1),Inter_p); % use points created above to interpolate x,y,and z
yy=spline(1:size(open,1),open(:,2),Inter_p);
zz=spline(1:size(open,1),open(:,3),Inter_p);
subplot(1,3,3)
plot3(xx,yy,zz,'LineWidth',2)
hold on
plot3(start_point(1), start_point(2), start_point(3),...
    "Marker","o","MarkerSize",5,...
    "LineWidth",2)
hold off
xlim([-500 500])
ylim([-400 100])
zlim([-1700 -1000])
title("Sorted Curve")
view(135,25)

