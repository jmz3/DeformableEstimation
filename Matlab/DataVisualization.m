randIdx = randperm(size(point,1));
point_rand = point(randIdx,:);
point_rand  = point_rand + 5*rand(size(point_rand,1),size(point_rand,2));


subplot(1,3,2)

plot3(start_point(1), start_point(2), start_point(3),"Marker","*")
% plot3(point_rand(:,1), point(:,2), point(:,3),"LineStyle","--")
hold on
point_rand = [start_point;point_rand];
title("Point Cloud with Noise and Random Order ")
plot3(point_rand(:,1), point_rand(:,2), point_rand(:,3),"LineStyle","--","Marker","*")
text(point_rand(:,1), point_rand(:,2),point_rand(:,3), [repmat('  ',size(point_rand,1),1), num2str((1:size(point_rand,1))')])
axis([ -500 500 -400 100 -1600 -1100])
view(135,25)
hold off

pub_raw = rospublisher("Raw_data","geometry_msgs/PoseArray");
posearray = rosmessage(pub_raw);
for k = 1:size(point_rand,1)
    posearray.Poses(k) = rosmessage("geometry_msgs/Pose");
    posearray.Poses(k).Position.X = point_rand(k,1);
    posearray.Poses(k).Position.Y = point_rand(k,2);
    posearray.Poses(k).Position.Z = point_rand(k,3);
end
% Strange bug: In matlab, when you try to push back a pose to the
% pose_array, it looks like the whole posearray is replaced by this pose
% repetitively

% Solved! The component of the posearray.Poses(k) must be decleared every
% time before you want to assign values to its attributes
% Wrong example is as following
% poseMsg = rosmessage("geometry_msgs/Pose") ;
% for k = 1:size(point_rand,1)
    %     poseMsg.Position.X = point_rand(k,1);
    %     poseMsg.Position.Y = point_rand(k,2);
    %     poseMsg.Position.Z = point_rand(k,3);
    %     posearray.Poses(k) = poseMsg;
% end

pub_normal = rospublisher("Normal_vec","geometry_msgs/Pose");
normalMsg = rosmessage(pub_normal);
normalMsg.Position.X = n(1);
normalMsg.Position.Y = n(2);
normalMsg.Position.Z = n(3);
send(pub_normal, normalMsg);
pause(1)

disp("Publishing...")
send(pub_raw, posearray);
pause(1)
