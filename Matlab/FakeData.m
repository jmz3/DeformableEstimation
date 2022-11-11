% generate random rotation in the start_point frame
center = start_point;
point_c = point - repmat(center, size(point,1), 1);
q_rand = quaternion([ 0.005 rand(1,3)]);
RM = rotmat(q_rand,'frame');
point_c = (RM*point_c')';
point_c  = point_c + 5*rand(size(point_c,1),size(point_c,2));

point_rand_rot = [ start_point ; point_c + repmat(center,size(point,1),1)];
plot3( point_rand_rot(:,1) , point_rand_rot(:,2) , point_rand_rot(:,3))
hold on
pause(0.5)


normalMsg.Position.X = n(1);
normalMsg.Position.Y = n(2);
normalMsg.Position.Z = n(3);
send(pub_fake_normal, normalMsg);

for k = 1:size(point_rand_rot,1)
    posearray.Poses(k) = rosmessage("geometry_msgs/Pose");
    posearray.Poses(k).Position.X = point_rand_rot(k,1);
    posearray.Poses(k).Position.Y = point_rand_rot(k,2);
    posearray.Poses(k).Position.Z = point_rand_rot(k,3);
end
send(pub_fake_data,posearray);
disp('sending')
