
start_point = [445.28 -257.41 -1221.45];
n = [120.86 14.73 -1381.89] - [ 109.28 57.02 -1406.65];
point = [
    350.51 -100.90 -1289.20;
    177.52 -0.34 -1326.52;
    18.38 12.51 -1456.44;
    -0.71 -119.10 -1600.27;
    9.38 -272.83 -1548.32;
    -76.45 -351.36 -1388.70;
    -203.97 -363.55 -1239.21;
    -366.53 -339.60 -1101.56];
iter_count = 0;
center = start_point;
point_c = point - repmat(center, size(point,1), 1);
% generate axis direction for quaternion
% axis = 0.3*rand(1,3);
end_position = [];
while true
axis = [ 1 1 1];
iter_count = iter_count + 1;
angle = iter_count * 0.001;

q_rand = quaternion([angle axis]);
RM = rotmat(q_rand,'frame');
point_c = (RM*point_c')';
point_c  = point_c + 10*rand(size(point_c,1),size(point_c,2));

point_rand_rot = [ start_point ; point_c + repmat(center,size(point,1),1)];
plot3( point_rand_rot(:,1) , point_rand_rot(:,2) , point_rand_rot(:,3))
end_position = [end_position ; point_rand_rot(8,:)];
hold on
pause(0.1)
end
title("Sorted Curve")