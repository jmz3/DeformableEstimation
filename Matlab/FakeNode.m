rosinit
node_pub = robotics.ros.Node('node_pub');

% Fake Data here 
point = [
    350.51 -100.90 -1289.20;
    177.52 -0.34 -1326.52;
    18.38 12.51 -1456.44;
    -0.71 -119.10 -1600.27;
    9.38 -272.83 -1548.32;
    -76.45 -351.36 -1388.70;
    -203.97 -363.55 -1239.21;
    -366.53 -339.60 -1101.56];
start_point = [445.28 -257.41 -1221.45];
n = [120.86 14.73 -1381.89] - [ 109.28 57.02 -1406.65];

pub_fake_data = rospublisher("/NDI/measured_cp_array","geometry_msgs/PoseArray");
pub_fake_normal = rospublisher("/Normal_vec","geometry_msgs/Pose");
posearray = rosmessage(pub_fake_data);
normalMsg = rosmessage(pub_fake_normal);


while true
    FakeData
end
