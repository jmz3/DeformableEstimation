%% Bring up ROS
ipaddress = '127.0.0.0';
rosinit(ipaddress,11311)

%% Load the kuka robot
lbr = loadrobot('kukaIiwa7');
lbr.Gravity=[0 0 -9.8];
lbr.DataFormat = 'column';

%% Visualize the robot
show(lbr);
view([150 12]);
axis([-0.6 0.6 -0.6 0.6 0 1.35]);
camva(9);
daspect([1 1 1]);

torque_seq_extf = [];
torque_seq_zero = [];
torque_seq_impe = [];

%% Constant External Force
waypoints = [   0 0 0 0 0 0 0;
                0.5683 0.738 0.8 1.4 1 1 1.75;
                0.365 0.52 0.851 1.656 0.8568 1.0747 1.919];
traj_extf = [];
timePoints = [0 5 10];
tsamples = 0:0.1:10;

[q,qd,qdd,pp ] = quinticpolytraj(waypoints',timePoints,tsamples);


for i=1:1:size(q,2)
show(lbr,q(:,i),'Frames','off','PreservePlot',false);

% end-effector position
T_ee = lbr.getTransform(q(:,i),'iiwa_link_ee_kuka','iiwa_link_0');
traj_extf = [traj_extf; T_ee(1:3,4)'];

view([150 12]);
axis([-0.6 0.6 -0.6 0.6 0 1.35]);
camva(9);
daspect([1 1 1]);
drawnow
end

figure
plot3(traj_extf(:,1),traj_extf(:,2),traj_extf(:,3))

%% Robot moves based on ideal trajectory

waypoints = [   0 0 0 0 0 0 0;
                0.5 0.5 1 1 1 1 1.5;
                0.3 0.4 0.9 1.3 0.9 0.2 2.0];

timePoints = [0 5 10];
tsamples = 0:0.1:10;

[q,qd,qdd,pp ] = quinticpolytraj(waypoints',timePoints,tsamples);

traj_zero = [];

for i=1:1:size(q,2)
show(lbr,q(:,i),'Frames','off','PreservePlot',false);

% end-effector position
T_ee = lbr.getTransform(q(:,i),'iiwa_link_ee_kuka','iiwa_link_0');
traj_zero = [traj_zero; T_ee(1:3,4)'];

view([150 12]);
axis([-0.6 0.6 -0.6 0.6 0 1.35]);
camva(9);
daspect([1 1 1]);
drawnow
end

figure
plot3(traj_zero(:,1),traj_zero(:,2),traj_zero(:,3))

%% Impedance Control with variable load considered
waypoints = [   0 0 0 0 0 0 0;
                0.5683 0.738 0.8 1.4 0.8 0.9 1.75;
                0.3 0.4 0.9 1.3 0.9 0.2 2.0];
acc_boundary = [0 0 0 0 0 0 0;0 -0.1 0.1 -0.1 -0.1 -.1 0.2; 0 0 -0.1 0.0 0.3 0.2 0.1];
timePoints = [0 5 10];
tsamples = 0:0.1:10;

[q,qd,qdd,pp ] = quinticpolytraj(waypoints',timePoints,tsamples,"AccelerationBoundaryCondition",acc_boundary');

traj_impe = [];

for i=1:1:size(q,2)
show(lbr,q(:,i),'Frames','off','PreservePlot',false);

% end-effector position
T_ee = lbr.getTransform(q(:,i),'iiwa_link_ee_kuka','iiwa_link_0');
traj_impe = [traj_impe; T_ee(1:3,4)'];

view([150 12]);
axis([-0.6 0.6 -0.6 0.6 0 1.35]);
camva(9);
daspect([1 1 1]);
drawnow
end

vizPosition(traj_zero,traj_extf,traj_impe)

%% Visualize the joint accleration
for i=1:1:7
plot(tsamples,q(i,:))
hold on
end

%% Inverse Dynamics

fext = externalForce(lbr,'iiwa_link_ee_kuka',[0 0 0 0 0 50]);
for i = 1:1:size(q,2)
joint_torque = inverseDynamics(lbr,q(:,i),qd(:,i),qdd(:,i),fext);
% torque_seq_extf = [torque_seq_extf joint_torque];
% torque_seq_zero = [torque_seq_zero joint_torque];
torque_seq_impe = [torque_seq_impe joint_torque];
end

%% Visualize Torque
vizTorque(torque_seq_extf,tsamples,'Constant External Force Applied')
vizTorque(torque_seq_zero,tsamples,'No External Force Applied')
vizTorque(torque_seq_impe,tsamples,'Variable External Force Applied')
%% Utility Function
vizPosition(traj_zero,traj_extf,traj_impe)

function []=vizPosition(traj_zero,traj_extf,traj_impe)

pose_num = length(traj_zero);

f = figure();
figure(f)
hold on
box on
grid on


p1 = plot3(traj_extf(:,1),traj_extf(:,2),traj_extf(:,3),'LineWidth',1.5);
p2 = plot3(traj_zero(:,1),traj_zero(:,2),traj_zero(:,3),'LineWidth',1.5);
p3 = plot3(traj_impe(:,1),traj_impe(:,2),traj_impe(:,3),'LineWidth',1.5,'MarkerIndices',[1,pose_num]);

points = [  traj_zero(1,1) traj_zero(pose_num,1) traj_extf(pose_num,1);
            traj_zero(1,1) traj_zero(pose_num,2) traj_extf(pose_num,2);
            traj_zero(1,3) traj_zero(pose_num,3) traj_extf(pose_num,3);];

p4 = plot3(points(1,:),points(2,:),points(3,:));

p4.LineStyle = "none";
p4.Marker = "o";
p4.MarkerSize = 5;

p2.LineStyleMode = "auto";
p2.LineStyle = "--";

text(traj_zero(1,1)+.025,traj_zero(1,2)-.025,traj_zero(1,3),'Initial Pose')
text(traj_zero(pose_num,1)+0.025,traj_zero(pose_num,2)+0.025,traj_zero(pose_num,3),'Reference Target Pose')
text(points(1,3),points(2,3)+.1,points(3,3),'Deviated Target Pose')

view([-5 -5 5])
xlabel('X/m')
ylabel('Y/m')
zlabel('Z/m')
fontsize(gca,14,'points')

axis tight
legend({'Constant External Force Applied','Reference Trajctory','Variable External Force Applied',''},...
    'Location','northeast')
title('Trajectory of the Robot EE')
end



%%

function []=vizTorque(torque,time,title_name)
f = figure();
figure(f)
f.Position = [650 300 750  600];
hold on
grid on
box on

for i = 1:1:7
    plot(time,torque(i,:))
end
title(gca,title_name)
xlabel('time/second')
ylabel('Joint Torque/Nm')
ylim([-30 30])
fontsize(gca,14,'points')

legend({'joint 1','joint 2','joint 3','joint 4','joint 5','joint 6','joint 7'})
end