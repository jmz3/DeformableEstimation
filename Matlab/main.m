clear;
clc;
rosinit
pub_raw = rospublisher("Raw_data","geometry_msgs/PoseArray");
pub_fit = rospublisher("Fit_data","geometry_msgs/PoseArray");
sub_raw = rossubscriber("/Raw_data","geometry_msgs/PoseArray");
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

figure(1)
subplot(1,3,1)

temp = [start_point;point];
plot3(temp(:,1), temp(:,2), temp(:,3),"LineStyle","--")
axis([ -500 500 -400 100 -1600 -1100])
hold on
plot3(start_point(1), start_point(2), start_point(3),...
    "Marker","o","MarkerSize",5,...
    "LineWidth",2)
title("Ground Truth")
view(135,25)
text(temp(:,1), temp(:,2),temp(:,3), [repmat('  ',size(temp,1),1), num2str((1:size(temp,1))')])
while true

    DataVisualization
    SortCpp
    %     MATLABROS
end

