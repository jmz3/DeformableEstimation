clear sub_raw
pause(1)
sub_raw = rossubscriber("/Raw_data","geometry_msgs/PoseArray");
input = receive(sub_raw,10);
for k = 1:size(input.Poses)
    poseMsg = input.Poses(k);
    p(k,1) = poseMsg.Position.X;
    p(k,2) = poseMsg.Position.Y;
    p(k,3) = poseMsg.Position.Z;
end

p_in = p;
open = p(1,:);
sigma = 50;
dL = 200;
direction = n;

for i=1:1:size(p,1)-1
    possibility = zeros(size(p,1),1);
    P1 = direction(i,:);
    for j=i+1:1:size(p,1)
        P2 = p(j,:)- p(i,:);
        angle = atan2(norm(cross(P1,P2)),dot(P1,P2));
        expect_value =  2 * dL * cos(angle)/( pi - 2 * angle );
        if angle < pi
            possibility(j) = (1/(sigma*sqrt(2*pi)))*exp(-0.5*((norm(P2)-expect_value)/sigma)^2);
        else
            possibility(j) = 0;
        end
    end
    [~, maxidx] = max(possibility);
    p([i+1 maxidx],:) = p([maxidx i+1], :);
    open = [open ; p(i+1,:)];
    P_next = p(i+1,:) - p(i,:);
    normal = cross(P1,P_next)/norm(cross(P1,P_next)); % find the normal to the plane P1xP2
    angle = atan2(norm(cross(P1,P_next)),dot(P1,P_next));
    direction_next = P1*cos(2*angle) + cross(normal,P1)*sin(2*angle) + P1*(dot(P1,normal))*(1-cos(2*angle));
    direction = [direction ; direction_next];
end
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

posearray = rosmessage(pub_fit);
for k = 1:size(point_rand,1)
    posearray.Poses(k) = rosmessage("geometry_msgs/Pose");
    posearray.Poses(k).Position.X = open(k,1);
    posearray.Poses(k).Position.Y = open(k,2);
    posearray.Poses(k).Position.Z = open(k,3);
end
send(pub_fit, posearray);
pause(1)
