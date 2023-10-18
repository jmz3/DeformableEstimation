% Find the next point according to the cable constraints
% Input -- points with random order
% Output -- sorted points

DistanceThreshold = 0.1*L;
TempContainer = marker;
figure 
hold on
axis equal

dx = diff(x);
dy = diff(y);

theta_r = pi/4:0.01:pi/2;
rho_r = 2*L*cos(theta_r)./(pi - 2*theta_r);
theta_l = pi/2:0.01:3*pi/4;
rho_l = 2*L*cos(pi - theta_l)./(2*theta_l-pi);
theta = [ theta_r , theta_l];
rho = [ rho_r , rho_l ]; % derive the total rho vs. theta locally

direction = [ dx(1) ; dy(1) ]/norm([ dx(1) ; dy(1) ]); % y-axis for point 1
R_0 = [ cos(-pi/2) -sin(-pi/2) ; sin(-pi/2) cos(-pi/2) ];
y_axis = direction;
x_axis = R_0*y_axis;
% plot([0 y_axis(1)], [0 y_axis(2)])
% hold on
% plot([0 x_axis(1)], [0 x_axis(2)])
% axis equal

R = [ x_axis , y_axis]; % change frame from local frame to global frame

x_local = rho.*cos(theta);
y_local = rho.*sin(theta);

global_coord = R* [ x_local ; y_local ] + [ x(1)*ones(1,length(x_local)) ;  y(1)*ones(1,length(x_local))];
% plot(global_coord(1,:) , global_coord(2,:),'r','LineWidth',3, 'DisplayName','Feasible Band')

label = cell(segments+1,1);

for i = 1:1:segments
    mi = marker_idx(i);
    direction = [ dx(mi) ; dy(mi) ]/norm([ dx(mi) ; dy(mi) ]); % y-axis for point 1
R_0 = [ cos(-pi/2) -sin(-pi/2) ; sin(-pi/2) cos(-pi/2) ];
y_axis = direction;
x_axis = R_0*y_axis;
% plot([0 y_axis(1)], [0 y_axis(2)])
% hold on
% plot([0 x_axis(1)], [0 x_axis(2)])
% axis equal

R = [ x_axis , y_axis]; % change frame from local frame to global frame

x_local = rho.*cos(theta);
y_local = rho.*sin(theta);

global_coord = R* [ x_local ; y_local ] + [ x(mi)*ones(1,length(x_local)) ;  y(mi)*ones(1,length(x_local))];
if i<segments
    plot(global_coord(1,:) , global_coord(2,:),'b','LineWidth',3)
    label{i,1} = '';
else
    plot(global_coord(1,:) , global_coord(2,:),'b','LineWidth',3,'DisplayName','Feasible Band')
    label{i,1} = 'Feasible Band';
end

end

plot(marker(1, :), marker(2, :), 'rx','LineWidth',1,'MarkerSize',15,'DisplayName','Marker Readings')
% plot(x,y,'Color','green','LineStyle','-','LineWidth',2)
label{segments+1,1} = 'Marker Readings';
legend(label,'FontSize',16)
box on
xlabel('x/m')
ylabel('y/m')
% hold on 
% for n = 1:50:length(x)-1
% 
% plot([x(n) , x(n)+100*dx(n)],[y(n), y(n)+100*dy(n)] )
% hold on 
% end