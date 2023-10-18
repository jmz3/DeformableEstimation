theta = 0:0.01:pi/2;
L = 1;
rho = 2*L*cos(theta)./(pi - 2*theta);
p1 = polarplot(theta,rho);
p1.LineWidth = 5;
p1.Color = 'b';
hold on
theta = pi/2:0.01:pi;
rho = 2*L*cos(pi - theta)./(2*theta-pi);
p2 = polarplot(theta,rho);
p2.LineWidth = 5;
p2.Color = 'b';
p2.DisplayName = 'Feasible Band';
rlim([0 1.5])
theta = 3*pi/2*ones(1,100);
rho = linspace(0,1.5,100);
p3 = polarplot(theta,rho);
p3.Color = 'k';
p3.LineWidth = 1;
p3.DisplayName = 'Previous Cable Segment';

theta = 0:0.4:pi;
rho = 2*L*cos(pi - theta)./(2*theta-pi) + 0.1*ones([1,length(theta)]) + L/2*rand([1,length(theta)]);
p4 = polarplot(theta,rho,'LineStyle','none','Marker','o','MarkerFaceColor',[255,0,0]/255,'MarkerSize',10);
p4.Color = 'r';