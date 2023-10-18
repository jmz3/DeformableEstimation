clear;
clc;

syms phi theta;
rho = 2*cos(theta)/(pi - 2*theta);

x = rho*sin(phi)*cos(theta);
y = rho*sin(phi)*sin(theta);
z = rho*cos(phi);
f = fsurf(x,y,z,[-pi/2 pi/2 0 pi]);
f.FaceColor = [0.8 0.8 0.8];
f.DisplayName = "Feasible Surface";
box on
hold on

xl = [0 0];
yl = [0 0];
zl = [-1 0];
line(xl,yl,zl, 'Color','k','LineWidth',5, 'DisplayName', "Previous Line Segment")

scatter3([0.9 0.8 -0.7 0],[0.9 -1.0 0.2 -0.75],[0.9 0.88 0.94 0.7],300,'.','red', 'DisplayName', "Optical Tracker Readings")
fontsize(gcf,16,'points')

legend()