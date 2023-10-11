function []=cable_sim(nodes)

x = [ 0 100 200];
y = [ 0 100 200];
z = [ 0 50 500];
xyz = [x ; y ; z ];

curve = cscvn(xyz);

fnplt(curve, 'r', 2);


hold on
t = linspace(1,40,200);
t = [t;t];
v = fnval(curve,t);
 
plot3(v(1,:), v(2,:) , v(3,:));
plot3(x,y,z,'x');

end
