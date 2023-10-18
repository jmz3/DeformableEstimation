clear;

lw = 3;

prev_x = [-7 -4 -2 -1 0];
prev_y = [.5 -0.6 0.1 0.2 0];

prev_xq = -7:0.1:0;
prev_yq = spline(prev_x,prev_y,prev_xq);

vec_tangent = [prev_xq(71) - prev_xq(70); prev_yq(71) - prev_yq(70) ];



plot(prev_xq, prev_yq,'Color',"#F7903D",'LineWidth',lw)
hold on
plot([0,20*vec_tangent(1)], [0,20*vec_tangent(2)],...
    'LineWidth',lw,'LineStyle',':',...
    'Marker','.',"MarkerIndices",1,'MarkerSize',20,'Color',"#F7903D")

r = 2;
r_x = - r / sqrt(1 + (vec_tangent(1)/vec_tangent(2))^2) ;
r_y = - r_x * vec_tangent(1)/vec_tangent(2);

viscircles([r_x, r_y],r, 'LineStyle','--','Color','#59A95A','LineWidth',1);

k =  -r_y / -r_x;
D_x = linspace(-1.3,0,10);
D_y = k*D_x;

plot(D_x,D_y)

h_x = linspace(r_x, r_x + r,10); h_x = [h_x 0];
h_y = r_y * ones(1,10); h_y(11) = 0;

plot(h_x,h_y,'Color','#4D85BD','Marker','.','MarkerIndices',1,'MarkerEdgeColor','auto',...
    "MarkerSize",20)

arc_x = linspace(0,r_x+r,100);
arc_y = sqrt(r^2 - (arc_x-r_x).^2) + r_y;

plot(arc_x,arc_y,'LineWidth',lw,'Color','#59A95A',...
    'MarkerIndices',100, 'LineStyle','--',...
    'MarkerSize',20,'Marker','.','MarkerEdgeColor','#59A95A',...
    'MarkerFaceColor','#59A95A')

axis equal
box off
axis off