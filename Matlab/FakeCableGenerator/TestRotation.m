x_1 = [ sqrt(2)/2 ; -sqrt(2)/2 ];
y_1 = [ sqrt(2)/2 ; sqrt(2)/2 ];

o_0 = [ 0 ; 0 ];
x_0 = [ 1 ; 0 ];
y_0 = [ 0 ; 1 ];

figure 
hold on
axis equal
plot( [ 0 x_0(1) ] , [ 0 x_0(2)] , 'b')
% plot( [ 0 y_0(1) ] , [ 0 y_0(2)] , 'b')

R = [ x_1 , y_1 ];

New = R*[ x_0 , y_0 ];

x_0 = New(:,1);
y_0 = New(:,2);

plot( [ 0 x_0(1) ] , [ 0 x_0(2)] , 'r')
% plot( [ 0 y_0(1) ] , [ 0 y_0(2)] , 'r')

R = [ x_1 , y_1 ];

n = [ 1 ; 1 ];

n = R*n;

plot( [ 0 n(1) ] , [ 0 n(2)] , 'k')