tip = [-0.1666, 0.0008, -0.0009];

% w x y z
A = [0.2275, -0.18126, -1.0717];
rotation_matrix_A = quat2rotm([0.0097, 0.44284, -0.0284, 0.896082]);

B = [0.17718, -0.29651, -1.10472];
rotation_matrix_B = quat2rotm([0.05411,0.4465, -0.1106122, 0.8865]);

C = [0.26959, -0.35631, -1.0374];
rotation_matrix_C = quat2rotm([0.0377041, 0.43665, -0.05967, 0.8968]);

D = [0.3211, -0.2468, -1.004];
rotation_matrix_D = quat2rotm([-0.0253, 0.440507, 0.0119, 0.897314]);

E = [0.25043, -0.26069, -1.0527];
rotation_matrix_E = quat2rotm([0.065607, 0.43924726, -0.0729078, 0.893]);

A_new = (rotation_matrix_A * tip' + A')'
B_new = (rotation_matrix_B * tip' + B')'
C_new = (rotation_matrix_C * tip' + C')'
D_new = (rotation_matrix_D * tip' + D')'
E_new = (rotation_matrix_E * tip' + E')'

figure
plot3(A_new(1), A_new(2), A_new(3), ">",  B_new(1), B_new(2), B_new(3), "+", C_new(1), C_new(2), C_new(3), "o", D_new(1), D_new(2), D_new(3), "x");

norm(A_new - B_new)
norm(B_new - C_new)
norm(C_new - D_new)
norm(D_new - A_new)
%sqrt(sum(A-B).^2)
%,  , -270.79, 121.64, -1344.73, "*", -372.89, 140.77, -1401.88, "*", -270.63, 211.989, -1343.830, "*", -278.75, 119.54, -1367.95, "*"