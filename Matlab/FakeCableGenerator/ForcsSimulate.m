% Define the initial position of the robot end-effector
initial_position = [0; 0; 0];

% Define the target position
target_position = [1; 1; 1];

% Define the external force
external_force = [0.0; 0.0; -0.5];

% Define the robot's speed
speed = 0.001;

% Initialize the current position
current_position = initial_position;

% Initialize a vector to store the position error
position_error = [];
traj = [];

% Simulate the robot's movement until it reaches the target
while norm(current_position - target_position) > 0.01
    % Calculate the direction of movement
%     direction = (target_position - current_position) / norm(target_position - current_position);
    direction = target_position - current_position;
    
    % Update the current position
    current_position = current_position + speed * direction + external_force
    
    % Calculate the position error
    error = norm(current_position - target_position);
    
    % Store the position error
    traj = [traj; current_position];
    position_error = [position_error; error];
end

% Plot the position error
figure;
plot(position_error);
title('Position Error');
xlabel('Time step');
ylabel('Position error');
