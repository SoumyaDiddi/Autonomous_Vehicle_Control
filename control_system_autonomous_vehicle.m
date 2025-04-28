clc;
clear;
close all;

%% --- Parameters ---
% Vehicle parameters
L = 2.9; % Wheelbase (meters)

% Simulation parameters
dt = 0.1; % Time step (seconds)
T = 30;   % Increased total simulation time
time_steps = T/dt;

% Initial state [x; y; yaw; velocity]
x = [0; 0; 0; 0];

% Reference state [x_ref; y_ref; yaw_ref; velocity_ref]
x_ref = [50; 0; 0; 10];

% Linearized model matrices (around straight motion)
v_nominal = 10.0; % Nominal velocity matches reference

A = [0 0 -v_nominal*sin(0) cos(0);
     0 0  v_nominal*cos(0) sin(0);
     0 0 0 tan(0)/L;
     0 0 0 0];

B = [0 0;
     0 0;
     v_nominal/(L*(cos(0))^2) 0;
     0 1];

% Cost matrices
Q = diag([10, 10, 10, 1]);
R = diag([1, 1]);

% Solve LQR controller
[K, ~, ~] = lqr(A, B, Q, R);

% Initialize storage
history = zeros(4, time_steps+1);
position_errors = zeros(1, time_steps+1);
velocity_errors = zeros(1, time_steps+1);
time_array = linspace(0, T, time_steps+1);

history(:,1) = x;
position_errors(1) = sqrt((x(1) - x_ref(1))^2 + (x(2) - x_ref(2))^2);
velocity_errors(1) = abs(x(4) - x_ref(4));

%% --- Simulation Loop ---
for k = 1:time_steps
    % Calculate error
    error = x - x_ref;
    
    % Control input
    u = -K * error;
    
    % Apply saturations
    max_steering = deg2rad(30); % Max ±30 degrees
    max_accel = 5.0;            % Increased acceleration limit
    u(1) = max(min(u(1), max_steering), -max_steering);
    u(2) = max(min(u(2), max_accel), -max_accel);
    
    % Update state using bicycle model
    delta = u(1);
    a = u(2);
    
    x(1) = x(1) + x(4) * cos(x(3)) * dt;
    x(2) = x(2) + x(4) * sin(x(3)) * dt;
    x(3) = x(3) + (x(4)/L) * tan(delta) * dt;
    x(4) = x(4) + a * dt;
    
    history(:,k+1) = x;
    position_errors(k+1) = sqrt((x(1) - x_ref(1))^2 + (x(2) - x_ref(2))^2);
    velocity_errors(k+1) = abs(x(4) - x_ref(4));
end

%% --- Plot Vehicle Path ---
figure;
plot(history(1,:), history(2,:), 'b', 'LineWidth', 2); hold on;
plot(x_ref(1), x_ref(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Autonomous Vehicle Path using LQR Control');
legend('Vehicle Path', 'Goal');
grid on;
axis equal;

%% --- Print Results in Command Window ---
final_state = history(:, end);

final_x = final_state(1);
final_y = final_state(2);
final_yaw = rad2deg(final_state(3)); % Convert yaw to degrees
final_velocity = final_state(4);

position_error_final = sqrt((final_x - x_ref(1))^2 + (final_y - x_ref(2))^2);
velocity_error_final = abs(final_velocity - x_ref(4));

fprintf('\n--- Autonomous Vehicle Final Results ---\n');
fprintf('Final X Position     : %.2f meters\n', final_x);
fprintf('Final Y Position     : %.2f meters\n', final_y);
fprintf('Final Yaw (Heading)  : %.2f degrees\n', final_yaw);
fprintf('Final Velocity       : %.2f m/s\n', final_velocity);
fprintf('Position Error       : %.2f meters\n', position_error_final);
fprintf('Velocity Error       : %.2f m/s\n', velocity_error_final);

% Check goal achievement
position_tolerance = 1.0; % meters
velocity_tolerance = 0.5; % m/s


%% --- Plot Error Graphs ---
figure;
subplot(2,1,1);
plot(time_array, position_errors, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position Error (m)');
title('Position Error vs Time');
grid on;

subplot(2,1,2);
plot(time_array, velocity_errors, 'g', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Velocity Error (m/s)');
title('Velocity Error vs Time');
grid on;
