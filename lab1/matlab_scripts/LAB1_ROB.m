%% Task 1

% Read the data from the file
data = readmatrix('LAB1_9.TXT');

% Extracting and preparing the data
time = data(:, 1) / 1e6; 
a_data_1 = data(:, 2) /100; 
a_data_2 = data(:, 3) /100;
a_data_3 = data(:, 4) /100; 
w_data_1 = data(:, 5) * (pi/180);
w_data_2 = data(:, 6) * (pi/180); 
w_data_3 = data(:, 7) * (pi/180);

% Offset to y to start at zero
a_data_2 = a_data_2 - a_data_2(1);

% 1st Figure
figure;

plot(time, a_data_1, 'r','LineWidth', 2, 'DisplayName', 'Accelerometer X');
hold on;
plot(time, a_data_2,'g','LineWidth', 2, 'DisplayName', 'Accelerometer Y');
plot(time, a_data_3,'b','LineWidth', 2, 'DisplayName', 'Accelerometer Z');
hold off;

title('Acceleration Data vs Time');
xlabel('Time (seconds)');
ylabel('Acceleration Data (m/s^2)');
legend;

% 2nd Figure 
figure;

plot(time, w_data_1,'r','LineWidth', 2, 'DisplayName', 'Angular velocity X');
hold on;
plot(time, w_data_2,'g','LineWidth', 2, 'DisplayName', 'Angular velocity Y');
plot(time, w_data_3,'b','LineWidth', 2, 'DisplayName', 'Angular velocity Z');
hold off;

title('Angular Velocity Data vs Time');
xlabel('Time (seconds)');
ylabel('Angular Velocity Data (rad/s)');
legend;

%% Task 2

% Median filter with 5
a_data_1_filtered = medfilt1(a_data_1, 5);
a_data_2_filtered = medfilt1(a_data_2, 5);
a_data_3_filtered = medfilt1(a_data_3, 5);
w_data_1_filtered = medfilt1(w_data_1, 5);
w_data_2_filtered = medfilt1(w_data_2, 5);
w_data_3_filtered = medfilt1(w_data_3, 5);

% Comparing Figures (original vs filtered acceleration data)
figure;
subplot(2,1,1);
plot(time, a_data_1, 'r', 'LineWidth', 2, 'DisplayName', 'Accelerometer X');
hold on;
plot(time, a_data_2, 'g', 'LineWidth', 2, 'DisplayName', 'Accelerometer Y');
plot(time, a_data_3, 'b', 'LineWidth', 2, 'DisplayName', 'Accelerometer Z');
hold off;
title('Original Acceleration Data vs Time');
xlabel('Time (s)');
ylabel('Acceleration Data (m/s^{2})');
legend;

subplot(2,1,2);
plot(time, a_data_1_filtered, 'r', 'LineWidth', 2, 'DisplayName', 'Accelerometer X');
hold on;
plot(time, a_data_2_filtered, 'g', 'LineWidth', 2, 'DisplayName', 'Accelerometer Y');
plot(time, a_data_3_filtered, 'b', 'LineWidth', 2, 'DisplayName', 'Accelerometer Z');
hold off;
title('Filtered Acceleration Data vs Time');
xlabel('Time (s)');
ylabel('Acceleration Data (m/s^{2})');
legend;

% Comparing Figures (original vs filtered angular velocity data)
figure;
subplot(2,1,1);
plot(time, w_data_1, 'r', 'LineWidth', 2, 'DisplayName', 'Angular velocity X');
hold on;
plot(time, w_data_2, 'g', 'LineWidth', 2, 'DisplayName', 'Angular velocity Y');
plot(time, w_data_3, 'b', 'LineWidth', 2, 'DisplayName', 'Angular velocity Z');
hold off;
title('Original Angular Velocity Data vs Time');
xlabel('Time (s)');
ylabel('Angular Velocity Data (rad/s)');
legend;

subplot(2,1,2);
plot(time, w_data_1_filtered, 'r', 'LineWidth', 2, 'DisplayName', 'Angular velocity X');
hold on;
plot(time, w_data_2_filtered, 'g', 'LineWidth', 2, 'DisplayName', 'Angular velocity Y');
plot(time, w_data_3_filtered, 'b', 'LineWidth', 2, 'DisplayName', 'Angular velocity Z');
hold off;
title('Filtered Angular Velocity Data vs Time');
xlabel('Time (s)');
ylabel('Angular Velocity Data (rad/s)');
legend;

%% Task 5

% Initializing variables for orientation
old_theta_x = 0; % Initial angle on the X axis
old_theta_y = 0; % Initial angle on the Y axis
old_theta_z = 0; % Initial angle on the Z axis

% Initial time
old_time = 0; 

% Initial angles
alpha = 0;
beta = 0;
lambda = 0;

% Preallocate arrays to store results
theta_x = zeros(size(time)); % Angle around the X axis
theta_y = zeros(size(time)); % Angle around the Y axis
theta_z = zeros(size(time)); % Angle around the Z axis

% Loop to calculate orientations
for i = 1:length(time)
    % Time step
    dt = time(i) - old_time;
    
 
    alphak = (w_data_1_filtered(i) - sin(beta) * w_data_3_filtered(i));
    betak =  (cos(alpha) * w_data_2_filtered(i) + sin(alpha) * cos(beta)*w_data_3_filtered(i));
    lambdak = (-sin(alpha) * w_data_2_filtered(i) + (cos(alpha) * cos(beta)) * w_data_3_filtered(i));

    % Update angles using motion equations
    % theta = theta_old + w * dt
    next_theta_x = old_theta_x + alphak * dt;
    next_theta_y = old_theta_y + betak * dt;
    next_theta_z = old_theta_z + lambdak * dt;

    % Store results
    theta_x(i) = next_theta_x;
    theta_y(i) = next_theta_y;
    theta_z(i) = next_theta_z;

    % Update old variables for the next iteration
    old_theta_x = next_theta_x;
    old_theta_y = next_theta_y;
    old_theta_z = next_theta_z;

    old_time = time(i);
end
% Plot all angles on the same graph
figure;

% Plot theta_x, theta_y, and theta_z on the same plot
plot(time, theta_x, 'LineWidth', 2, 'Color', 'r', 'DisplayName', '\theta_x');
hold on;
plot(time, theta_y, 'LineWidth', 2, 'Color', 'g', 'DisplayName', '\theta_y'); 
plot(time, theta_z, 'LineWidth', 2, 'Color', 'b', 'DisplayName', '\theta_z'); 

xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Angles (rad)', 'FontSize', 12, 'FontWeight', 'bold');
title('Angles (\theta_x, \theta_y, \theta_z) Over Time', 'FontSize', 14, 'FontWeight', 'bold');

grid on;
legend('show', 'Location', 'best');


hold off;



% Plot of the 3D orientation trajectory
figure;
plot3(theta_x, theta_y, theta_z, 'LineWidth', 2, 'Color', [0.1 0.6 0.8]); 

grid on; 
grid minor;
xlabel('\theta_x (rad)', 'FontSize', 12, 'FontWeight', 'bold'); 
ylabel('\theta_y (rad)', 'FontSize', 12, 'FontWeight', 'bold'); 
zlabel('\theta_z (rad)', 'FontSize', 12, 'FontWeight', 'bold'); 
title('3D Orientation Trajectory', 'FontSize', 14, 'FontWeight', 'bold'); 
legend('Trajectory (Motion Equations)', 'FontSize', 10, 'Location', 'best');

axis equal; 
rotate3d on;

xlim([min(theta_x) - 0.1, max(theta_x) + 0.1]);
ylim([min(theta_y) - 0.1, max(theta_y) + 0.1]);
zlim([min(theta_z) - 0.1, max(theta_z) + 0.1]);

hold on;
plot3(theta_x, theta_y, theta_z, 'o', 'MarkerSize', 4, 'MarkerEdgeColor', [0.1 0.6 0.8], 'MarkerFaceColor', [0.1 0.6 0.8]);
hold off;

%% Task 6

% Preallocate arrays for acceleration components
aworld_x = zeros(1, length(time));
aworld_y = zeros(1, length(time));
aworld_z = zeros(1, length(time));

% Initialize variables
old_velocity_x = 0; % Initial velocity in the X axis
old_velocity_y = 0; % Initial velocity in the Y axis
old_velocity_z = 0; % Initial velocity in the Z axis

old_pos_x = 0; % Initial position in the X axis
old_pos_y = 0; % Initial position in the Y axis
old_pos_z = 0; % Initial position in the Z axis

old_time = 0; % Initial time

% Preallocate arrays to store results
velocity_x = zeros(size(time));
velocity_y = zeros(size(time));
velocity_z = zeros(size(time));

position_x = zeros(size(time));
position_y = zeros(size(time));
position_z = zeros(size(time));

% Loop to calculate velocities and positions
for i = 1:length(time) % Starts from the second point
    tx = theta_x(i);
    ty = theta_y(i);
    tz = theta_z(i);

    % Computing rotation matrix manually
    Rz = [cos(tz), -sin(tz), 0;
          sin(tz),  cos(tz), 0;
          0,        0,       1];
    Ry = [cos(ty),  0, sin(ty);
          0,        1, 0;
         -sin(ty),  0, cos(ty)];
    
    % Full rotation matrix (ZY)
    R = Rz * Ry;
    
    aworld = R*[a_data_1_filtered(i);a_data_2_filtered(i);a_data_3_filtered(i)] - 9.8*[0;0;1];
    display(aworld)

    % Store acceleration components
    aworld_x(i) = aworld(1);
    aworld_y(i) = aworld(2);
    aworld_z(i) = aworld(3);

    % Time step
    dt = time(i) - old_time;

    % Update velocities using v = v_old + a * dt
    next_velocity_x = old_velocity_x + aworld(1) * dt;
    next_velocity_y = old_velocity_y + aworld(2) * dt;
    next_velocity_z = old_velocity_z + aworld(3) * dt;

    % Update positions using p = p_old + v_old * dt + 0.5 * a * dt^2
    next_pos_x = old_pos_x + old_velocity_x * dt + 0.5 * aworld(1) * dt^2;
    next_pos_y = old_pos_y + old_velocity_y * dt + 0.5 * aworld(2) * dt^2;
    next_pos_z = old_pos_z + old_velocity_z * dt + 0.5 * aworld(3) * dt^2;

    % Store results
    velocity_x(i) = next_velocity_x;
    velocity_y(i) = next_velocity_y;
    velocity_z(i) = next_velocity_z;

    position_x(i) = next_pos_x;
    position_y(i) = next_pos_y;
    position_z(i) = next_pos_z;

    % Update old variables
    old_velocity_x = next_velocity_x;
    old_velocity_y = next_velocity_y;
    old_velocity_z = next_velocity_z;

    old_pos_x = next_pos_x;
    old_pos_y = next_pos_y;
    old_pos_z = next_pos_z;

    old_time = time(i);
end

% Integrating the acceleration to get velocity
velocity_x = cumtrapz(time, aworld_x); % Velocity in the X direction
velocity_y = cumtrapz(time, aworld_y); % Velocity in the Y direction
velocity_z = cumtrapz(time, aworld_z); % Velocity in the Z direction

% Integrating the velocity to get position
position_x_cum = cumtrapz(time, velocity_x); % Position in the X direction
position_y_cum = cumtrapz(time, velocity_y); % Position in the Y direction
position_z_cum = cumtrapz(time, velocity_z); % Position in the Z direction
    
% Create a new figure
figure;

% Plot 1: Numerical Trajectory
subplot(1, 2, 1);
plot3(position_x, position_y, position_z, '-b', 'LineWidth', 2, 'Marker', 'o'); 
hold on;
plot3(position_x(1), position_y(1), position_z(1), 'gp', 'MarkerSize', 12, 'LineWidth', 2); 
hold off;
grid on; 
xlabel('Position X (m)', 'FontSize', 12, 'FontWeight', 'bold'); 
ylabel('Position Y (m)', 'FontSize', 12, 'FontWeight', 'bold'); 
zlabel('Position Z (m)', 'FontSize', 12, 'FontWeight', 'bold'); 
title('3D Trajectory using kinematic equations', 'FontSize', 14, 'FontWeight', 'bold'); 
set(gca, 'FontSize', 10, 'LineWidth', 1.5); 
legend('Trajectory', 'Initial Point', 'Location', 'best'); 

% Plot 2: cumtrapz Trajectory
subplot(1, 2, 2); 
plot3(position_x_cum, position_y_cum, position_z_cum, '-r', 'LineWidth', 2, 'Marker', 's'); 
hold on;
plot3(position_x_cum(1), position_y_cum(1), position_z_cum(1), 'gp', 'MarkerSize', 12, 'LineWidth', 2); 
hold off;
grid on; 
xlabel('Position X (m)', 'FontSize', 12, 'FontWeight', 'bold'); 
ylabel('Position Y (m)', 'FontSize', 12, 'FontWeight', 'bold'); 
zlabel('Position Z (m)', 'FontSize', 12, 'FontWeight', 'bold'); 
title('3D Trajectory using cumtrapz', 'FontSize', 14, 'FontWeight', 'bold'); 
set(gca, 'FontSize', 10, 'LineWidth', 1.5); 
legend('Trajectory', 'Initial Point', 'Location', 'best'); 

sgtitle('Comparison of 3D Trajectories', 'FontSize', 16, 'FontWeight', 'bold');

set(gcf, 'Position', [100, 100, 1200, 500]); 



% Final Plot
scale = 2;
figure('Position', [100, 100, 600, 500]); 
hold on;

% Position Trajectory
plot3(position_x, position_y, position_z, 'k', 'LineWidth', 1.5); 
scatter3(position_x, position_y, position_z, 10, 'filled', 'MarkerFaceColor', [0.3, 0.75, 0.93], 'MarkerEdgeColor', 'k');

for i = 1:10:length(time) 
    tx = theta_x(i);
    ty = theta_y(i);
    tz = theta_z(i);

    % Rotation Matrix
    Rz = [cos(tz), -sin(tz), 0;
          sin(tz),  cos(tz), 0;
          0,        0,       1]; 
    Ry = [cos(ty),  0, sin(ty);
          0,        1, 0;
         -sin(ty),  0, cos(ty)]; 
    
    R = Rz * Ry;

    x_dir = R(:, 1) * scale; 
    y_dir = R(:, 2) * scale; 
    z_dir = R(:, 3) * scale; 

    % Recent position
    pos = [position_x(i), position_y(i), position_z(i)];

    quiver3(pos(1), pos(2), pos(3), x_dir(1), x_dir(2), x_dir(3), 'r', 'LineWidth', 1.5, 'MaxHeadSize', 1); 
    quiver3(pos(1), pos(2), pos(3), y_dir(1), y_dir(2), y_dir(3), 'g', 'LineWidth', 1.5, 'MaxHeadSize', 1); 
    quiver3(pos(1), pos(2), pos(3), z_dir(1), z_dir(2), z_dir(3), 'b', 'LineWidth', 1.5, 'MaxHeadSize', 1); 
end

xlabel('Position X (m)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Position Y (m)', 'FontSize', 14, 'FontWeight', 'bold');
zlabel('Position Z (m)', 'FontSize', 14, 'FontWeight', 'bold');

title('3D Trajectory with Local Orientations', 'FontSize', 16, 'FontWeight', 'bold');

grid on;
grid minor;

xlim([min(position_x)-0.2, max(position_x)+0.2]);
ylim([min(position_y)-0.2, max(position_y)+0.2]);
zlim([min(position_z)-0.2, max(position_z)+0.2]);

view(45, 35);

legend({'Trajectory', 'Position', 'Orientation X', 'Orientation Y', 'Orientation Z'}, ...
    'FontSize', 12, 'Location', 'bestoutside', 'FontWeight', 'bold');

rotate3d on;

hold off;

data_to_save = struct();
data_to_save.positions = [position_x, position_y, position_z]; % 3D Positions
data_to_save.orientations = [theta_x, theta_y, theta_z];       % 3D Orientations

save('trajectory_data.mat', '-struct', 'data_to_save');
