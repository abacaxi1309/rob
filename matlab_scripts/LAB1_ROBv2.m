%% Task 1

% Read the data
data = readmatrix('LAB1_9.TXT');

% Extract data and prepare the data
time = data(:, 1) / 1e6; %Convert the time micro for seconds
a_data_1 = data(:, 2) /100; %Convert cm/s^2 to m/s^2
a_data_2 = data(:, 3) /100;
a_data_3 = data(:, 4) /100; 
w_data_1 = data(:, 5) * (pi/180); %convert degrees to rad
w_data_2 = data(:, 6) * (pi/180); 
w_data_3 = data(:, 7) * (pi/180);

% a_data is data from acceleometer and w_data is from gyroscope

% from the data show in the plot, we can see a_data_3 is z.

% First figure: Plot columns 2, 3, 4 vs column 1
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

% Second figure: Plot columns 5, 6, 7 vs column 1
figure;

plot(time, w_data_1,'g','LineWidth', 2, 'DisplayName', 'Angular velocity Y');
hold on;
plot(time, w_data_2,'b','LineWidth', 2, 'DisplayName', 'Angular velocity Z');
plot(time, w_data_3,'r','LineWidth', 2, 'DisplayName', 'Angular velocity X');
hold off;

title('Angular Velocity Data vs Time');
xlabel('Time (seconds)');
ylabel('Angular Velocity Data (rad/s)');
legend;

%% Task 2

% Apply median filter
a_data_1_filtered = medfilt1(a_data_1, 5); % Janela de filtro = 5
a_data_2_filtered = medfilt1(a_data_2, 5);
a_data_3_filtered = medfilt1(a_data_3, 5);
w_data_1_filtered = medfilt1(w_data_1, 5);
w_data_2_filtered = medfilt1(w_data_2, 5);
w_data_3_filtered = medfilt1(w_data_3, 5);

% Plot original vs filtered acceleration data
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

% Plot original vs filtered angular velocity data
figure;
subplot(2,1,1);
plot(time, w_data_1, 'g', 'LineWidth', 2, 'DisplayName', 'Angular velocity Y');
hold on;
plot(time, w_data_2, 'b', 'LineWidth', 2, 'DisplayName', 'Angular velocity Z');
plot(time, w_data_3, 'r', 'LineWidth', 2, 'DisplayName', 'Angular velocity X');
hold off;
title('Original Angular Velocity Data vs Time');
xlabel('Time (s)');
ylabel('Angular Velocity Data (rad/s)');
legend;

subplot(2,1,2);
plot(time, w_data_1_filtered, 'g', 'LineWidth', 2, 'DisplayName', 'Angular velocity Y');
hold on;
plot(time, w_data_2_filtered, 'b', 'LineWidth', 2, 'DisplayName', 'Angular velocity Z');
plot(time, w_data_3_filtered, 'r', 'LineWidth', 2, 'DisplayName', 'Angular velocity X');
hold off;
title('Filtered Angular Velocity Data vs Time');
xlabel('Time (s)');
ylabel('Angular Velocity Data (rad/s)');
legend;

%% Task 5
% Initialize variables for orientation
old_theta_x = 0; % Initial angle on the X axis
old_theta_y = 0; % Initial angle on the Y axis
old_theta_z = 0; % Initial angle on the Z axis

old_time = 0; % Initial time

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

    alphak = cos(alpha) * (w_data_1_filtered(i) + sin(alpha) * tan(beta) * w_data_2_filtered(i) + cos(alpha) * tan(beta) * w_data_3_filtered(i));
    betak = cos(alpha) * (cos(alpha) * w_data_2_filtered(i) + sin(alpha) * w_data_3_filtered(i));
    lambdak = cos(alpha) * ((sin(alpha) / cos(beta)) * w_data_2_filtered(i) + (cos(alpha) / cos(beta)) * w_data_3_filtered(i));

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

% Plot the 3D orientation trajectory
figure;
% Trajectory 1: Using motion equations
plot3(theta_x, theta_y, theta_z, 'LineWidth', 1.5); % Orientation trajectory (Euler)

% Adjust plot
grid on; % Enable grid
xlabel('\theta_x (rad)');
ylabel('\theta_y (rad)');
zlabel('\theta_z (rad)');
title('3D Orientation Trajectory');
legend('Trajectory (Motion Equations)');
%view(3);
%axis equal; % Ensure axes have the same scale
%hold on; % Allow adding another plot to the same figure
%% Task 6
% Step 1: Integrate the acceleration to get velocity
velocity_x = cumtrapz(time, a_data_1_filtered); % Velocity in the X direction
velocity_y = cumtrapz(time, a_data_2_filtered); % Velocity in the Y direction
velocity_z = cumtrapz(time, a_data_3_filtered); % Velocity in the Z direction

% Step 2: Integrate the velocity to get position
position_x_cum = cumtrapz(time, velocity_x); % Position in the X direction
position_y_cum = cumtrapz(time, velocity_y); % Position in the Y direction
position_z_cum = cumtrapz(time, velocity_z); % Position in the Z direction

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
for i = 1:length(time) % Start from the second point
    % Time step
    dt = time(i) - old_time;

    % Update velocities using v = v_old + a * dt
    next_velocity_x = old_velocity_x + a_data_1_filtered(i) * dt;
    next_velocity_y = old_velocity_y + a_data_2_filtered(i) * dt;
    next_velocity_z = old_velocity_z + a_data_3_filtered(i) * dt;

    % Update positions using p = p_old + v_old * dt + 0.5 * a * dt^2
    next_pos_x = old_pos_x + old_velocity_x * dt + 0.5 * a_data_1_filtered(i) * dt^2;
    next_pos_y = old_pos_y + old_velocity_y * dt + 0.5 * a_data_2_filtered(i) * dt^2;
    next_pos_z = old_pos_z + old_velocity_z * dt + 0.5 * a_data_3_filtered(i) * dt^2;

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
    
% Plot the final 3D trajectory
figure; % Create new figure
plot3(position_x, position_y, position_z, 'LineWidth', 1.5); % Full trajectory
hold on; % Allow adding another plot to the same figure
plot3(position_x_cum, position_y_cum, position_z_cum, 'LineWidth', 2); % Orientation trajectory (cumtrapz)
grid on; % Enable grid
xlabel('Position X (m)'); % X-axis label
ylabel('Position Y (m)'); % Y-axis label
zlabel('Position Z (m)'); % Z-axis label
title('Calculated 3D Trajectory'); % Plot title

legend('Trajectory (Numerical)', 'Trajectory (cumtrapz)', 'Location', 'best'); 
hold off;

% Configure orientation arrow scale
scale = 20; % Scale for arrow length

% Create the figure
figure;
hold on;

% Position trajectory
plot3(position_x, position_y, position_z, 'k', 'LineWidth', 1.5); % Trajectory in black
scatter3(position_x, position_y, position_z, 10, 'filled'); % Points along the trajectory

% Loop to draw orientations
for i = 1:10:length(time) % Every 10 points
    % Get current angles
    tx = theta_x(i);
    ty = theta_y(i);
    tz = theta_z(i);

    % Calculate rotation matrix manually
    Rz = [cos(tz), -sin(tz), 0;
          sin(tz),  cos(tz), 0;
          0,        0,       1];
    Ry = [cos(ty),  0, sin(ty);
          0,        1, 0;
         -sin(ty),  0, cos(ty)];
    Rx = [1, 0,        0;
          0, cos(tx), -sin(tx);
          0, sin(tx),  cos(tx)];
    
    % Full rotation matrix (ZYX)
    R = Rz * Ry * Rx;

    % Locally oriented unit vectors (X, Y, Z local)
    x_dir = R(:, 1) * scale; % Local X axis
    y_dir = R(:, 2) * scale; % Local Y axis
    z_dir = R(:, 3) * scale; % Local Z axis

    % Current position
    pos = [position_x(i), position_y(i), position_z(i)];

    % Draw local axis vectors
    quiver3(pos(1), pos(2), pos(3), x_dir(1), x_dir(2), x_dir(3), 'r', 'LineWidth', 1.5); % X axis in red
    quiver3(pos(1), pos(2), pos(3), y_dir(1), y_dir(2), y_dir(3), 'g', 'LineWidth', 1.5); % Y axis in green
    quiver3(pos(1), pos(2), pos(3), z_dir(1), z_dir(2), z_dir(3), 'b', 'LineWidth', 1.5); % Z axis in blue
end

% Adjust plot
xlabel('Position X (m)');
ylabel('Position Y (m)');
zlabel('Position Z (m)');
title('3D Trajectory with Local Orientations');
grid on;
view(3); % 3D
legend({'Trajectory', 'Position','Orientation X', 'Orientation Y', 'Orientation Z'}, 'Location', 'best');
hold off;