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
%Integration to obtain the orientation angles
theta_x = cumtrapz(time, w_data_1_filtered); % Angle around the X-axis
theta_y = cumtrapz(time, w_data_2_filtered); % Angle around the Y-axis
theta_z = cumtrapz(time, w_data_3_filtered); % Angle around the Z-axis

%Trajectory in the orientation space
figure;
plot3(theta_x, theta_y, theta_z, 'LineWidth', 2);
grid on;
xlabel('\theta_x (rad)');
ylabel('\theta_y (rad)');
zlabel('\theta_z (rad)');
title('Trajectory in Orientation Space');
legend('Orientation Trajectory');

%% Task 6

% Step 1: Integrate the acceleration to get velocity
velocity_x = cumtrapz(time, a_data_1_filtered); % Velocity in the X direction
velocity_y = cumtrapz(time, a_data_2_filtered); % Velocity in the Y direction
velocity_z = cumtrapz(time, a_data_3_filtered); % Velocity in the Z direction

% Step 2: Integrate the velocity to get position
position_x = cumtrapz(time, velocity_x); % Position in the X direction
position_y = cumtrapz(time, velocity_y); % Position in the Y direction
position_z = cumtrapz(time, velocity_z); % Position in the Z direction

% Step 3: Plot the 3D Cartesian trajectory
figure;
plot3(position_x, position_y, position_z, 'LineWidth', 2);
grid on;
xlabel('Position X (m)');
ylabel('Position Y (m)');
zlabel('Position Z (m)');
title('Reconstructed 3D Trajectory in Cartesian Space');
legend('Trajectory in 3D Space');
axis tight;


% Animate 3D position trajectory
figure;
subplot(1,2,1);
h1 = plot3(position_x(1), position_y(1), position_z(1), 'o'); % Initialize the plot with the first point
axis([min(position_x) max(position_x) min(position_y) max(position_y) min(position_z) max(position_z)]);
xlabel('Position X (m)');
ylabel('Position Y (m)');
zlabel('Position Z (m)');
title('Animated 3D Position Trajectory');
grid on;

for i = 2:length(position_x)
    set(h1, 'XData', position_x(1:i), 'YData', position_y(1:i), 'ZData', position_z(1:i)); % Update the plot
    pause(0.01); % Adjust the frame rate of the animation
end

% Animate orientation trajectory in 3D space
subplot(1,2,2);
h2 = plot3(theta_x(1), theta_y(1), theta_z(1), 'o'); % Initialize the plot with the first point
axis([min(theta_x) max(theta_x) min(theta_y) max(theta_y) min(theta_z) max(theta_z)]);
xlabel('\theta_x (rad)');
ylabel('\theta_y (rad)');
zlabel('\theta_z (rad)');
title('Animated Orientation Trajectory');
grid on;

for i = 2:length(theta_x)
    set(h2, 'XData', theta_x(1:i), 'YData', theta_y(1:i), 'ZData', theta_z(1:i)); % Update the plot
    pause(0.01); % Adjust the frame rate of the animation
end