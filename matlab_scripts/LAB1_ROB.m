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

%% Task 4


%% Task 5