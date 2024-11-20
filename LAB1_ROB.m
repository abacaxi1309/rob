% Read the data
data = readmatrix('LAB1_9.TXT');

% Extract data
time = data(:, 1);
a_data_1 = data(:, 2);
a_data_2 = data(:, 3);
a_data_3 = data(:, 4); 
w_data_1 = data(:, 5); 
w_data_2 = data(:, 6); 
w_data_3 = data(:, 7);

% % Create subplots for each data column (2 to 7) against time
% figure;
% 
% subplot(3, 2, 1);
% plot(time, a_data_1);
% title('a_data_1 vs Time');
% xlabel('Time');
% ylabel('a_data_1');
% 
% subplot(3, 2, 2);
% plot(time, a_data_2);
% title('a_data_2 vs Time');
% xlabel('Time');
% ylabel('a_data_2');
% 
% subplot(3, 2, 3);
% plot(time, a_data_3);
% title('a_data_3 vs Time');
% xlabel('Time');
% ylabel('a_data_3');
% 
% subplot(3, 2, 4);
% plot(time, w_data_1);
% title('w_data_1 vs Time');
% xlabel('Time');
% ylabel('w_data_1');
% 
% subplot(3, 2, 5);
% plot(time, w_data_2);
% title('w_data_2 vs Time');
% xlabel('Time');
% ylabel('w_data_2');
% 
% subplot(3, 2, 6);
% plot(time, w_data_3);
% title('w_data_3 vs Time');
% xlabel('Time');
% ylabel('w_data_3');
% 
% % Adjust the layout for better spacing
% tight_layout();

% First figure: Plot columns 2, 3, 4 vs column 1
figure;

plot(time, a_data_1, 'r','LineWidth', 2, 'DisplayName', 'a_data_1');
hold on;
plot(time, a_data_2,'g','LineWidth', 2, 'DisplayName', 'a_data_2');
plot(time, a_data_3,'b','LineWidth', 2, 'DisplayName', 'a_data_3');
hold off;

title('Acceleration Data vs Time');
xlabel('Time');
ylabel('Acceleration Data');
legend;

% Second figure: Plot columns 5, 6, 7 vs column 1
figure;

plot(time, w_data_1,'r','LineWidth', 2, 'DisplayName', 'w_data_1');
hold on;
plot(time, w_data_2,'g','LineWidth', 2, 'DisplayName', 'w_data_2');
plot(time, w_data_3,'b','LineWidth', 2, 'DisplayName', 'w_data_3');
hold off;

title('Angular Velocity Data vs Time');
xlabel('Time');
ylabel('Angular Velocity Data');
legend;
