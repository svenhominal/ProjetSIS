%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Description: A basic data loading and ground truth plot
%   Last modified: 2023-09-06
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc                         % clear command window
clear all                   % clear workspace
close all                   % close all open figures
%% Load the CSV file
filename = '../../controllers/supervisor/data/ground_truth.csv';
file_odometry = '../../controllers/controller/data/odometry.csv';
data = readtable(filename);
data2 = readtable(file_odometry);

% Strip spaces from column names
data.Properties.VariableNames = strtrim(data.Properties.VariableNames);
data2.Properties.VariableNames = strtrim(data2.Properties.VariableNames);

%% Plot the columns
figure;

subplot(2, 2, 1);
plot(data.x, data.y);
title('Position');
xlabel('x [m]');
ylabel('y [m]');
grid on;

subplot(2, 2, 2);
plot(data.time, data.heading);
title('Heading');
xlabel('time [s]');
ylabel('[rad]');
grid on;

subplot(2, 2, 3);
plot(data.time, data.vel_x);
hold on;
plot(data.time, data.vel_y);
title('Velocity');
xlabel('time [s]');
ylabel('[m/s]');
legend({'$\dot{x}$', '$\dot{y}$'}, 'Interpreter', 'latex');
grid on;

subplot(2, 2, 4);
plot(data.time, data.vel_heading);
title('Angular velocity');
xlabel('time [s]');
ylabel('[rad/s]');
grid on;

sgtitle('Pose Ground Truth');

%% Plot for odometry 

figure()
plot(data2.x, data2.y)
title('Pose estimation (with odometry)');
xlabel('x [m]');
ylabel('y [m]');
grid on;
