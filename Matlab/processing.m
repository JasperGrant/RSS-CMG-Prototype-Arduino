%% Plotting Gyroscope Prototype Data
% Andrew Doucet
clear; clc;

% Importing data from file
data = importdata("data.txt");

time = data(:,1)';
x = data(:,2)';
y = data(:,3)';
z = data(:,4)';
angle = data(:,5)';

% Converts time to seconds since start
time = (time - time(1))/1000;

% Determination of actual rotation directoins
magnitude = sqrt(x.^2+y.^2+z.^2);
direction = -z./abs(z);
gyro = magnitude.*direction;

% Plot of velocity over time
figure(1)
plot(time,gyro)
title("Data Logging Demonstration")
xlabel("Time (s)")
ylabel("RPM")
grid on
