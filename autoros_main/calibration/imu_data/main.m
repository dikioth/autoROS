%% Angular velocity

d = load('ang_vel_data.mat');

ang_vel = d.ang_vel;
%
% Mean and std.
xvar = var(ang_vel(:,1)); xmean = mean(ang_vel(:,1));
yvar = var(ang_vel(:,2)); ymean = mean(ang_vel(:,2));
zvar = var(ang_vel(:,3)); zmean = mean(ang_vel(:,3));

% Results.
% x:
%   - offset: -0.0011
%   - variance: 2.5178e-06
%
% y:
%   - offset: 6.0784e-05
%   - variance: 2.1043e-06
% z :
%   - offset: -0.0036
%   - variance: 5.7853e-06


%% Linear acceleration
close all; clear all;
d = load('linear_accel_data.mat');

linear_accel = d.linear_accel;
%
% Mean and std.
xvar = var(linear_accel(:,1)); xmean = mean(linear_accel(:,1));
yvar = var(linear_accel(:,2)); ymean = mean(linear_accel(:,2));
zvar = var(linear_accel(:,3)); zmean = mean(linear_accel(:,3));

% Results.
% x:
%   - offset: -0.0253
%   - variance: 0.0028
%
% y:
%   - offset: -0.0135
%   - variance: 0.0044

% z :
%   - offset: 9.8287
%   - variance: 0.0043
