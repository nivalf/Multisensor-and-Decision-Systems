% Template for Kalman Filter (KF), Extended Kalman Filter (EKF) and Iterated Extended Kalman Filter (IEKF)

clear; clear all; close all;

% Read data files
load('..\..\Data\data_task1_1.mat')

% Initialization
Ts=dt;
xhat_km1_km1 = Ex_0; % x(0|0) = E{x_0}
P_km1_km1 = P_0;  % P(0|0) = P(0)

n = length(xhat_km1_km1); % n: state dimension
m = 1;     % m: observation dimension

% Preallocate storage
stdx_cor  = zeros(N, n);  % \sigma(k-1|k-1), standard deviation of state estimation error (hint: diagonal elements of P(k-1|k-1))
x_cor     = zeros(N, n);  % \hat{x}(k-1|k-1), previous estimation
K_k       = cell(N, 1);   % K(k) Kalman Gain
innov     = zeros(N, m);  % y(k)-y(k|k-1), innovation, with y(k|k-1)=h(\hat{x}(k|k-1),u(k|k-1),k);

for k=1:N
    % Step 1: Prediction

    % Step 2: Covariance matrix of state prediction error / Minimum
    % prediction MSE

    % Step 3: Kalman Gain
    
    % Step 4: Measurement Update (Correction)

    % Step 5: Correction for Covariance matrix of state Estimate error /
    % Minimum MSE

    % Save data: State estimate and std dev
    stdx_cor(:,k) = ...; % \sigma(k-1|k-1) Standard deviation of state estimation error
    x_cor(:,k) = ...; % \hat{x}(k-1|k-1), estimated state
    K_k(:,k) = ...; % K(k), Kalman Gain
    innov(k,:)= ...; % innovation

    % Recursive step

end

%% Plots

figure

subplot(311)
plot(t, x_k, 'g'); % state trajectory
hold on
plot(t, z_k, 'r'); % output trajectory
plot(t, x_cor, 'b'); % state estimation
ylim([0 10])
title('Real state and Estimated state')

subplot(312)
plot(t, x_cor - x_k, 'r'); % estimation error
hold on
plot(t, +stdx_cor, 'b'); % + standard deviation
plot(t, -stdx_cor, 'b'); % - standard deviation
ylim([-max(stdx_cor(10:end))*2 +max(stdx_cor(10:end))*2])
title('Error of Estimated state and Standard Deviation')

subplot(313)
plot(t, innov, 'b'); % Innovation
title('Innovation')


