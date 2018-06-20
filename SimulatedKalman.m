clear all;
close all;
clc;


%Create Time Vector
maxtime = 30;
f = 15; %sampling Frequency in Hz
dt = 1/f;
t = 1:dt:maxtime;

maxdrift = 3; %maximum drift in deg/s over total time

%Statistics
m_a = 0; %mean of accelerometer bias
%var_ar = 0.022; %roll reading variance
%var_ap = 0.022; %pitch reading variance
var_ar = 0.5;
var_ap = 0.5;

var_gr = 0.034; %gyro roll variance
var_gp = 0.034; %gyro pitch variance

var_grb = .4; %gyro roll bias variance
var_gpb = .4; %gyro pitch bias variance

bias_gp = normrnd(0, var_grb, 1, 1);
bias_gr = normrnd(0, var_gpb, 1, 1);

%Create bias vector
bias_gp = ones(1,length(t))*bias_gp;
bias_gr = ones(1,length(t))*bias_gr;

%Add drift, where bias increases to maximum deg/s over total time
drift = linspace(0,maxdrift,length(t));
%bias_gr = drift+bias_gr;
%bias_gp = drift+bias_gp;

%Create Actual Position Vector
%actual_angVel_r = 100*sin(2*pi()*.25*t);
actual_angVel_r = zeros(1,length(t));
%actual_angVel_p = 100*sin(2*pi()*.5*t);
actual_angVel_p = zeros(1,length(t));

%Control Input Sinusoidal Roll (deg/s)
u_r = normrnd(0, var_gr, 1, length(t)) + bias_gr + actual_angVel_r;
u_p = normrnd(0, var_gp, 1, length(t)) + bias_gp + actual_angVel_p;

%Actual Roll and Tilt Angle from Euler Integration
actual_angle_r = zeros(1, length(t));
actual_angle_p = zeros(1, length(t));

%Add current velocity times change in time to the total angle
for i = 1:length(t)-1
    actual_angle_r(i+1) = actual_angle_r(i) + actual_angVel_r(i)*dt;
    actual_angle_p(i+1) = actual_angle_p(i) + actual_angVel_p(i)*dt;
end

%Accelerometer measurements
z_r = normrnd(m_a, var_ar, 1, length(t)) + actual_angle_r;
z_p = normrnd(m_a, var_ap, 1, length(t)) + actual_angle_p;

%% Kalman Filter

% Dynamic Model

%Observation Model
H = [1 0 0 0; 0 1 0 0];

%Transition Model
PHI = [1 0 -dt 0; 0 1 0 -dt; 0 0 1 0; 0 0 0 1];

%Control Input Model
GAMMA = [dt 0; 0 dt; 0 0; 0 0];

%Bias Model
THETA = [0 0; 0 0; 1 0; 0 1];

B = [var_grb 0; 0 var_gpb];

%Measurement Variance Matrix
V = [var_ar 0; 0 var_ap];

%Process Noise Variance Matrix
W = [var_gr 0; 0 var_gp];

%x is the state

%u is the control input recorded from the gyro
u = [u_r; u_p];

%z is the measurement taken from the accelerometer
z = [z_r; z_p];

%Initial A Priori State
x_bar = zeros(4,length(t));
x_bar(:,1) = [0;0;0;0];

%Initialize A Posteriori State
x_hat = zeros(4,length(t));

%Initial A Priori Covariance
M = zeros(4,4,length(t));
M(:,:,1) = [var_gr 0 0 0; 0 var_gp 0 0; 0 0 var_grb 0; 0 0 0 var_gpb];

%Initial A Posteriori Covariance
P = zeros(4,4,length(t));
P(:,:,1) = (M(:,:,1)^-1 + H'*V^-1*H)^-1;

%Kalman Gain
K = zeros(4,2,length(t));

%% Kalman Filter Algorithm
for k = 2:length(t)
    %Calculate A Priori Estimate
    x_bar(:,k) = PHI*x_hat(:,k-1) + GAMMA*u(:,k-1);
    %Calculate A Priori Covariance
    M(:,:,k) = PHI*P(:,:,k-1)*PHI' + GAMMA*W*GAMMA' + THETA*B*THETA';
    %Calculate Kalman Gain
    K(:,:,k) = M(:,:,k)*H'*(H*M(:,:,k)*H' + V)^-1;
    %Calculate Residual
    r = z(:,k)-H*x_bar(:,k);
    %Calculate A Posteriori Estimate
    x_hat(:,k) = x_bar(:,k) + K(:,:,k)*r;
    %Calculate A Posteriori Covariance
    P(:,:,k) = (M(:,:,k)^-1 + H'*V^-1*H)^-1;

end

x = [actual_angle_r; actual_angle_p; bias_gr; bias_gp];
error = x-x_hat;




figure()
%Plot error in Roll and Roll Bias
plot(t,error(1,:), t, error(2,:))
title('Error in position in deg')
legend('Roll error', 'Pitch error');


figure()
%Plot error in Pitch and Pitch Bias
plot(t, error(3,:), t, error(4,:));
title('Error in Bias in deg/sec')
legend('Roll Bias error', 'Pitch Bias Error');
figure()
for i = 1:length(t)
    P11(i) = P(1,1,i);
    P22(i) = P(2,2,i);
    P33(i) = P(3,3,i);
    P44(i) = P(4,4,i);
end
subplot(4,1,1);
plot(t, P11);
subplot(4,1,2);
plot(t, P22);
subplot(4,1,3);
plot(t, P33);
subplot(4,1,4);
plot(t, P44);











