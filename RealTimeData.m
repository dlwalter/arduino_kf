clear all;
close all;
clc;

load KalmanData2.mat;

% plot(data.time(1,1:450),data.kpitch(1,1:450), 'k',...
%      data.time(1,1:450), data.encoder(1,1:450), 'b',...
%                 data.time(1,1:450),data.accel(2,1:450), 'r');
%             %set(handles.axes, 'Xlim', [xleft, xright])
%             %set(handles.axes, 'Ylim', [-35, 35]);
% xlabel('Time (sec)');
% ylabel('Position (deg)');
% title('Real Time Data Collected From IMU');
% legend('Kalman Estimate', 'Actual Position', 'Accelerometer');
% grid on;
% 
% figure()
% plot(data.time(1,1:450),data.kpitch(1,1:450) - data.encoder(1,1:450), 'k',...
%      data.time(1,1:450), data.encoder(1,1:450) - data.encoder(1,1:450), 'b',...
%                 data.time(1,1:450),data.accel(2,1:450) - data.encoder(1,1:450), 'r');
%             %set(handles.axes, 'Xlim', [xleft, xright])
%             %set(handles.axes, 'Ylim', [-35, 35]);
% xlabel('Time (sec)');
% ylabel('Error (deg)');
% title('Error of Real Time Data Collected From IMU');
% legend('Kalman Estimate', 'Actual Position', 'Accelerometer');
% grid on;

errormean = nanmean(abs(data.kpitch(1,1:450) - data.encoder(1,1:450)));
errorvar = nanvar(data.kpitch(1,1:450) - data.encoder(1,1:450));

avar = nanvar(data.accel(1,1:450));
erroramean = nanmean(abs(data.accel(1,1:450) - data.encoder(1,1:450)));

P11 = reshape(data.Pcovariance(1,1,1:450), 1, 450);
P22 = reshape(data.Pcovariance(2,2,1:450), 1, 450);
P33 = reshape(data.Pcovariance(3,3,1:450), 1, 450);
P44 = reshape(data.Pcovariance(4,4,1:450), 1, 450);

t = data.time(1,1:450);
t(t==0) = NaN;
%Plot Diagonals of Covariance of P
figure()
subplot(4,1,1);
plot(t, P11);
title('Diagonals of Real Time Covariance P');
ylabel('P11');
ylim([0 0.5]);
subplot(4,1,2);
plot(t, P22);
ylabel('P22')
ylim([0 0.5]);
subplot(4,1,3);
plot(t, P33);
ylabel('P33');
subplot(4,1,4);
plot(t, P44);
ylabel('P44');
xlabel('Time (sec)');

%Plot Kalman Gains for Roll and Pitch
figure()

K11 = reshape(data.KalmanGain(1,1,1:450), 1, 450);
K22 = reshape(data.KalmanGain(2,2,1:450), 1, 450);

subplot(2,1,1);
plot(t,K11);
title('Kalman Gains for Real Time IMU');
ylabel('First Kalman Gain K11');
subplot(2,1,2);
plot(t,K22);
ylabel('Second Kalman Gain K22')
xlabel('Times (s)');

