clear all
close all
clc

[a.s f] = setupSerial('/dev/tty.usbmodem1451')

%% Get Data
sum_roll = 0; sum_pitch = 0;
for i = 1:100
    [roll pitch] = getAccelAngles(a, [0,0]);
    sum_roll = sum_roll + roll;
    sum_pitch = sum_pitch + pitch;
end
offset = [sum_roll/100 sum_pitch/100];

totalSamples = 1000;
data = zeros(totalSamples,2);

tStart = tic;
for i = 1:totalSamples
    [roll pitch] = getAccelAngles(a, offset);
    data(i,:) = [roll pitch];
end

tElapsed = toc(tStart);
sampleRate = length(data)/tElapsed;

%%

closeSerial()





