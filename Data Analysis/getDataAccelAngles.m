clear all
close all
clc

[a.s f] = setupSerial('/dev/tty.usbmodem1451')
offset = [0,0]
%% Get Data

totalSamples = 100000;
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

histfit(data(:,1));





