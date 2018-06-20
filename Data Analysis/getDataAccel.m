clear all
close all
clc

[a.s f] = setupSerial('/dev/tty.usbmodem1451')

fs = 100;       %set sample rate
sn = 100000;    %set number of samples
mt = sn/fs;     %measurement time in seconds


%% Get Data

totalSamples = mt*fs;
dataAccel = zeros(totalSamples,3);
%Do first read to avoid 0 returns.
readAccel(a);
pause(0.005)


tic;
last_read = 0;
ind = 1;
nind = 1;

while toc < mt
    
    %Get Measurement
    [ax ay az] = readAccel(a);
    dataAccel(ind,:) = [ax ay az];
    
    %Wait for next measurement
    while(nind == ind)
        nind = floor(toc*fs)+1;
    end
    ind = nind;
    
    
end

tElapsed = toc;
sampleRate = length(dataAccel)/tElapsed;


closeSerial()





