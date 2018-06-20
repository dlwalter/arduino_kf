clear all;
close all;
clc;

load('accelData3D');

%% Data Processing
set (gcf, 'renderer', 'opengl')
P = var(dataAccel);
data = dataAccel*100;
data = data(any(data~=0,2),:);
data(:,3) = data(:,3)-981; %Remove gravity on Z axis
data_min = min(data);
data_max = max(data);

% shift = zeros(1,3);
% for i = 1:3
%    if data_min(i) < 0;
%       shift(i) = abs(data_min(i))+1;
%    else
%        shift(i) = 0;
%    end
% end

%Create the vector to store frequency of occurance of each data point
%It must be long enough to handle the maximum data point stored + the
%shift to ensure any indice is >=1.
abs_x = zeros(max(data_max),1);
freq_occur = zeros(length(abs_x),3);

abs_x = 0:1:length(freq_occur(:,1))-1;
abs_y = 0:1:length(freq_occur(:,2))-1;
abs_z = 0:1:length(freq_occur(:,3))-1;

abs_x = abs_x/100;
abs_y = abs_y/100;
abs_z = abs_z/100;

%Get Statistics for data
index = 0;
%From Curve Fitting Tool
m_bar = [.3723 .3819 .1715];
P = 1/2*[.03796^2 .0379^2 .04062^2];
A = [0.1567 0.1495 0.1454];


%Sum the frequency of occurance of each data point
for i = 1:length(data)
   
    val = floor(data(i,:));
    
    for j = 1:3
        freq_occur(val(j),j) = freq_occur(val(j),j)+1;
    end
end
%Normalize the data using trapezoidal integration
for i = 1:3
    freq_occur_norm(:,i) = freq_occur(:,i)/(3.8*trapz(freq_occur(:,i)));
end

%Remove Zero Data
freq_occur(freq_occur==0) = NaN;
freq_occur_norm(freq_occur_norm==0) = NaN;
freq_occur_x = freq_occur_norm(:,1);
freq_occur_y = freq_occur_norm(:,2);
freq_occur_z = freq_occur_norm(:,3);

%Plot Normalized data
plot(abs_x, freq_occur_norm(:,1), '.b',...
    abs_y, freq_occur_norm(:,2), '.r',...
    abs_z, freq_occur_norm(:,3), '.g','MarkerSize', 7);
grid on;
hold on;



%%Plot theoretical Gaussian
%abs_thX = min(data_min):1:max(data_max);
%abs_thY = abs_thX;
%abs_thZ = abs_thX;
abs_thX = abs_x;
abs_thY = abs_y;
abs_thZ = abs_z;

shift = 0;
gaussianX = A(1).*exp(-0.5./P(1).*(abs_thX(1,:)' - m_bar(1)).^2);
gaussianY = A(2).*exp(-0.5./P(2).*(abs_thY(1,:)' - m_bar(2)).^2);
gaussianZ = A(3).*exp(-0.5./P(3).*(abs_thZ(1,:)' - m_bar(3)).^2);

plot(abs_thX, gaussianX, '--b', abs_thY, gaussianY, '--r', abs_thZ, gaussianZ, '--g');
legend('X-Axis', 'Y-Axis', 'Z-Axis');
title('Noise Measurements for ADXL345 3-Axis Accelerometer')
xlabel('Accelerometer Output');
ylabel('Frequency of Occurence')




