clear all;
close all;
clc;

load('dataAccelAngles');

%% Data Processing
set (gcf, 'renderer', 'opengl')
data = floor(data*10000);
data_min = min(data);
data_max = max(data);

shift = zeros(1,2);
for i = 1:2
   if data_min(i) < 0;
      shift(i) = abs(data_min(i))+1;
   else
       shift(i) = 0;
   end
end
%%
%Create the vector to store frequency of occurance of each data point
%It must be long enough to handle the maximum data point stored + the
%shift to ensure any indice is >=1.
abs_x_shifted = zeros(max(data_max)+max(shift),1);
freq_occur = zeros(length(abs_x_shifted),2);

abs_x = data_min(1):1:data_min(1)+length(freq_occur(:,1))-1;
abs_y = data_min(2):1:data_min(2)+length(freq_occur(:,2))-1;
%abs_z = data_min(3):1:data_min(3)+length(freq_occur(:,3))-1;
abs_x = abs_x/10000;
abs_y = abs_y/10000;
%Get Statistics for data
index = 0;
m_bar = mean(data);
P = var(data);

%Sum the frequency of occurance of each data point
for i = 1:length(data)
   
    val = data(i,:)+shift;
    
    for j = 1:2
        freq_occur(val(j),j) = freq_occur(val(j),j)+1;
    end
end

%Remove Zero Entries
freq_occur(freq_occur==0) = NaN;


freq_occur_x = freq_occur(:,1);
%Normalize the data using trapezoidal integration
for i = 1:2
    freq_occur_norm(:,i) = freq_occur(:,i)/trapz(freq_occur(:,i));
end  
%Plot Normalized data
plot(abs_x, freq_occur(:,1), '.b');
grid on;
hold on;

%%Plot theoretical Gaussian
abs_thX = min(data_min):1:max(data_max);
abs_thY = abs_thX;
%abs_thZ = abs_thX;

% gaussianX = 1./sqrt(2*pi.*P(1)).*exp(-0.5./P(1).*(abs_thX(1,:)' - m_bar(1)).^2);
% gaussianY = 1./sqrt(2*pi.*P(2)).*exp(-0.5./P(2).*(abs_thY(1,:)' - m_bar(2)).^2);
% gaussianZ = 1./sqrt(2*pi.*P(3)).*exp(-0.5./P(3).*(abs_thZ(1,:)' - m_bar(3)).^2);
% 
% plot(abs_thX, gaussianX, '--b', abs_thY, gaussianY, '--r', abs_thZ, gaussianZ, '--g');
% legend('X-Axis', 'Y-Axis', 'Z-Axis');
% title('Noise Measurements for L3G4200D 3-Axis Gyro')
% xlabel('Gyro Output');
% ylabel('Frequency of Occurence')


%%
clear all;
close all;
clc;

load('dataAccelAngles');
figure()
hold on;
x = 0:.1:10;
[mu,sigma,muci,sigmaci] = normfit(data);
y=normpdf(x,mu(1),sigma(1));
h = hist(data(:,1),50);
h = h./trapz(h);
bar(h)
hold on;
plot(x,y);
axis([0 10 0 4]);
