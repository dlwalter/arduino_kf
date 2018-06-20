function [ax ay az] = readAccel(out)
       
    %Request Accelerometer read from Arduino
    fprintf(out.s, 'A');
    
    %Read values from Arduino in m/s^2
    ax = fscanf(out.s,'%f');
    ay = fscanf(out.s,'%f');
    az = fscanf(out.s,'%f');
    
end