function [gx gy gz] = readGyro(out)
       
    %Request Gyro read from Arduino
    fprintf(out.s, 'G');
    
    %Read values from Arduino in deg/sec
    gx = fscanf(out.s,'%f');
    gy = fscanf(out.s,'%f');
    gz = fscanf(out.s,'%f');
    
end