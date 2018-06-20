function [roll pitch gx gy gz] = readBoth(out, offset)
       
    %Request Gyro read from Arduino
    fprintf(out.s, 'B');
    
    %Read values from Arduino
    ax = fscanf(out.s,'%f');
    ay = fscanf(out.s,'%f');
    az = fscanf(out.s,'%f');
    gx = fscanf(out.s,'%f');
    gy = fscanf(out.s,'%f');
    gz = fscanf(out.s,'%f');
    
    %Using Gravitational Force components, find pitch and roll
    roll = atan2(-ay, az)*180.0/pi()-offset(1);
    pitch = atan2(ax, sqrt(ay^2+az^2))*180/pi()+offset(2);
    
end