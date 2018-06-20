function [roll pitch] = getAccelAngles(A, offset)
    %Read Accelerometer axis and return pitch and roll of device.
    
    [ax ay az] = readAccel(A);
    
    %Using Gravitational Force components, find pitch and roll
    roll = atan2(-ay, az)*180.0/pi()-offset(1);
    pitch = atan2(ax, sqrt(ay^2+az^2))*180/pi()-offset(2);
end