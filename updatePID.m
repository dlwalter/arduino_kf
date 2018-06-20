function [PWM error] = updatePID(PWM, tgtVal, currentVal, last_error)
    Kp = 0.4;
    Kd = 1;
     
    error = tgtVal - currentVal;
    pidTerm = (Kp*error) + Kd* (error - last_error);
    
    PWM = PWM + pidTerm;
end