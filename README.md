# arduino_kf


![Arduino Test Stand](https://raw.githubusercontent.com/dlwalter/arduino_kf/master/test_stand.jpg)

This project runs a matlab serial server on an Arduino Uno which gathers inertial data to pass back to the host computer and operates the DC motor to generate a desired attitude profile for truth comparison.

Run the project with the included script KalmanGUI2.m.  This script relies on the program kalman_server.ino to be running on the Arduino and connected to the host over serial.

The figure below shows the error between the Kalman Filter estimate and the accelerometer measurement:

![Error Plot](https://raw.githubusercontent.com/dlwalter/arduino_kf/master/Figures/ErroRealTime.png)

The figure below shows the diagonals of the Error State Covariance Matrix:

![Error Plot](https://raw.githubusercontent.com/dlwalter/arduino_kf/master/Figures/RealTimeCovar.png)