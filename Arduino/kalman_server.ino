#include <Wire.h>
#include <L3G.h>
#include <ADXL345.h>

#define ACC (0xA7>>1)
#define A_TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)

/*define gyro */
L3G gyro;
ADXL345 accel;
int mode = -1;
//int16_t ax, ay, az;
int ax, ay, az;

//double a_conversion = 1;
double a_conversion = 3.9/100; //converts accelerometer output to m/s^2
//double g_conversion = PI/180;
double g_conversion = 8.75/1000.0;

void setup() {
             
  Serial.begin(115200);
  Wire.begin();
  /* initialize and enable gyro */
  gyro.init();
  gyro.enableDefault();
  
  /* initialize and enable accelerometer */
  accel.powerOn();
  //initAcc();
  
  /* begin handshake with Matlab */
  Serial.println('a');
  char a = 'b';
  while (a !='a')
  {
   a = Serial.read(); 
  }
  
}

void loop() {
  
  if (Serial.available() >0) {

    /* whatever is available from the serial is read here    */
    mode = Serial.read();
    
    switch (mode) {
      
      case 'G':
               gyro.read();
	       Serial.println((int)gyro.g.x*g_conversion); //readings in degrees/second
	       Serial.println((int)gyro.g.y*g_conversion);
	       Serial.println((int)gyro.g.z*g_conversion);
               mode = -1;
                                                break;
      case 'A':
                accel.readAccel(&ax, &ay, &az);
                Serial.println(ax*a_conversion);
	        Serial.println(ay*a_conversion);  //Y axis is reversed in current setup to match gyroscope axis
	        Serial.println(az*a_conversion);
                mode = -1;                      break;
                
      case 'B': //Read Both
                accel.readAccel(&ax, &ay, &az);
                Serial.println(-ax*a_conversion);
	        Serial.println(ay*a_conversion);  //Y axis is reversed in current setup to match gyroscope axis
	        Serial.println(az*a_conversion);
                gyro.read();
	        Serial.println((int)gyro.g.x*g_conversion); //readings in degrees/second
	        Serial.println((int)gyro.g.y*g_conversion);
	        Serial.println((int)gyro.g.z*g_conversion);
                mode = -1;                      break;          
                
      //default: 
      //           Serial.println('x');          break; 
    }
  }
}
