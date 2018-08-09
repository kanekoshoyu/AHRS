#include "quaternionFilters.h"
#include "MPU9250.h"
#include <SoftwareSerial.h>

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

MPU9250 myIMU;
SoftwareSerial Bluetooth(10, 11);

void setup(){
  Wire.begin();
  Bluetooth.begin(9600);
}

void loop(){
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01){  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Convert accleration into actual g's, depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];
    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Convert gyro into actual deg/s, depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    // Read the x/y/z adc values
    myIMU.readMagData(myIMU.magCount);  
    myIMU.getMres();
    myIMU.magbias[0] = +470.;
    myIMU.magbias[1] = +120.;
    myIMU.magbias[2] = +125.;
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -myIMU.magbias[2];
  }
  myIMU.updateTime();
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD, myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my, myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS){
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
      myIMU.count = millis();
  }else{//(AHRS)
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500){
      // Print acceleration values in milligs!
      Bluetooth.print((int)1000*myIMU.ax);
      Bluetooth.print(' ');
      Bluetooth.print((int)1000*myIMU.ay);
      Bluetooth.print(' ');
      Bluetooth.print((int)1000*myIMU.az);
      Bluetooth.print(' ');

      /*
      Bluetooth.print(*getQ());
      Bluetooth.print(' ');
      Bluetooth.print(*(getQ() + 1));
      Bluetooth.print(' ');
      Bluetooth.print(*(getQ() + 2));
      Bluetooth.print(' ');
      Bluetooth.println(*(getQ() + 3));
      */
      
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;
      myIMU.yaw   -= 8.5;
      myIMU.roll  *= RAD_TO_DEG;

      
      Bluetooth.print(myIMU.pitch);
      Bluetooth.print(' ');
      Bluetooth.print(myIMU.yaw);
      Bluetooth.print(' ');
      Bluetooth.println(myIMU.roll);

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)
}
