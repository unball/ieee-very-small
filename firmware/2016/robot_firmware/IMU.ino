#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

int ax, ay, az;
int gx, gy, gz;

void IMUSetup() {
  Serial.println("teste1");
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif    
    
    accelgyro.initialize();
    accelgyro.testConnection();
}

void getIMUinfo(int *imu_data) {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  int data[6] = {ax, ay, az, gx, gy, gz};
  int i;
  for (i=0;i<6;i++)
    imu_data[i] = data[i];
}

