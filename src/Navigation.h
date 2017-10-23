#ifndef _NAVI_H
#define  _NAVI_H

#include "I2Cdev.h"
#include <Arduino.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//Time that takes the sensor to calibrate
//10 - 20 seconds
#define CAL_TIME  30000
#define MPU_ADDR  0x68
#define MAG_ADDR  0x1E
#define AVG_BUFFER_SIZE 20
#define PEAK_threshold 512
extern MPU6050 mpu;
extern HMC5883L mag;

class Navigation {

private:

  long aabiasRaw[3];
  long aabiasCount;
  int aabias[3];
  int aaMem[3][AVG_BUFFER_SIZE];
  int aaOutRaw[3];
  float aaOut[3];
  int current = 0;
  float gglast[3];

  const int threshold = PEAK_threshold;
  int aaPeak[3];

  float magoffset[3];
  float maglast[3];

  bool conectionError = false;
  int16_t mx, my, mz;

  // MPU control/status vars
  bool dmpReady = false;  // set true if DMP init was successful
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer

  // orientation/motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

  void updateCal();

public:

  void setup();
  void update(unsigned long mils);
  float getHeading(){
    return(ypr[0] + M_PI)/M_PI*180;
  }

  int getBump(int dir = 0) {
    return aaPeak[dir];
  }

  Navigation(){};
  virtual ~Navigation (){};
};



void Navigation::setup(){

  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    mag.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");


    conectionError = !mpu.testConnection() || !mag.testConnection();

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();


    mpu.setXGyroOffset(278);
    mpu.setYGyroOffset(-14);
    mpu.setZGyroOffset(110);
    mpu.setXAccelOffset(-1597); // 1688 factory default for my test chip
    mpu.setYAccelOffset(1280); // 1688 factory default for my test chip
    mpu.setZAccelOffset(472); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));

        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}

void Navigation::update(unsigned long mils){
  //MPU error
  if (!dmpReady) return;

  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();



  mag.getHeading(&mx, &my, &mz);

  // display tab-separated gyro x/y/z values
//  Serial.print("mag:\t");
//  Serial.print(mx); Serial.print("\t");
//  Serial.print(my); Serial.print("\t");
//  Serial.print(mz); Serial.print("\t");

// To calculate heading in degrees. 0 degree indicates North
  float heading = atan2(my, mx);
  if(heading < 0)
    heading += 2 * M_PI;
//  Serial.print("heading:\t");
//  Serial.println(heading * 180/M_PI);



  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if(mpuIntStatus & 0x02) {
        // wait for correct available data length,


        if(fifoCount < packetSize) return;

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        Serial.print(heading * 180/M_PI);
        Serial.print("\t");
        Serial.print(getHeading());
        Serial.print("\t");
        Serial.println(getHeading()  - (heading * 180/M_PI));


        if(mils < CAL_TIME ) {
          if(mils < 10000) {
            return;
          }
          updateCal();
          return;
        }

        aaMem[0][current] =   aaReal.x +17;
        aaMem[1][current] =   aaReal.y -15;
        aaMem[2][current] =   aaReal.z -280;

        current = (current + 1) % AVG_BUFFER_SIZE;

        aaOutRaw[0] = 0;
        aaOutRaw[1] = 0;
        aaOutRaw[2] = 0;
        for(int i = 0; i < AVG_BUFFER_SIZE; i++) {
          aaOutRaw[0] += aaMem[0][i] ;//* (AVG_BUFFER_SIZE-1);
          aaOutRaw[1] += aaMem[1][i] ;//* (AVG_BUFFER_SIZE-1);
          aaOutRaw[2] += aaMem[2][i] ;//* (AVG_BUFFER_SIZE-1);
        }

        aaOut[0] = (float)aaOutRaw[0] / (float)(AVG_BUFFER_SIZE);//*AVG_BUFFER_SIZE+1)/2;
        aaOut[1] = (float)aaOutRaw[1] / (float)(AVG_BUFFER_SIZE);//*AVG_BUFFER_SIZE+1)/2;
        aaOut[2] = (float)aaOutRaw[2] / (float)(AVG_BUFFER_SIZE);//*AVG_BUFFER_SIZE+1)/2;



        aaPeak[0] = aaOut[0] / threshold;
        aaPeak[1] = aaOut[1] / threshold;
        aaPeak[2] = aaOut[2] / threshold;
/*
        Serial.print("vreal\t");
        Serial.print(  aaPeak[0] );
        Serial.print("\t");
        Serial.print(  aaPeak[1]);
        Serial.print("\t");
        Serial.println( aaPeak[2]);
*/


  }

}

void Navigation::updateCal() {

  aabiasRaw[0] += aaReal.x;
  aabiasRaw[1] += aaReal.y;
  aabiasRaw[2] += aaReal.z;
  aabiasCount++;

  aabias[0] = (float)aabiasRaw[0] / (float)aabiasCount;
  aabias[1] = (float)aabiasRaw[1] / (float)aabiasCount;
  aabias[2] = (float)aabiasRaw[2] / (float)aabiasCount;
}


#endif // _NAVI_H
