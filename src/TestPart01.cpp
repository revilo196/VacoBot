#include <NewPing.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include "config.h"
#include "VacESC.h"
#include "Drive.h"
#include "Sonar.h"
#include "Navigation.h"
#include <Arduino.h>

Navigation navi;
MPU6050 mpu;
HMC5883L mag;

VacESC myESC;
Sonar msonar;
Drive driver(MotorRPIN1,MotorRPIN2,MotorLPIN1,MotorLPIN2);

float heading = -999.9;

float rotationCheck;
unsigned long lastCheckTime;

bool isRot = false;
/*
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(0x1E);
void displaySensorDetails(void) {
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
float getHeading() {

  sensors_event_t event;
  mag.getEvent(&event);

  // Display the results (magnetic vector values are in micro-Tesla (uT))


  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI;

  #ifdef DEBUG
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  #endif

  return headingDegrees;
}
*/

// the setup function runs once when you press reset or power the board
void setup() {

   myESC.setup(VAC_PIN);
   driver.setup();
   msonar.setup();

   //Brush;
   pinMode(BRUSH_PIN,OUTPUT);

  #ifdef DEBUG
  Serial.begin(BAUD);
  #endif

  /* Initialise the sensor
  if(!mag.begin())
  {
    // There was a problem detecting the HMC5883 ... check your connections
    #ifdef DEBUG
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    #endif

    while(1);
  }*/

/*
  #ifdef DEBUG
  // Display some basic information on this sensor
  displaySensorDetails();
  #endif*/

  navi.setup();

}


float angleDiff(float unit1,float unit2) {

  float phi = fmod((unit2-unit1) , 360);
  float sign = 1;
  float result;

/*  if (!((phi >= 0 && phi <= 180) || ( phi <= -180 && phi >= -360))) {
        sign = 1;
    }*/

     if(unit1-unit2 >= 0 && unit1-unit2 <= 180) {
       sign = 1;
     } else {
       sign = -1;
     }

     if (unit1-unit2 <= -180 and unit1-unit2 >= -360) {
       sign = 1;
     } else {
       sign = -1;
     }

    if (phi > 180) {
        result = 360-phi;
    }  else {
        result = phi;
    }

    return (result*sign);


}

bool checkRotation(float adiff) {
  if(millis() - lastCheckTime< 200) {
    if(fabs(adiff) < rotationCheck) {
      rotationCheck = fabs(adiff);
      return true;
    } else {
      rotationCheck = fabs(adiff);
      return false;
    }
  }
}

// the loop function runs over and over again forever
void loop() {


  //Li-ION Voltage Control
  float cGes = (analogRead(CGES_PIN) * 2) * 0.00488;
  float c1 = (analogRead(C1_PIN)) * 0.00488;
  float c2 = cGes - c1;

#ifdef DEBUG_CELLS
  Serial.print("Gesamt:");
  Serial.println(cGes);
  Serial.print("C1:");
  Serial.println(c1);
  Serial.print("C2:");
  Serial.println(c2);
#endif

float headingNow;

msonar.update(millis());

navi.update(millis());

//init
if (heading < -400) {
 heading =  navi.getHeading();
 headingNow = heading;
 rotationCheck = 0;
 lastCheckTime = millis();
} else {
  headingNow = navi.getHeading();
}

//Only run if the Cells have enougth Power
  if(c1 > MIN_VOLT && c2 > MIN_VOLT) {

    //Enable Cleaning
    digitalWrite(BRUSH_PIN, HIGH);
    myESC.vacOn();


    if(!driver.hasQuery()) {

      //Simple AI
      //Sensor Input         !isRot -> Dont measure sensors when rotating
      if((msonar.getCm(0) < 20 || msonar.getCm(1) < 20 || msonar.getCm(2) < 20) && !isRot ) {
        isRot = true;
        heading = fmod ((heading +85.5),360);
      } else {

/*
        Serial.print(heading);
        Serial.print("\t");
        Serial.print(headingNow);
        Serial.print("\t");*/
        float adiff = angleDiff(headingNow,heading);


      //  Serial.print(adiff);
        if(adiff < -15 ) {
          driver.setRight();
      //    Serial.println("   RIGHT");
        } else  if (adiff > 15){
          driver.setLeft();
      //    Serial.println("   LEFT");
        } else if(adiff < -5 ) {
          driver.setRightLight();
      //    Serial.println("   RIGHT LIGHT");
        } else  if (adiff > 5 ){
          driver.setLeftLight();
      //    Serial.println("   LEFT LIGHT");

        } else {
          driver.setForward();
          isRot = false;

          if(navi.getBump(0) <= -1) {
      //      Serial.println("BUMP");

            driver.setBackward();
            driver.queryMove(DriveDir::LEFT, millis() + 1000);
            heading = fmod ((heading +85.5),360);
            rotationCheck = 85.5;
          }

      //    Serial.println("   FOARWARD");

        }

      }
  } else {
    //Update Driver if there is Query In the Driver
    driver.update(millis());
  }

  }
  else { // Turn every thing off if there is not enougth Power
    digitalWrite(BRUSH_PIN, LOW);
    driver.setStopp();
    myESC.vacOff();
    delay(50000);
  }





}
