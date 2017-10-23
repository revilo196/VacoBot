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


//Sammelt und Verarbeitet Ergebisse der Sensoren
Navigation navi;
//Beschleunigugs und Rotations Sensor
MPU6050 mpu;
//Magnetfeldsensor
HMC5883L mag;

//Steuert den
VacESC myESC;
//Steuert die Sensoren
Sonar msonar;
//Steuert die Motoren
Drive driver(MotorRPIN1,MotorRPIN2,MotorLPIN1,MotorLPIN2);

//Rotation des Roboters im Raum in [degrees]
float heading = -999.9;

//Variablen um zu püfen ob der Roboter sich festgefahren hat
float rotationCheck;
unsigned long lastCheckTime;


bool isRot = false;

// the setup function runs once when you press reset or power the board
void setup() {

   myESC.setup(VAC_PIN);
   driver.setup();
   msonar.setup();

  //Brush;
  pinMode(BRUSH_PIN,OUTPUT);

  //Debug verbindung zum Computer
  #ifdef DEBUG
  Serial.begin(BAUD);
  #endif

  navi.setup();
}

/**
 * angleDiff(float unit1,float unit2);
 * Berechnert die differenz von 2 winkeln
 * unter der Berücksichtung das 0° = 360° ist
**/
float angleDiff(float unit1,float unit2) {

  float phi = fmod((unit2-unit1) , 360);
  float sign = 1;
  float result;

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

//überprüft ob der Roboter sich wie gewollt gedreht hat
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

  //Hier wird gemessen wie voll der akku noch ist
  //Li-ION Voltage Control
  float cGes = (analogRead(CGES_PIN) * 2) * 0.00488;
  float c1 = (analogRead(C1_PIN)) * 0.00488;
  float c2 = cGes - c1;

//Debug output zum PC
#ifdef DEBUG_CELLS
  Serial.print("Gesamt:");
  Serial.println(cGes);
  Serial.print("C1:");
  Serial.println(c1);
  Serial.print("C2:");
  Serial.println(c2);
#endif


//Die akuelle rotation des Roboters
float headingNow;

msonar.update(millis());
navi.update(millis());


//init
if (heading < -400) {
  //Wird nur beim ersten mal ausgefürt
 heading =  navi.getHeading();
 headingNow = heading;
 rotationCheck = 0;
 lastCheckTime = millis();
} else {
  //hole die akuelle  rotation des Roboters vom navi
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
  else {
    // Turn every thing off if there is not enougth Power
    digitalWrite(BRUSH_PIN, LOW);
    driver.setStopp();
    myESC.vacOff();
    delay(50000);
  }





}
