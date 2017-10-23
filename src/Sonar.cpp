
#include "Sonar.h"

#define SONAR_NUM     3 // Number of sensors.
#define MAX_DISTANCE 300 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
uint8_t currentSensor = 0;
NewPing sonar[SONAR_NUM] = {     // Sensor object array.
NewPing(6, 8, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
NewPing(5, 7, MAX_DISTANCE),
NewPing(4, A3, MAX_DISTANCE),
};

unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.

Sonar::Sonar() {

}


// If ping received, set the sensor distance to array.
void Sonar::echoCheck() {

  if (sonar[currentSensor].check_timer()){

  unsigned int  us = sonar[currentSensor].ping_result;

  //Only use received echos
  if(us > 0) {
      cm[currentSensor] = ((cm[currentSensor]) + (sonar[currentSensor].ping_result / US_ROUNDTRIP_CM))/2;
    }

  }

}

// Sensor ping cycle complete, do something with the results.
// The following code would be replaced with your code that does something with the ping results.
void Sonar::oneSensorCycle() {



/*
    for (uint8_t i = 0; i < SONAR_NUM; i++) {
      Serial.print(i);
      Serial.print("=");
      Serial.print(cm[i]);
      Serial.print("cm ");
    }
    Serial.println();
*/
}

void Sonar::setup() {
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void Sonar::update(unsigned long mils) {

  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (mils >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      //cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }

}

int Sonar::getCm(int i) {
  return cm[i];
}
