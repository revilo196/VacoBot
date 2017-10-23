
#include <Arduino.h>
#include <Servo.h>

#define ESC_OFF_SPEED 50
#define ESC_ON_SPEED 60

class VacESC {
private:

  Servo myservo;
  int myspeed = 0;
  int port;



public:
  VacESC () : myservo() {

  }

  void setup(int pin) {
    myservo.attach(pin);
    arm();
    vacOn();
    delay(1000);
    vacOff();
    delay(1000);
    vacOn();
    delay(1000);
    vacOff();
    delay(1000);
  }

  void arm() {
    // arm the speed controller, modify as necessary for your ESC
    setSpeed(0);
    delay(2000); //delay 1 second,  some speed controllers may need longer
    setSpeed(100);
    delay(1000);
    setSpeed(0);
    delay(3000);
  }

  void setSpeed(int speed) {
    int angle = map(speed, 0, 100, 0, 180);
    myservo.write(angle);
    myspeed = speed;
  }

  void  vacOn() {
    setSpeed(90);
  }

  void vacOff() {
    setSpeed(30);
  }

  virtual ~VacESC () {}
};
