
#include <Arduino.h>

enum DriveDir {
  STOPP,
  FOARWARD,
  BACKWARD,
  LEFT,
  RIGHT
};

class Drive {
private:

DriveDir direction;

unsigned long timer;
DriveDir next;

int M1APin;
int M1BPin;

int M2APin;
int M2BPin;

public:
  Drive (int m1aPin,  int m1bPin,  int m2aPin,  int m2bPin) {
    M1APin = m1aPin;
    M1BPin = m1bPin;
    M2APin = m2aPin;
    M2BPin = m2bPin;
    timer = 0;
    next = DriveDir::STOPP;
    direction = DriveDir::STOPP;
  }

  void setup() {
    pinMode(M1APin, OUTPUT);
    pinMode(M1BPin, OUTPUT);
    pinMode(M2APin, OUTPUT);
    pinMode(M2BPin, OUTPUT);
  }

  void queryMove(DriveDir dir, unsigned long time) {
    next = dir;
    timer = time;
  }

  bool hasQuery() {
    return timer != 0;
  }

  void update(unsigned long mil) {
    if(timer != 0) {

        if(timer <= mil) {
          setDir(next);
          timer = 0;
          next = DriveDir::STOPP;
        }

    }
  }

  void setStopp() {
    digitalWrite(M1APin, LOW);
    digitalWrite(M1BPin, LOW);
    digitalWrite(M2APin, LOW);
    digitalWrite(M2BPin, LOW);
    direction = DriveDir::STOPP;
  }

  void setForward() {
    digitalWrite(M1APin, HIGH);
    digitalWrite(M1BPin, LOW);
    digitalWrite(M2APin, HIGH);
    digitalWrite(M2BPin, LOW);
    direction = DriveDir::FOARWARD;

  }

  void setBackward() {
    digitalWrite(M1APin, LOW);
    digitalWrite(M1BPin, HIGH);
    digitalWrite(M2APin, LOW);
    digitalWrite(M2BPin, HIGH);
    direction = DriveDir::BACKWARD;

  }

  void setLeft() {
    digitalWrite(M1APin, HIGH);
    digitalWrite(M1BPin, LOW);
    digitalWrite(M2APin, LOW);
    digitalWrite(M2BPin, HIGH);
    direction = DriveDir::LEFT;
  }

  void setRight() {
    digitalWrite(M1APin, LOW);
    digitalWrite(M1BPin, HIGH);
    digitalWrite(M2APin, HIGH);
    digitalWrite(M2BPin, LOW);
    direction = DriveDir::RIGHT;
  }
  void setLeftLight() {
    digitalWrite(M1APin, HIGH);
    digitalWrite(M1BPin, LOW);
    digitalWrite(M2APin, LOW);
    digitalWrite(M2BPin, LOW);
    direction = DriveDir::LEFT;
  }

  void setRightLight() {
    digitalWrite(M1APin, LOW);
    digitalWrite(M1BPin, LOW);
    digitalWrite(M2APin, HIGH);
    digitalWrite(M2BPin, LOW);
    direction = DriveDir::RIGHT;
  }

  void  setDir(DriveDir dir) {
    switch (dir) {
      case DriveDir::STOPP:
      setStopp();
      break;
      case DriveDir::FOARWARD:
      setForward();
      break;
      case DriveDir::BACKWARD:
      setBackward();
      break;
      case DriveDir::LEFT:
      setLeft();
      break;
      case DriveDir::RIGHT:
      setLeft();
      break;
    }
  }

  virtual ~Drive () {

  }
};
