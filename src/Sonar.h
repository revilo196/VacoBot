#ifndef SONAR
#define  SONAR

#include <Arduino.h>
#include <NewPing.h>

class Sonar {
private:

public:
  static void echoCheck();
  Sonar ();
  virtual ~Sonar () {};

  void oneSensorCycle() ;
  void setup() ;
  void update(unsigned long mils) ;
  int getCm(int i);

};

#endif
