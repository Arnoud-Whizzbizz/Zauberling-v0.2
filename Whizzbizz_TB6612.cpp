//****************************************************************************************************
//
// Whizzbizz.com - Arnoud van Delden - november/december 2021
//
// Dedicated class for the TB6612FNG H-Bridge Output Driver based on the SparkFun TB6612FNG library
// Adapted because the 'Zauberling' needs to have individual access to all four outputs
//
// ***************************************************************************************************

#include "Whizzbizz_TB6612.h"
#include <Arduino.h>

Output::Output(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin) {
  In1 = In1pin;
  In2 = In2pin;
  PWM = PWMpin;
  Standby = STBYpin;
  Offset = offset;
  
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(Standby, OUTPUT);
}

void Output::set(int speed) {
  digitalWrite(Standby, HIGH);
  speed = speed * Offset;
  if (speed>=0) {
    if (speed==0) {
      // Set both control lines to LOW
      digitalWrite(In1, LOW);
      digitalWrite(In2, LOW);
    } else {
      digitalWrite(In1, HIGH);
      digitalWrite(In2, LOW);
      analogWrite(PWM, speed);
    }
  } else {
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
    analogWrite(PWM, -speed);
  }
}

void Output::brake() {
   // Set both control lines to HIGH
   digitalWrite(In1, HIGH);
   digitalWrite(In2, HIGH);
   analogWrite(PWM,0);
}

void Output::clear() {
   digitalWrite(In1, LOW);
   digitalWrite(In2, LOW);
}

void Output::standby() {
   digitalWrite(Standby, LOW);
}