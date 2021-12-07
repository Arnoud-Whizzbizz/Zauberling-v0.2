//****************************************************************************************************
//
// Whizzbizz.com - Arnoud van Delden - november/december 2021
//
// Dedicated class for the TB6612FNG H-Bridge Output Driver based on the SparkFun TB6612FNG library
// Adapted because the 'Zauberling' needs to have individual access to all four outputs
//
// ***************************************************************************************************

#ifndef WHIZZBIZZ_TB6612_h
#define WHIZZBIZZ_TB6612_h

#include <Arduino.h>

class Output {
  public:
    // Constructor, basically sets up the control pins
    Output(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin);

    // Drive in direction given by sign, at output value defined by parameter 'speed'
    void set(int speed);

	  // Stops output by setting both control pins HIGH, may be used as a motor brake
    void brake();

    // Flushes output by clearing both control pins to LOW
    void clear();

    // Set the chip to standby mode. The drive function takes it out of standby
    // (forward, back, left, and right all call drive)
    void standby();

  private:
    //variables for the 2 inputs, PWM input, Offset value, and the Standby pin
    int In1, In2, PWM, Offset,Standby;
};

#endif