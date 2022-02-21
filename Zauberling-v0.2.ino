// Zauberling version v0.2
// Whizzbizz.com - Arnoud van Delden - november 2021 ~ februari 2022
//
// 'Zauberling' is build with a Arduino Pro Mini Atmega328P 5V 16Mhz board.
// Sketch to be uploaded with FTDI FT232RL USB To TTL Serial IC Adapter Converter
// or other USB to TTL interface (e.g. Arduino Uno with chip removed)
//
// The modified TB6612 lib is based on: https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library
// Display 128x64 SSD1306 OLED - AdaFruit outputr: https://learn.adafruit.com/adafruit-gfx-graphics-library/
//
#include <Adafruit_SSD1306.h>
#include "Whizzbizz_TB6612.h" // Output lib for TB6612FNG enabling control over each output individually

// Function 12 and 13 are test routines for various external I2C devices/sensors, outcomment defines if applicable 
//#define FUNC12_NUNCHUK   // Testfunction to display I2C nunchuck values
//#define FUNC13_COLORSENSOR // I2C color sensor TCS34725 connected

#ifdef FUNC12_NUNCHUK
  #include "ArduinoNunchuk.h"   // Test 2nd device on I2C bus only....
  ArduinoNunchuk nunchuk = ArduinoNunchuk();
#endif

#ifdef FUNC13_COLORSENSOR
  #include "Adafruit_TCS34725.h"
  Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
  #define MAX_COLOR_OFFSET 10  // the maximum diff you found between readings
  typedef struct {  // Struct to hold raw color data 
     int red;
     int green;
     int blue;
     int clr;
  } Color;

  // Default color values. Please calibrate depending on sensor circumstances...
  Color colorRed    = {48, 23, 24, 92 };  // Red
  Color colorGreen  = {39, 50, 33, 120 }; // Green
  Color colorBlue   = {30, 42, 57, 126};  // Blue
  Color colorYellow = {87, 64, 36, 185};  // Yellow
#endif

// TB6612 pins definition
#define AIN1 2   // Buffered/inverted AIN1 = digital OUT 1
#define AIN2 3   // Buffered/inverted AIN2 = digital OUT 2
#define PWMA 5
#define BIN1 10  // Buffered/inverted BIN1 = digital OUT 4
#define BIN2 11  // Buffered/inverted BIN2 = digital OUT 3
#define PWMB 6
#define STBY 13

// Inputs, pos/neg logic switch and potmeters
#define IN1            A6
#define IN2            A7
#define IN3            A0
#define IN4            A1
#define LOGIC_MODE     12
#define POTMETER1      A2
#define POTMETER2      A3
#define MINREG         0     // Minimal analogue speed potmeter value
#define MAXREG         1023  // Maximal analogue potmeter value
#define DEFAULT_TRESH  450   // Detects >4.5 volt as HIGH
#define TRESHOLD_MAX   50    // Maximal level of trigger/detection level or theshold
#define DELAY_MAX      5000  // Maximal delay after sensor trigger or pulse width of monoflop in ms (0 ~ 5 sec)
#define SPEED_STEPS    30    // Step value for smooth motor speed
#define MAX_VALUE      255   // Used to output logic on the motor-/outputr-outputs

// Inputs used to read the DIP-switches
// Only read as seperate lines during setup, during program all lines HIGH means 'trigger' (seperate push button on front)
#define DIP1  4
#define DIP2  7
#define DIP3  8
#define DIP4  9

bool logging = true;          // Serial logging during development...
int currentOutputSpeed;
bool currentOutputDir = true; // true=forward(CW), false=backwards(CCW)
uint16_t potValue1;           // Value read from potentiometer 1
uint16_t potValue2;           // Value read from potentiometer 2
uint8_t sensorTresh1;         // Analog trigger threshold value of potentiometer 1
uint8_t sensorTresh2;         // Analog trigger threshold value of potentiometer 2
int timeDelay;                // Smooth motor delay or pulse width of monoflop in ms
uint8_t dipSwitches = 0;
uint8_t trigButton;           // Reads DIP values during program to signal the trigger being pressed
uint8_t trigMask;             // Configs per function which INputs are triggered by the override trigger button

// Initializing TB6612FNG driver and SSD1306 screen
Output Out1 = Output(AIN1, AIN2, PWMA, 1, STBY);
Output Out2 = Output(BIN1, BIN2, PWMB, 1, STBY);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

uint16_t inValue;
bool in1Active;
bool in1TriggerUsed = true;
bool in2Active;
bool in2TriggerUsed = true;
bool in3Active;
bool in3TriggerUsed = true;
bool in4Active;
bool in4TriggerUsed = true;
bool trigActive;
bool trigTriggerUsed = true;

bool negativeLogic = true; // Default mode is negative logic
bool runningTimer = false; // Used for e.g. the MonoFlop
bool Q_FF = false;         // Flipflop output state
uint8_t outValue = 0;      // Generic output value for multiple use

// ----------------------------------------------------------------------------------------------------------------

void setup() {
  if (logging) Serial.begin(38400);
  
  // Init display and draw input-/output boxes...
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  
  // Set pin-modes and read sensor defaults...
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(IN3, INPUT);
  pinMode(IN4, INPUT);
  pinMode(LOGIC_MODE, INPUT_PULLUP);
  pinMode(POTMETER1, INPUT);
  pinMode(POTMETER2, INPUT);

  // Read program DIP-switches...
  pinMode(DIP1, INPUT_PULLUP);
  pinMode(DIP2, INPUT_PULLUP);
  pinMode(DIP3, INPUT_PULLUP);  
  pinMode(DIP4, INPUT_PULLUP);
  dipSwitches = readdipSwitches();

  // Check positive/negative logic
  display.setTextSize(1);
  display.setTextColor(WHITE,BLACK);
  display.setCursor(64-(14*6)/2,13);
  if (digitalRead(LOGIC_MODE)==HIGH) {
    negativeLogic = false;
    display.println(F("POSITIVE LOGIC"));
  } else {
    negativeLogic = true;
    display.println(F("NEGATIVE LOGIC"));    
  }

  // Only for functions that need explicit logic level settings (i.e. the 'Basic Program')...
  //logSerial("dipSwitches="+(String)dipSwitches);
  switch (dipSwitches) { // Switch different usage of potentionmeter...
    case 0: 
      // Basic program negative logic...
      display_function(F("OUT-TOGGLE"));
      trigMask = 1;
      drawInputOutputBoxes(false); // Two analogue motor speed bars...
      break;
    case 1: 
      // 1*4 input AND/NAND gate
      display_function(F("1*4 (N)AND"));
      trigMask = 15;
      drawInputOutputBoxes(true); // Show four digital ouput boxes...
      break;
    case 2: 
      // 2*2 input AND/NAND gate
      display_function(F("2*2 (N)AND"));
      trigMask = 15;
      drawInputOutputBoxes(true); // Show four digital ouput boxes...
      break;
    case 3:  
      // 1*4 input OR/NOR gate
      display_function(F("1*4 (N)OR"));
      trigMask = 15;
      drawInputOutputBoxes(true); // Show four digital ouput boxes...
      break;
    case 4:
      // 2*2 input OR/NOR gate
      display_function(F("2*2 (N)OR"));
      trigMask = 15;
      drawInputOutputBoxes(true); // Show four digital ouput boxes...
      break;     
    case 5:
      // 1*4 input XOR/XNOR gate
      display_function(F("1*4 X(N)OR"));
      trigMask = 15;
      drawInputOutputBoxes(true); // Show four digital ouput boxes...
      break;
    case 6:
      // 2*2 input XOR/XNOR gate
      display_function(F("2*2 X(N)OR"));
      trigMask = 15;
      drawInputOutputBoxes(true); // Show four digital ouput boxes...
      break; 
    case 7:
      // SR-flipflop
      display_function(F("SR-FLPFLP"));
      trigMask = 2; // CLK only
      drawInputOutputBoxes(true); // Show four digital ouput boxes...
      break;
    case 8:
      // Monoflop configurable pulse time
      trigMask = 2; // EDGE only
      display_function(F("MONOFLOP"));
      drawInputOutputBoxes(true); // Show four digital ouput boxes...
      break;   
    case 9:
      // Counter...
      display_function(F("2*COUNTER"));
      trigMask = 15;
      drawInputOutputBoxes(true); // Show four digital ouput boxes...
      break;
    case 10:
      // Oscillator...
      display_function(F("PULSE GEN"));
      trigMask = 2;  // EDGE only
      drawInputOutputBoxes(true); // Show four digital ouput boxes...
      break;
    case 11:
      display_function(F("SEQUENCE"));
      trigMask = 2;  // EDGE only
      drawInputOutputBoxes(true); // Show four digital ouput boxes...
      break;   
    case 12:
      trigMask = 1;
#ifdef FUNC12_NUNCHUK
      display_function(F("NUNCHUK"));
      drawInputOutputBoxes(false); // Two analogue speed bars...      
      nunchuk.init();
#else
      display_function(F("B-DECODE"));
      trigMask = 1; // CLK
      drawInputOutputBoxes(true); // Show four digital ouput boxes...
#endif
      break;
    case 13:
      trigMask = 1;
      drawInputOutputBoxes(true); // Show four digital ouput boxes...      
#ifdef FUNC13_COLORSENSOR
      if (!tcs.begin()) {
        display_function(F("NO SENSOR"));
        //Serial.println("NO sensor");
        display.setTextSize(1);
        display.setTextColor(WHITE,BLACK);
        display.setCursor(64-(17*6)/2,13);
        display.println(F("No TCS34725 found"));
        display.display();
        while (1);
      }
      display_function(F("RGBY COLOR"));
#else
      display_function(F("FUNC 13"));
#endif
      break;
    case 14:
      // Two speed-regulated motors
      display_function(F("CONTROLLER"));
      trigMask = 2;  // EDGE only
      drawInputOutputBoxes(false); // Two analogue speed bars...      
      break;   
    case 15: 
      // Output/Out demo (optionally connected motors)...
      trigMask = 15;
      display_function(F("MOTOR DEMO"));
      drawInputOutputBoxes(false); // Two analogue speed bars...
      break;
  }
  
  // Init motors/outputs
  currentOutputSpeed = 0;
  currentOutputDir = true; // Forward/CW

  // Read potmeters an init tresholds and inputs based on the logic mode...
  initTresholds();
  inValue = analogRead(IN1);
  if (inValue < (sensorTresh1*10))
    in1Active = negativeLogic;
  else
    in1Active = !negativeLogic;
  inValue = analogRead(IN2);
  if (inValue < (sensorTresh1*10))
    in2Active = negativeLogic;
  else
    in2Active = !negativeLogic;
  inValue = analogRead(IN3);
  if (inValue < (sensorTresh2*10))
    in3Active = negativeLogic;
  else
    in3Active = !negativeLogic;
  inValue = analogRead(IN4);
  if (inValue < (sensorTresh2*10))
    in4Active = negativeLogic;
  else
    in4Active = !negativeLogic;
    
  logSerial(F("Setup OK..."));
}

void loop() {
  initTresholds(); // Read potmeters an init tresholds. Routines that claim the loop must call this themselves!
  
  // Service routines...
  switch (dipSwitches) {
    default:
    case 0:
      function0();
      break;
    case 1:
      function1();
      break;
    case 2:
      function2();
      break;   
    case 3:
      function3();
      break;
    case 4:
      function4();
      break;
    case 5:
      function5();
      break;   
    case 6:
      function6();
      break;
    case 7:
      function7();
      break;
    case 8:
      function8();
      break;   
    case 9:
      function9();
      break;
    case 10:
      function10();
      break;
    case 11:
      function11();
      break;   
    case 12: 
      function12();
      break;
    case 13: 
      function13();
      break;
    case 14: 
      function14();
      break;   
    case 15:
      function15();
  }
}

void initTresholds() {
  // Reads potmeters and sets thesholds. Called once each loop(). Any (or both) tresholds may be overloaded by service routines before calling readShowInputs()
  
  // Read analog potmeter values, used as trigger level, hysteresis and/or time delay or pulse width for functions
  // potValues are global, so functions may use these otherwise as long as sensorTresh1 and sensorTresh2 are sensible overloaded
  potValue1 = analogRead(POTMETER1);
  potValue2 = analogRead(POTMETER2);
  sensorTresh1 = round10(map(potValue1, MINREG, MAXREG, 1, TRESHOLD_MAX)); 
  sensorTresh2 = round10(map(potValue2, MINREG, MAXREG, 1, TRESHOLD_MAX));
}

void readShowInputs() {
  // Reads inputs. Called from the respective service routines...
  // Read inputs. Default: IN1-IN2 use sensorTresh1, IN3-IN4 use sensorTresh2
  inValue = analogRead(IN1);
  if ( ((inValue < (sensorTresh1*10)) && negativeLogic) ||
       ((inValue > (sensorTresh1*10)) && !negativeLogic) ) {
    in1Active = true;
  } else {
    in1Active = false;
    in1TriggerUsed = false;
  }
  inValue = analogRead(IN2);
  if ( ((inValue < (sensorTresh1*10)) && negativeLogic) ||
       ((inValue > (sensorTresh1*10)) && !negativeLogic) ) {
    in2Active = true;
  } else {
    in2Active = false;
    in2TriggerUsed = false;
  }
  inValue = analogRead(IN3);
  if ( ((inValue < (sensorTresh2*10)) && negativeLogic) ||
       ((inValue > (sensorTresh2*10)) && !negativeLogic) ) {
    in3Active = true;
  } else {
    in3Active = false;
    in3TriggerUsed = false;
  }
  inValue = analogRead(IN4);
  if ( ((inValue < (sensorTresh2*10)) && negativeLogic) ||
       ((inValue > (sensorTresh2*10)) && !negativeLogic) ) {
    in4Active = true;
  } else {
    in4Active = false;
    in4TriggerUsed = false;
  }

  // Read trigger button on front, for manual triggering
  trigButton = readdipSwitches();
  if (dipSwitches!=15 && trigButton==15) {
    trigActive = true;
  } else {
    trigActive = false;
    trigTriggerUsed = false;
  }

  // Show feedback about input states...
  showInput( 0, (in1Active || (trigActive && (trigMask & (uint8_t)1) )));
  showInput( 1, (in2Active || (trigActive && (trigMask & (uint8_t)2) )));
  showInput( 2, (in3Active || (trigActive && (trigMask & (uint8_t)4) )));
  showInput( 3, (in4Active || (trigActive && (trigMask & (uint8_t)8) )));
  display.display();
}

// ----------------------------------------------------------------------------------------------------------------

void function0() {  // Output/Base program
  // Base output/motor program...
  // IN1=start CW or toggle, IN2=start CCW or toggle, IN3=start or stop (toggle)
  sensorTresh2 = sensorTresh1;
  //sensorTresh1 = sensorTresh2 = TRESHOLD_MAX/2; // Fixed: Both pots are being used for something else... 
  timeDelay = round10(map(potValue2, MINREG, MAXREG, 0, DELAY_MAX));
  displayValues(round10(map(potValue1, MINREG, MAXREG, 1, TRESHOLD_MAX)), "P:", "", timeDelay, "", " ms");
  printInputTags("P-CLK", "FRWD", "REV", "");
  readShowInputs();

  if (trigActive) // Manual test: set only IN1 to active...
    in1Active = true;

  // IN1 is the On/Off switch...
  if (in1Active && !in1TriggerUsed) { // Only once for each trigger slope...
    // Toggle motor on/off...
    if (currentOutputSpeed>0)
      setMotorSpeedFunction0(currentOutputDir, 0, true);
    else
      setMotorSpeedFunction0(currentOutputDir, MAX_VALUE, true);
    in1TriggerUsed = true;
  }

  // IN2, IN3 and IN4 function as directional toggles...
  if (currentOutputSpeed>0) { // Already running, toggle based on sensor logic...
    if (in2Active && !in2TriggerUsed) { // Check sensor on IN2, only once for each trigger slope...
      setMotorSpeedFunction0(true, currentOutputSpeed, true);
      in2TriggerUsed = true;
    }
    if (in3Active && !in3TriggerUsed) { // Check sensor on IN3, only once for each trigger slope...
      setMotorSpeedFunction0(false, currentOutputSpeed, true);
      in3TriggerUsed = true;
    } 
    if (in4Active && !in4TriggerUsed) { // Check sensor on IN4, only once for each trigger slope...
      setMotorSpeedFunction0(true, currentOutputSpeed, true);
      in4TriggerUsed = true;
    }   
  }
}

void function1() { // 1*4 input AND/NAND gate
  displayValues(map(potValue1, MINREG, MAXREG, 1, TRESHOLD_MAX), "P1:", "", map(potValue2, MINREG, MAXREG, 1, TRESHOLD_MAX), "P2:", "");
  readShowInputs();
  
  if (trigActive) // Manual test: set all inputs to active...
    in1Active = in2Active = in3Active = in4Active = true;

  bool outputState12 = (in1Active && in2Active && in3Active && in4Active);
  bool outputState34 = outputState12;
  digitalOut(outputState12, !outputState12, outputState34, !outputState34);    
}

void function2() { // 2*2 input AND/NAND gate
  displayValues(map(potValue1, MINREG, MAXREG, 1, TRESHOLD_MAX), "P1:", "", map(potValue2, MINREG, MAXREG, 1, TRESHOLD_MAX), "P2:", "");
  readShowInputs();
  
  if (trigActive) // Manual test: set all inputs to active...
    in1Active = in2Active = in3Active = in4Active = true;

  bool outputState12 = (in1Active && in2Active);
  bool outputState34 = (in3Active && in4Active);
  digitalOut(outputState12, !outputState12, outputState34, !outputState34);
}

void function3() { // 1*4 input OR/NOR gate
  displayValues(map(potValue1, MINREG, MAXREG, 1, TRESHOLD_MAX), "P1:", "", map(potValue2, MINREG, MAXREG, 1, TRESHOLD_MAX), "P2:", "");
  readShowInputs();
  
  if (trigActive) // Manual test: set all inputs to active...
    in1Active = in2Active = in3Active = in4Active = true;

  bool outputState12 = (in1Active || in2Active || in3Active || in4Active);
  bool outputState34 = outputState12;
  digitalOut(outputState12, !outputState12, outputState34, !outputState34);
}

void function4() { // 2*2 input OR/NOR gate
  displayValues(map(potValue1, MINREG, MAXREG, 1, TRESHOLD_MAX), "P1:", "", map(potValue2, MINREG, MAXREG, 1, TRESHOLD_MAX), "P2:", "");
  readShowInputs();
  
  if (trigActive) // Manual test: set all inputs to active...
    in1Active = in2Active = in3Active = in4Active = true;

  bool outputState12 = (in1Active || in2Active);
  bool outputState34 = (in3Active || in4Active);
  digitalOut(outputState12, !outputState12, outputState34, !outputState34);
}

void function5() { // 1*4 input XOR/XNOR gate
  displayValues(map(potValue1, MINREG, MAXREG, 1, TRESHOLD_MAX), "P1:", "", map(potValue2, MINREG, MAXREG, 1, TRESHOLD_MAX), "P2:", "");
  readShowInputs();
  
  if (trigActive) // Manual test: set all inputs to active...
    in1Active = in2Active = in3Active = in4Active = true;

  bool outputState12 = ((in1Active && in2Active && in3Active && in4Active) || (!in1Active && !in2Active && !in3Active && !in4Active));
  bool outputState34 = outputState12;
  digitalOut(outputState12, !outputState12, outputState34, !outputState34);
}

void function6() { // 2*2 input XOR/XNOR gate
  displayValues(map(potValue1, MINREG, MAXREG, 1, TRESHOLD_MAX), "P1:", "", map(potValue2, MINREG, MAXREG, 1, TRESHOLD_MAX), "P2:", "");
  readShowInputs();
  
  if (trigActive) // Manual test: set all inputs to active...
    in1Active = in2Active = in3Active = in4Active = true;

  bool outputState12 = ((in1Active && in2Active) || (!in1Active && !in2Active));
  bool outputState34 = ((in3Active && in4Active) || (!in3Active && !in4Active));
  digitalOut(outputState12, !outputState12, outputState34, !outputState34);
}

void function7() { // SR-flipflop - SET=IN1, CLK=IN2, RESET=IN3
  printInputTags("SET", "CLK", "RESET", "");
  displayValues(map(potValue1, MINREG, MAXREG, 1, TRESHOLD_MAX), "P1:", "", map(potValue2, MINREG, MAXREG, 1, TRESHOLD_MAX), "P2:", "");
  readShowInputs();
  
  if (!trigTriggerUsed && trigActive) { // Manual trigger: set IN2 (CLK) inputs to active...
    in2Active = true;
    in2TriggerUsed = false;
    trigTriggerUsed = true;
  }
    
  if (in1Active) { // IN1=SET
    Q_FF = true;
    if (!in1TriggerUsed) {
      in1TriggerUsed = true;
    }
  } else {
    in1TriggerUsed = false;
  }

  if (in2Active) { // IN2=CLK
    if (!in2TriggerUsed) {
      Q_FF = !Q_FF;
      in2TriggerUsed = true;
    }
  } else {
    in2TriggerUsed = false;
  }
  
  if (in3Active) { // IN3=RESET
    Q_FF = false;
    if (!in3TriggerUsed) {
      in3TriggerUsed = true;
    }
  } else {
    in3TriggerUsed = false;
  }

  // Set Q (=O1-O3) and NOT-Q (=Q2-Q4) accordingly...
  if (Q_FF)
    digitalOut(true, false, true, false);
  else
    digitalOut(false, true, false, true);
}

void function8() { // Monoflop configurable pulse length
  // potValue2 is pulse length for the monoflop, potValue1 is used as sensor treshold for all inputs...
  sensorTresh2 = sensorTresh1; 
  timeDelay = round10(map(potValue2, MINREG, MAXREG, 0, DELAY_MAX));
  printInputTags("DSBL", "TRIG", "", "");
  displayValues(potValue1, "P1:", "", timeDelay, "T:", " ms"); // Something else!
  readShowInputs();
  
  if (!trigTriggerUsed && trigActive) { // Manual trigger: set IN2 (TRIG) inputs to active...
    in2Active = true;
    in2TriggerUsed = false;
    trigTriggerUsed = true;
  }
    
  unsigned long pulseStart;
  char displayVal[15];

  if (!in1Active) {
    if (in2Active && !in2TriggerUsed) {
      in2TriggerUsed = true; // IN2 is edge sensitive CLK...
      pulseStart = millis(); // Restart pulse timer...
      runningTimer = true;
      digitalOut(true, false, true, false);
      display.fillRect(0, 24, 128, 15, BLACK); // Clean function row...
    }
  }

  if (runningTimer) {
    if ((millis() - pulseStart) > timeDelay) {
      // Flush pulse...
      runningTimer = false;
      display_function(F("MONOFLOP"));
    } else {
      // Display time counting down...
      display.setTextSize(2);
      display.setTextColor(WHITE,BLACK);
      sprintf(displayVal, "  %d  ", timeDelay-(millis()-pulseStart));
      display.setCursor(64-(strlen(displayVal)*12)/2,24);
      display.println(displayVal);
    }
  } else {
    digitalOut(false, true, false, true);
  }
}

void function9() { // Counter  
  // IN1 & IN2 sum count on counter 1, IN3 & IN4 sum count on counter 2
  int counter1 = 0;
  int counter2 = 0;
  char displayCount[10];

  printInputTags("C1", "C1", "C2", "C2");
  while (1) { // For ever...
    displayValues(map(potValue1, MINREG, MAXREG, 1, TRESHOLD_MAX), "P1:", "", map(potValue2, MINREG, MAXREG, 1, TRESHOLD_MAX), "P2:", "");
    initTresholds();
    readShowInputs();
    
    // Count on counter 1...
    if (in1Active && !in1TriggerUsed) {
      in1TriggerUsed = true; // Edge sensitive toggle...
      counter1++;
    }    
    if (in2Active && !in2TriggerUsed) {
      in2TriggerUsed = true; // Edge sensitive toggle...
      counter1++;
    }
    
    // Count on counter 2...    
    if (in3Active && !in3TriggerUsed) {
      in3TriggerUsed = true; // Edge sensitive toggle...
      counter2++;
    }
    if (in4Active && !in4TriggerUsed) {
      in4TriggerUsed = true; // Edge sensitive toggle...
      counter2++;
    }

    if (counter1>9999) counter1 = 0;
    if (counter2>9999) counter2 = 0;
    
    // Display counters...
    display.setTextSize(2);
    display.fillRect(0, 24, 128, 15, BLACK); // Clean counters row...
    sprintf(displayCount, "%d", counter1);
    display.setCursor(32-(strlen(displayCount)*12)/2,24);
    display.println(displayCount);
    sprintf(displayCount, "%d", counter2);
    display.setCursor(96-(strlen(displayCount)*12)/2,24);
    display.println(displayCount);

    // Toggle respective outputs when counting...
    digitalOut((counter1%2)?true:false, (counter1%2)?false:true, (counter2%2)?true:false, (counter2%2)?false:true);

    if (!trigTriggerUsed && trigActive) // Set counters to 0...
      counter1 = counter2 = 0;
  }
}

void function10() { // Oscillator
  // Block wave oscillator / pulse generator. Pot1=period, pot2=duty...
  bool waveEdge = true; // TRUE is pulse time logic '1', FALSE is pulse time of '0'
  bool generatorRunning = false;
  bool byEdge = false;
  unsigned long pulseStart;
  unsigned long periodTime;
  
  sensorTresh2 = sensorTresh1;
  printInputTags("PWR", "EDGE", "", "");

  // Init output...
  digitalOut(true, false, false, true);
      
  while (1) { // For ever...
    timeDelay  = round10(map(potValue1, MINREG, MAXREG, 0, DELAY_MAX));
    periodTime = round10(map(potValue2, MINREG, MAXREG, 0, DELAY_MAX));
    //dutyCycle  = round10(map(potValue2, MINREG, MAXREG, 1, 50));
    //displayValues(timeDelay, "", " ms", dutyCycle, "Duty ", "%");
    displayValues(timeDelay, "", " ms", periodTime, " ", " ms");
    initTresholds();
    readShowInputs();
    
    if (!trigTriggerUsed && trigActive) { // Manual: set IN2 (TRIG) inputs to active...
      in2Active = true;
      in2TriggerUsed = false;
      trigTriggerUsed = true;
    }

    if (in1Active) {
      if (!generatorRunning) {
        // Start block wave...
        pulseStart = millis(); // Restart pulse timer...
        waveEdge = true;
        digitalOut(true, false, false, true);
      }
      generatorRunning = true;
      byEdge = false;
     
    } else {
      if (in2Active && !in2TriggerUsed) {
        in2TriggerUsed = true; // M2 is edge sensitive toggle...
        generatorRunning = !generatorRunning;
        byEdge = true;
        if (generatorRunning) {
          // Start block wave...
          pulseStart = millis(); // Restart pulse timer...
          waveEdge = true;
          digitalOut(true, false, false, true);       
        }
      } else {
        if (!byEdge) generatorRunning = false;
      }
    }

    if (generatorRunning) {
      if ( ( waveEdge && (millis() - pulseStart) > timeDelay  ) ||
           (!waveEdge && (millis() - pulseStart) > periodTime ) ) {
        // Invert pulse...
        if (waveEdge) {
          digitalOut(false, true, true, false);
        } else {
          digitalOut(true, false, false, true);    
        }
        waveEdge = !waveEdge;
        pulseStart = millis(); // Restart pulse timer...
        //logSerial(F("pulseStart 3"));
      }
    } else {
      // Idle/stopped...
      digitalOut(false, true, true, false);
    }
  }
}

void function11() { 
  // Shift/Sequencer. Pot1=Sensor threshold (for all inputs), pot2=speed while active
  bool isRunning = false;
  bool byEdge = false;
  unsigned long pulseStart;
  uint8_t nextOutput = 0;
  
  sensorTresh2 = sensorTresh1; 
  printInputTags("PWR", "EDGE", "", "");

  // Init to output 1 active...
  digitalOut(true, false, false, false);
  nextOutput = 2;

  while (1) { // For ever...
    timeDelay = round10(map(potValue2, MINREG, MAXREG, 0, DELAY_MAX));
    displayValues(potValue1, "P1:", "", timeDelay, "T:", " ms"); // Something else!
    initTresholds();
    readShowInputs();
    
    if (!trigTriggerUsed && trigActive) { // Manual: set IN2 (TRIG) inputs to active...
      in2Active = true;
      in2TriggerUsed = false;
      trigTriggerUsed = true;
    }

    if (in1Active) {
      if (!isRunning) {
        // Start...
        pulseStart = millis(); // Restart pulse timer...
      }
      isRunning = true;
      byEdge = false;
    } else {
      if (in2Active && !in2TriggerUsed) {
        in2TriggerUsed = true; // M2 is edge sensitive toggle...
        isRunning = !isRunning;
        byEdge = true;
        if (isRunning) {
          // Restart...
          pulseStart = millis(); // Restart pulse timer...
        }
      } else {
        if (!byEdge) isRunning = false;
      }
    }

    if (isRunning) {
      if ((millis() - pulseStart) > timeDelay)   { 
        // Set corresponding output...
        switch(nextOutput) {
          case 1:
            digitalOut(true, false, false, false);
            break;
          case 2:
            digitalOut(false, true, false, false);
            break;
          case 3:
            digitalOut(false, false, true, false);
            break;                       
          case 4:
            digitalOut(false, false, false, true);
        }
        pulseStart = millis(); // Restart pulse timer...
        nextOutput++;
        if (nextOutput>4) nextOutput = 1;
      }
    }
  }
}

#ifdef FUNC12_NUNCHUK
void function12() { // Test Nunchuk as device on the i2C bus...
  int motorSpeed1, motorSpeed2;
  motorSpeed1 = motorSpeed2 = 0;
  sensorTresh1 = sensorTresh2 = TRESHOLD_MAX/2; // Use defaults for all inputs...

  while (1) { // For ever...
    initTresholds();
    // Allow manual triggering, but don't use readShowInputs(), 2 inputs are read via I2C...
    trigButton = readdipSwitches();
    if (dipSwitches!=15 && trigButton==15) {
      trigActive = true;
    } else {
      trigActive = false;
      trigTriggerUsed = false;
    }

    nunchuk.update();
    // Possible values:
    //   nunchuk.analogX,nunchuk.analogY
    //   nunchuk.accelX, nunchuk.accelY, nunchuk.accelZ
    //   nunchuk.zButton, nunchuk.cButton
    displayValues(motorSpeed1, "M1:", "", motorSpeed2, "M2:", "");
  
    // Output 1
    if (nunchuk.analogX<127 || nunchuk.analogX>128) {
      motorSpeed1 = map(nunchuk.analogX, 0, 255, -255, 255);
      Out1.set(motorSpeed1);
    } else {
      motorSpeed1 = 0;
      Out1.brake();   
    }

    // Output 2
    if (nunchuk.analogY<127 || nunchuk.analogY>128) {
      motorSpeed2 = map(nunchuk.analogY, 0, 255, -255, 255);
      Out2.set(motorSpeed2);
    } else {
      motorSpeed2 = 0;
      Out2.brake();
    }

    // Optionally do something useful with the trigger (IN1), cButton and zButton :-)
    
    // Show driver outputs...
    displayValues(motorSpeed1, "M1:", "", motorSpeed2, "M2:", "");
    speedBar(0, (nunchuk.analogX>128), motorSpeed1);      
    speedBar(1, (nunchuk.analogY>128), motorSpeed2);

    // Reflect state of buttons...
    showInput(0, (bool)trigActive);
    showInput(1, (bool)nunchuk.cButton);
    showInput(2, (bool)nunchuk.zButton);
  }
}
#else
void function12() { // 3 bit binary decoder - IN1=CLK, IN2=bit2, IN3=bit1, IN4=bit0
  // bit2 is used as !enable: when it is active the output is disabled, so 2 decoders can be contatenated to decode to 8 individual outputs
  unsigned long pulseStart;

  sensorTresh2 = sensorTresh1;
  timeDelay = round10(map(potValue2, MINREG, MAXREG, 0, 1000)); // Max settle time inputs is 1 sec...
  displayValues(round10(map(potValue1, MINREG, MAXREG, 1, TRESHOLD_MAX)), "P:", "", timeDelay, "", " ms");
  printInputTags("CLK", "B2", "B1", "B0");
  readShowInputs();

  if (!in2Active) { // IN2 acts as !enable (carry bit)
    if (in1Active && !in1TriggerUsed) {
      in1TriggerUsed = true; // IN1 is edge sensitive CLK...
      pulseStart = millis(); // Restart pulse timer...
      runningTimer = true;
    }
    if (runningTimer) {
      if ((millis() - pulseStart) > timeDelay) { // Inputs are stable...
        runningTimer = false;
        outValue = (uint8_t)(in3Active*2) + (uint8_t)in4Active;
      }
    }
    digitalOut((outValue==0), (outValue==1), (outValue==2), (outValue==3));
  } else {
    digitalOut(false, false, false, false);
  }
}
#endif

#ifdef FUNC13_COLORSENSOR
void function13() { // TCS34725 color sensor, Red=Q1, Green=Q2, Blue=Q3, Yellow=Q4...
  // See https://arduino.stackexchange.com/questions/19688/use-adafruit-colorsensor-to-distinguish-between-green-and-red
  uint16_t red, green, blue, clr;
    
  displayValues(map(potValue1, MINREG, MAXREG, 1, TRESHOLD_MAX), "P1:", "", map(potValue2, MINREG, MAXREG, 1, TRESHOLD_MAX), "P2:", "");
  readShowInputs();
  
  tcs.setInterrupt(false);  // turn on LED
  delay(60);  // takes 50ms to read 
  tcs.getRawData(&red, &green, &blue, &clr);
  tcs.setInterrupt(true);  // turn off LED
    
  if (compareColor(&colorRed, red, green, blue, clr)) {
    Serial.println("Red...");
  }
  if (compareColor(&colorGreen, red, green, blue, clr)) {
    Serial.println("Green...");
  }
  if (compareColor(&colorBlue, red, green, blue, clr)) {
    Serial.println("Blue...");
  }
  if (compareColor(&colorYellow, red, green, blue, clr)) {
    Serial.println("Yellow...");
  }
  
  Out1.clear();
  Out2.clear();
  digitalOut(
    compareColor(&colorRed, red, green, blue, clr), 
    compareColor(&colorGreen, red, green, blue, clr), 
    compareColor(&colorBlue, red, green, blue, clr), 
    compareColor(&colorYellow, red, green, blue, clr));

}
#else
void function13() { // Future ideas...
  displayValues(map(potValue1, MINREG, MAXREG, 1, TRESHOLD_MAX), "P1:", "", map(potValue2, MINREG, MAXREG, 1, TRESHOLD_MAX), "P2:", "");
  readShowInputs();
  
  if (trigActive) // Manual test: set all inputs to active...
    in1Active = in2Active = in3Active = in4Active = true;
}
#endif

void function14() { // Two speed-regulated motors
  int motorSpeed1, motorSpeed2;
  bool motorsRunning = false;
  bool motorDir1 = true; // CW is default...
  bool motorDir2 = true; // CW is default...
  bool byEdge = false;

  printInputTags("PWR", "EDGE", "DIR1", "DIR2");
  while (1) { // For ever...
    // IN1 is 'Active'=run, IN2 flipflop-toggles on/off, 
    // IN3 is rotation direction for Out1, IN4 is rotation direction for Out2
    initTresholds();
    sensorTresh1 = sensorTresh2 = TRESHOLD_MAX/2; // Use defaults for all inputs...
    readShowInputs();
    displayValues(motorSpeed1, "M1:", "", motorSpeed2, "M2:", "");

    if (!trigTriggerUsed && trigActive) { // Manual: set IN2 (TRIG) inputs to active...
      in2Active = true;
      in2TriggerUsed = false;
      trigTriggerUsed = true;
    }
    
    // Do some logic...
    if (in1Active) {
      motorsRunning = true;
      byEdge = false;
    } else {
      if (in2Active && !in2TriggerUsed) {
        in2TriggerUsed = true; // M2 is edge sensitive toggle...
        motorsRunning = !motorsRunning;
        byEdge = true;
      } else {
        if (!byEdge) motorsRunning = false;
      }
    }
    if (in3Active)
      motorDir1 = true;
    else
      motorDir1 = false;
    if (in4Active)
      motorDir2 = true;
    else
      motorDir2 = false;      

    if (motorsRunning) {
      motorSpeed1 = map(potValue1, MINREG, MAXREG, 0, MAX_VALUE);
      motorSpeed2 = map(potValue2, MINREG, MAXREG, 0, MAX_VALUE);
      Out1.set((motorDir1)?motorSpeed1:-motorSpeed1);
      Out2.set((motorDir2)?-motorSpeed2:motorSpeed2);
    } else {
      motorSpeed1 = motorSpeed2 = 0;
      Out1.brake();
      Out2.brake();      
    }
    speedBar(0, motorDir1, motorSpeed1);
    speedBar(1, !motorDir2, motorSpeed2);
  }
}

void function15() { // Output/Out demo (optionally connected motors)...
  int motorSpeed;
  
  // Output/Out 1 demo...
  for (motorSpeed=0; motorSpeed<=MAX_VALUE; motorSpeed+=5) {
    Out1.set(motorSpeed);
    speedBar(0, true, motorSpeed);
    displayValues(motorSpeed, "M1:", "", 0, "M2:", "");
  }
  for (motorSpeed=MAX_VALUE; motorSpeed>=0; motorSpeed-=5) {
    Out1.set(motorSpeed);
    speedBar(0, true, motorSpeed);
    displayValues(motorSpeed, "M1:", "", 0, "M2:", "");    
  }
  for (motorSpeed=0; motorSpeed>-MAX_VALUE; motorSpeed-=5) {
    Out1.set(motorSpeed);
    speedBar(0, false, motorSpeed);
    displayValues(motorSpeed, "M1:", "", 0, "M2:", "");
  }
  for (motorSpeed=-MAX_VALUE; motorSpeed<=0; motorSpeed+=5) {
    Out1.set(motorSpeed);
    speedBar(0, false, motorSpeed);
    displayValues(motorSpeed, "M1:", "", 0, "M2:", "");
  }
  Out1.set(0);
  speedBar(0, false, 0);
    
  // Output/Out 2 demo...
  for (motorSpeed=0; motorSpeed<=MAX_VALUE; motorSpeed+=5) {
    Out2.set(motorSpeed);
    speedBar(1, true, motorSpeed);
    displayValues(0, "M1:", "", motorSpeed, "M2:", "");
  }
  for (motorSpeed=MAX_VALUE; motorSpeed>=0; motorSpeed-=5) {
    Out2.set(motorSpeed);
    speedBar(1, true, motorSpeed);
    displayValues(0, "M1:", "", motorSpeed, "M2:", "");
  }
  for (motorSpeed=0; motorSpeed>-MAX_VALUE; motorSpeed-=5) {
    Out2.set(motorSpeed);
    speedBar(1, false, motorSpeed);
    displayValues(0, "M1:", "", motorSpeed, "M2:", "");
  }
  for (motorSpeed=-MAX_VALUE; motorSpeed<=0; motorSpeed+=5) {
    Out2.set(motorSpeed);
    speedBar(1, false, motorSpeed);
    displayValues(0, "M1:", "", motorSpeed, "M2:", "");
  }
  Out2.set(0);
  speedBar(1, false, 0);

  delay(2000); // Pause 2 sec. then start routine over...
}


// ----------------------------------------------------------------------------------------------------------------

unsigned long round10(unsigned long n) {
  return (n/10 + (n%10>4)) * 10;
}

void digitalOut(bool DigitalOut1, bool DigitalOut2, bool DigitalOut3, bool DigitalOut4) {
  // Because the digital outputs are inverted/buffered AIN1(=Out1), AIN2(=Out2), BIN2(=Out3) and BIN1(=Out4) lines, this also sets 
  // Motor/analog output OUT1 reflects the state of DigitalOut1, motor/analog output OUT2 reflects the state of DigitalOut4

  if (!DigitalOut1 && !DigitalOut2) { // Both LOW...
    if (negativeLogic)
      Out1.clear();
    else
      Out1.brake();
  } else {
    if (DigitalOut1 && DigitalOut2) { // Both HIGH...
      if (negativeLogic)
        Out1.brake();
      else
        Out1.clear();
    } else { // Signals differ...
      if (!DigitalOut1 && !negativeLogic ||
           DigitalOut1 && negativeLogic) {
        Out1.set(MAX_VALUE);
      } else {
        Out1.set(-MAX_VALUE);
      }
    }
  }

  if (!DigitalOut3 && !DigitalOut4) { // Both LOW...
    if (negativeLogic)
      Out2.clear();
    else
      Out2.brake();
  } else {
    if (DigitalOut3 && DigitalOut4) { // Both HIGH...
      if (negativeLogic)
        Out2.brake();
      else
        Out2.clear();
    } else { // Signals differ...
      if (!DigitalOut3 && !negativeLogic ||
           DigitalOut3 && negativeLogic) {
        Out2.set(MAX_VALUE);
      } else {
        Out2.set(-MAX_VALUE);
      }
    }
  }

  showOutput(0, DigitalOut1);
  showOutput(1, DigitalOut2);
  showOutput(2, DigitalOut3);
  showOutput(3, DigitalOut4);
}

void drawInputOutputBoxes(bool fourOutputs) {
  // fourOutputs is default, call with 'false' to show only two (motorspeed-)bars...
  for (int x=1; x<128; x+=32) // Inputs, top of screen...
      display.drawRect(x, 0, 30, 10, WHITE); 
  if (fourOutputs) {
    for (int x=1; x<128; x+=32)
      display.drawRect(x, 54, 30, 10, WHITE);    
  } else {
    display.drawRect(1, 54, 62, 10, WHITE);
    display.drawRect(65, 54, 62, 10, WHITE);    
  }
}

void printInputTags(char* Tag1, char* Tag2, char* Tag3, char* Tag4) {
  // Print input tags...
  display.setTextSize(1);
  display.setTextColor(WHITE,BLACK);
  display.fillRect(0, 13, 128, 8, BLACK);
  display.setCursor(16-(strlen(Tag1)*6)/2,12);
  display.println(Tag1);
  display.setCursor(48-(strlen(Tag2)*6)/2,12);
  display.println(Tag2);
  display.setCursor(80-(strlen(Tag3)*6)/2,12);
  display.println(Tag3);   
  display.setCursor(112-(strlen(Tag4)*6)/2,12);
  display.println(Tag4);    
}

void showInput(uint8_t inputNr, bool showState) {
  // Draws solid boxes at top of screen. Input: outputNr is 0 based, so either 0, 1, 2 or 3  
  display.fillRect(2+(inputNr*32), 1, 28, 8, (showState)?WHITE:BLACK);
}

void showOutput(uint8_t outputNr, bool showState) {
  // Draws solid boxes at bottom of screen. Input: outputNr is 0 based, so either 0, 1, 2 or 3
  display.fillRect(2+(outputNr*32), 55, 28, 8, (showState)?WHITE:BLACK);
  display.display();
}

void speedBar(uint8_t outputNr, bool direction, int showValue) {
  // Draws a horizontal bar on one of the two positions at the bottom of the screen
  // Input: outputNr is 0 or 1 - showValue = 0 ~ 255

  int barLen = map(abs(showValue), 0, MAX_VALUE, 0, 60);
  if (abs(barLen<2)) {
    display.fillRect((2+(64*outputNr)), 55, 60, 8, BLACK);
  } else {
    if (direction) {
      display.fillRect((2+(64*outputNr)), 55, barLen, 8, WHITE);
      display.fillRect((2+(64*outputNr)+barLen), 55, 60-barLen, 8, BLACK);
    } else {
      display.fillRect((2+(64*outputNr)), 55, 60-barLen, 8, BLACK);      
      display.fillRect((2+(64*outputNr)+(60-barLen)), 55, barLen, 8, WHITE);
    }
  }
  display.display();
}

uint8_t readdipSwitches() {
  uint8_t DIPval;

  DIPval = digitalRead(DIP1);
  DIPval = DIPval << 1;
  DIPval += digitalRead(DIP2);
  DIPval = DIPval << 1;
  DIPval += digitalRead(DIP3);
  DIPval = DIPval << 1;
  DIPval += digitalRead(DIP4);
  return(15-DIPval); // Dec 2021: Switches are negative logic...
}

void display_function(String Line) {
  // Displays current function in a big font
  display.setTextSize(2);
  display.setTextColor(WHITE,BLACK);
  display.setCursor(64-((Line.length())*12)/2,24);
  display.println(Line);
}

void showInput(bool Active, uint8_t xPos) {
  // Draws an 'active'-box at the top of the screen
  if (Active)
    display.fillRect(xPos, 1, 28, 8, WHITE);
  else
    display.fillRect(xPos, 1, 28, 8, BLACK);
}

void displayValues(int dispValue1, char* preTxt1, char* postTxt1, int dispValue2, char* preTxt2, char* postTxt2) {
  // Displays two values, both centered in the left or right part of the screen. 
  // Provides optional pre- and post-fix of tags or units, etc.
  char displayVal[15];
  
  display.fillRect(0, 43, 128, 10, BLACK); // Clean row...
  display.setTextSize(1);
  display.setTextColor(WHITE,BLACK);

  sprintf(displayVal, "%s%d%s", preTxt1, dispValue1, postTxt1);
  display.setCursor(0+(32-(strlen(displayVal)*3)),43);
  display.print(displayVal);

  sprintf(displayVal, "%s%d%s", preTxt2, dispValue2, postTxt2);
  display.setCursor(64+(32-(strlen(displayVal)*3)),43);
  display.print(displayVal);
}

void logSerial(String message) {
  // Handled overhere so logging to serial can easily be switched off...
  if (logging) Serial.println(message);
  return;
}

void setMotorSpeedFunction0(bool rotateDir, int motorSpeed, bool smooth) { // ONLY used for function0...
  // rotateDir = TRUE means O1/O2 CW rotation. Output O3/O4 is reversed CCW
  // rotateDir = FALSE means O1/O2 CCW. Output O3/O4 is reversed CW
  // motorSpeed is a value from 0 to 255
  // smooth = potmeter setting is value of gracefully descend and ascend
  int tempSpeed;
  int speedStep;

  if (rotateDir!=currentOutputDir || motorSpeed!=currentOutputSpeed) {  
    if (!smooth) { // Standard, simple reverse direction
      setSpeedFunction0(rotateDir, motorSpeed);
    } else { // Speed up and down with smooth limits
      if (currentOutputDir == rotateDir) { // No directional changes...
        speedStep = (motorSpeed-currentOutputSpeed)/SPEED_STEPS;
        tempSpeed = currentOutputSpeed;
        do {
          tempSpeed += speedStep;
          if (abs(motorSpeed-tempSpeed) <= abs(speedStep)) tempSpeed = motorSpeed; // Last step...       
          //logSerial("No dir change - tempSpeed:"+(String)tempSpeed+" - currentOutputSpeed:"+(String)currentOutputSpeed+" - motorSpeed:"+(String)motorSpeed+" - speedStep:"+(String)speedStep);
          readDelayFunction0();
          delay(timeDelay/25);
          setSpeedFunction0(rotateDir, tempSpeed);
        } while (abs(motorSpeed-tempSpeed) > abs(speedStep));
      } else { // Towards and through zero...
        for (tempSpeed=currentOutputSpeed; tempSpeed>0; tempSpeed -= (currentOutputSpeed/SPEED_STEPS)) { // Slow down to zero...
          //logSerial("To zero - tempSpeed:"+(String)tempSpeed+" - currentOutputSpeed:"+(String)currentOutputSpeed+" - motorSpeed:"+(String)motorSpeed+" - speedStep:"+(String)speedStep);
          readDelayFunction0();
          delay(timeDelay/25);
          setSpeedFunction0(currentOutputDir, tempSpeed);
        }
        speedStep = motorSpeed/SPEED_STEPS; // Always positive...
        for (tempSpeed=0; tempSpeed<=motorSpeed; tempSpeed += (motorSpeed/SPEED_STEPS)) { // Build up to motorSpeed again...
          if (abs(motorSpeed-tempSpeed) <= speedStep) tempSpeed = motorSpeed; // Last step...
          //logSerial("Build up - tempSpeed:"+(String)tempSpeed+" - currentOutputSpeed:"+(String)currentOutputSpeed+" - motorSpeed:"+(String)motorSpeed+" - speedStep:"+(String)speedStep);
          readDelayFunction0();
          delay(timeDelay/25);
          setSpeedFunction0(rotateDir, tempSpeed);
          readShowInputs();
        }       
      }
    }
    currentOutputDir = rotateDir;
    currentOutputSpeed = motorSpeed;
  }
}

void readDelayFunction0() {
  initTresholds();
  sensorTresh2 = sensorTresh1; // Use tresh1 for all inputs...
  readShowInputs();
  timeDelay = round10(map(potValue2, MINREG, MAXREG, 0, DELAY_MAX));
  displayValues(round10(map(potValue1, MINREG, MAXREG, 1, TRESHOLD_MAX)), "P:", "", timeDelay, "", " ms");
}

void setSpeedFunction0(bool rotateDir, int motorSpeed) {  
  Out1.set((rotateDir)?motorSpeed:-motorSpeed);
  Out2.set((rotateDir)?-motorSpeed:motorSpeed);
  speedBar(0, rotateDir, abs(motorSpeed));
  speedBar(1, !rotateDir, abs(motorSpeed));   
}

#ifdef FUNC13_COLORSENSOR
boolean compareColor(Color *std, int r, int g, int b, int c) {

  //Serial.print(F("RED "));
  //Serial.print(std->red);
  //Serial.print(F("-"));
  //Serial.print(r);
  //Serial.print(F("/"));
  //Serial.print(abs(r - std->red));
  //Serial.print(F(" GREEN "));
  //Serial.print(std->green);
  //Serial.print(F("-"));
  //Serial.print(g);  
  //Serial.print(F("/"));
  //Serial.print(abs(g - std->green));
  //Serial.print(F(" BLUE "));
  //Serial.print(std->blue);
  //Serial.print(F("-"));
  //Serial.print(b);  
  //Serial.print(F("/"));
  //Serial.print(abs(b - std->blue));
  //Serial.print(F(" CLR "));
  //Serial.print(std->clr);
  //Serial.print(F("-"));
  //Serial.print(c);  
  //Serial.print(F("/"));
  //Serial.println(abs(c - std->clr));
 
  if ( (abs(r - std->red) <= MAX_COLOR_OFFSET) &&
       (abs(g - std->green) <= MAX_COLOR_OFFSET) &&
       (abs(b - std->blue) <= MAX_COLOR_OFFSET) &&
       (abs(c - std->clr) <= MAX_COLOR_OFFSET) ) {
    return true;
  }
  return false;
}
#endif
