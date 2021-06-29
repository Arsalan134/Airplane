// PINS------------------------------------
// ~ ~ ~ ~ ~         ~ ~
// 2 3 4 5 6 7  8    9 10 11    12    13
// - - - - - ce csn  - +  mosi  miso  sck

// ANALOG
// short vibroPin = A0;
// #define photoresistorPin A1 // no need because 33 sense has camera

// DIGITAL
#define lightPin 2

// PWM
#define yawServoPin 3
#define rollServoLeftPin 4  // look from Top
#define rollServoRightPin 6 // look from Top

#define pitchServoPin 5 // combine 2 servos to one signal cable
#define motorPin 9

#define minThrottle 1000
#define maxThrottle 2000

#define degreeOfFreedom 90

#define rollIndex 0
#define pitchIndex 1
#define yawIndex 2
#define throttleIndex 3
#define batteryIndex 0
