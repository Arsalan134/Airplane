// PINS------------------------------------
//   ~   ~ ~         ~ ~  ~           ~
// 2 3 4 5 6 7  8    9 10 11    12    13
// I - - - - ce csn  - -  mosi  miso  sck

// ANALOG
// short vibroPin = A0;
// #define photoresistorPin A1 // no need because 33 sense has camera

// DIGITAL

// PWM

// look from Top
// #define pitchServoLeftPin 3 SCL reserved

#define pitchServoPin 5

#define rollServoLeftPin 4
#define rollServoRightPin 6

#define yawServoPin 10

#define motorPin 9

#define minThrottle 1000
#define maxThrottle 2000

#define degreeOfFreedom 90

// Indices in recieve payload
#define rollIndex 0
#define pitchIndex 1
#define yawIndex 2
#define throttleIndex 3

// Indices in transmit payload
#define batteryIndex 0
