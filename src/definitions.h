// PINS------------------------------------
//   ~   ~ ~         ~ ~  ~     ~     ~
// 2 3 4 5 6 7  8    9 10 11    12    13
// I + - - - ce csn  - -  mosi  miso  +

//  I for Interrupt

/*

D0
D1      Interrupt
D2
D3  ~   SCL I2C
D4      Left Roll
D5  ~   Pitch
D6  ~   Right Roll
D7      CE
D8      CSN
D9  ~   Motor
D10 ~   Yaw
D11 ~   MOSI ? FREE
D12 ~   MISO ? FREE
D13 ~   === FREE ===

*/

// ANALOG
// short vibroPin = A0;
// #define photoresistorPin A1 // no need because 33 sense has camera

// DIGITAL

// PWM

// look from Top

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
