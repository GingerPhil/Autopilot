//Autopilot
//MPU-9150
//30-10-2016
//
//Header file

//**********PID********** - for tuning (may need seperate for X and Y)
const float kp = 1; //proportional constant
const float ki = 0; //integral constant
const float kd = 0.1; //derivative constant

//**********servos**********
const int maxServoAngle = 30; //maximum throw of servo - equal to RC

//**********stall speed**********
const int stallSpeed = 100; //minimum airspeed - knots

//**********RSSI**********
//RSSI / lost packet select
const bool RSSIselect = true; //true RSSI, false lost packet count
//invert
const bool RSSIinvert = false; //true invert, false normal

//**********autopilot**********
//ailerons enable - (rudder only unstable)
const bool alrnEnable = true; //true enable, false disable
//bank limit
const float maxBank = 45; //from level - degrees (0-90)

//max distance enable
const bool maxDistEnable = true; //true enable, false disable
//max distance
const float maxDist  = 3; //from GPS home - nauticle miles

//alt ceiling enable
const bool maxAltEnable = false; //true enable, false disable
//alt ceiling
const int maxAlt = 5000; //above sealevel - ft (max 32,000)




