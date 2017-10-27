//Autopilot
//MPU-9150
//15-11-2016
//
//Main file

#include <Wire.h>
#include <Servo.h>
#include "TinyGPS.h"
#include "config.h"

//****************************************CONSTANTS****************************************
//IMU constants
const int MPU = 0x68;  //I2C address of the MPU9150
const float gyroScaleFactor = 16.4; //sensitivity scale factor from datasheet 0-131 1-65.6 2-32.8 3-16.4
const int acclScaleFactor = 2048; //sensitivity scale factor from datasheet 0-16384 1-8192 2-4096 3-2048

//GPS constants
const float metreToFeet = 3.28084; //feet in one metre

//air data constants
const float soundSeaLevel = 661.4788; //speed of sound at sea level - knots (Kts)

const float tempSeaLevel = 288.15; //air temperature at sea level - kelvin (K)
const float pressureSeaLevel = 29.92126; //air pressure at sea level - inches mercury (inHg)

//****************************************PINS****************************************

//switch pins
const char rcSwitchPin = 0;
const char modePin = 0;

//servo pins
const char servoSwitchPin = 0;
const char elevatorPin = 2;
const char leftAlrnPin = 0;
const char rightAlrnPin = 0;
const char rudderPin = 3;

//indicator pins
const char LEDstatusPin = 0;
const char LEDfaultPin = 0;

//temperature sensor pins
const char ESCtempPin = 0;
const char OATPin = 0;

//pressure sensor pins
const char totalPressurePin = 0;
const char staticPressurePin = 0;

//ultrasonic pins
const char ultraSonicPin = 0;

//current sensor pins
const char currentSensePin = 0;

//RSSI / lost packet pins
const char RSSIPin = 0;


//****************************************VARIABLES****************************************
//timing variables
unsigned long loopTime; //TESTING
unsigned long loopStartTime; //TESTING

//mode variables
int mode;

//IMU calc variables
unsigned long pastMicros = 0, currentMicros = 0; //time since program started in us
float dt; //change in time for calculations

int16_t acclX, acclY, acclZ, IMUtemperature, gyroX, gyroY, gyroZ; //16 byte variables for accelerometer,gyro and temperature readings

double acclAngleX = 0, acclAngleY = 0, gyroRateX, gyroRateY, gyroAngleX = 0, gyroAngleY = 0; //variables for angles and rates
float currentPitchAngle = 0, currentRollAngle = 0; //variables for the final angles

//Control PID variables
unsigned long lastPID; //last time PID computed

float pitchAngle, elevatorOutput, pitchSetpoint; //pitch input to PID, output of PID, setpoint for PID
float pitchErrorSum, pitchErrorLast; //pitch sum for integral, last error for dirivative

float rollAngle, alrnOutput, rollSetpoint; //roll 
float rollErrorSum, rollErrorLast; //roll

float heading, rudderOutput, headingSetpoint; //heading
float headingErrorSum, headingErrorLast; //heading

//GPS variables
TinyGPS GPS; //tny GPS object

float latitude, longitude; //decimals
float GPSAltitude; //GPS alt in feet
float groundSpeed; //knots

float currentCourse = 0; //GPS heading
float homeCourse = 0; //heading to face home

unsigned long timeLastGPS = 0; //time since last fix given
unsigned long GPSFixAge = 0; //age of fix

//more variables


//servo variables
Servo elevatorServo, leftAlrnServo, rightAlrnServo, rudderServo;

//****************************************SETUP****************************************
//setup function
void setup()
{
  //start serial for debug
  Serial.begin(250000);

  //start serial for GPS
  Serial1.begin(9600);

  //start serial for telemetry
  Serial2.begin(9600);
  Serial2.println("Telemetry OK");
  
  //set up ports

  //set up GPS and wait for fix


  //set upI2C for MPU
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); //PWR_MGMT_1 register
  Wire.write(0); //set to zero, turn off sleep mode
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU);
  Wire.write(0x1B); //GYRO_CONFIG register FS_SEL
  Wire.write(B00011000); //change range of gyro dps 0-250 1-500 2-1000 3-2000
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU);
  Wire.write(0x1C); //ACCL_CONFIG register AFS_SEL
  Wire.write(B00011000); //change range of gyro dps 0-2 1-4 2-8 3-16
  Wire.endTransmission(true);


  //calibrate IMU using averaged accl value

  //get current angle
  
  //attach servos
  elevatorServo.attach(elevatorPin); //attach elevator
  leftAlrnServo.attach(leftAlrnPin); //attach left aileron
  rightAlrnServo.attach(rightAlrnPin); //attach right aileron
  rudderServo.attach(rudderPin); //attach rudder

  //test all servos
  Serial.println("Servos minimum");
  elevatorServo.write(90 + maxServoAngle);
  leftAlrnServo.write(90 + maxServoAngle);
  rightAlrnServo.write(90 + maxServoAngle);
  rudderServo.write(90 + maxServoAngle);
  delay(1000);
  Serial.println("Servos maximum");
  elevatorServo.write(90 - maxServoAngle);
  leftAlrnServo.write(90 - maxServoAngle);
  rightAlrnServo.write(90 - maxServoAngle);
  rudderServo.write(90 - maxServoAngle);
  delay(1000);
  Serial.println("Servos centre");
  elevatorServo.write(90);
  leftAlrnServo.write(90);
  rightAlrnServo.write(90);
  rudderServo.write(90);

  //wait for RC mode to be selected

  Serial.println("Setup OK");
}

//****************************************FUNCTIONS****************************************
//read raw IMU values
void rawInert()
{
  //setup I2C
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); //start with register 0x3B (ACCEL_XOUT_H)
  if(Wire.endTransmission(false) == 0) //return 0 = no faults
  {
    Wire.requestFrom(MPU, 14, true); //14 registers - LOCKS HERE IF GPS CONNECTED

    //get accl and gyro values
    acclX = Wire.read() <<8|Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    acclY = Wire.read() <<8|Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    acclZ = Wire.read() <<8|Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    IMUtemperature = Wire.read() <<8|Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gyroX = Wire.read() <<8|Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gyroY = Wire.read() <<8|Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gyroZ = Wire.read() <<8|Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
    IMUtemperature = IMUtemperature/340.00+36.53; //temperature in degC from datasheet
  }
  
  //print to serial - comment out
  //Serial.print("AcclX = "); Serial.print(acclX);
  //Serial.print(" | AcclY = "); Serial.print(acclY);
  //Serial.print(" | AcclZ = "); Serial.print(acclZ);
  //Serial.print(" | Temp = "); Serial.print(IMUtemperature);
  //Serial.print(" | GyroX = "); Serial.print(gyroX);
  //Serial.print(" | GyroY = "); Serial.print(gyroY);
  //Serial.print(" | GyroZ = "); Serial.println(gyroZ);
}


//calculate accelerometer angles in degrees
void acclAngles()
{
  //replace with atan2
  acclAngleX = RAD_TO_DEG * atan((float)acclY / sqrt(pow((float)acclZ, 2) + pow((float)acclX, 2))); //convert to angles in X plane
  acclAngleY = RAD_TO_DEG * atan((float) - acclX / sqrt(pow((float)acclZ, 2) + pow((float)acclY, 2))); //convert to angle in Y plane

  //correct range to 360deg
  if(acclZ < 0)
  {
    if(acclAngleX < 0)
      acclAngleX = -180 - acclAngleX;
    else
      acclAngleX = 180 - acclAngleX;
    
    if(acclAngleY < 0)
      acclAngleY = -180 - acclAngleY;
    else
      acclAngleY = 180 - acclAngleY;
  }

  
  //print to serial - comment out
  //Serial.print("X accl angle "); Serial.println(acclAngleX);
  //Serial.print("Y accl angle "); Serial.println(acclAngleY);
}


//get gyro rates in degrees per second
void gyroRates()
{
  gyroRateX = (float)gyroX / gyroScaleFactor; //calculate gyro rate in X plane
  gyroRateY = (float)gyroY / gyroScaleFactor; //calculate gyro rate in Y plane
  
  //print to serial - comment out
  //Serial.print("X gyro rate "); Serial.println(gyroRateX);
  //Serial.print("Y gyro rate "); Serial.println(gyroRateY);
}


//get current angle using gyro rate
void gyroAngles()
{
  //get gyro rates
  gyroRates();
   
  pastMicros = currentMicros; //time since program started for last loop
  currentMicros = micros(); //find time since program started

  //wrap micros
  if (currentMicros > pastMicros) //micros overflows after ~70 minutes
    dt = (float)(currentMicros - pastMicros)/1000000;
  else
    dt = (float)((4294967295 - pastMicros) + currentMicros)/1000000;

  //integrate gyro rate
  gyroAngleX += (gyroRateX * dt);
  if (gyroAngleX > 360) //wrap around
    gyroAngleX -= 360;
    
  gyroAngleY += (gyroRateY * dt);
  if (gyroAngleY > 360) //wrap around
    gyroAngleY -= 360;

  //print to serial - comment out
  //Serial.print("dt "); Serial.println(dt);
  //Serial.print("gyro X angle "); Serial.println(gyroAngleX);
  //Serial.print("gyro Y angle "); Serial.println(gyroAngleY);
}


void complimentaryFilter()
{
  //calcuate current angle - integrate gyro
  currentPitchAngle += gyroRateX * dt;
  currentRollAngle += gyroRateY * dt;

  //check G value
  int forceMagnitude;
  forceMagnitude = abs(acclX) + abs(acclY) + abs(acclZ);

  //factor in accl when stable (0.5 - 2G)
  if (forceMagnitude >= (float)acclScaleFactor/2 && forceMagnitude <= (float)acclScaleFactor*2)
  {
    currentPitchAngle = (0.98 * currentPitchAngle) + (0.02 * acclAngleX); //X filter: factor in accl
    currentRollAngle = (0.98 * currentRollAngle) + (0.02 * acclAngleY); //Y filter: factor in accl
  } 
}


//calculate the current angles from gyro and accelerometer using complementary filter
void currentAngles()
{
  acclAngles(); //get angles from accelerometer
  gyroAngles(); //get angles from gyroscope

  //filter pitch and roll - store globally
  complimentaryFilter();
  
  
  //store fault if NAN
  
  //print to serial - TESTING
  //Serial.print("CURRENT ANGLE X "); Serial.println(currentPitchAngle);
  //Serial.print("CURRENT ANGLE Y "); Serial.println(currentRollAngle);
  //Serial.println("");

  //Serial.print(gyroAngleX); Serial.print(",");
  //Serial.print(acclAngleX); Serial.print(",");
  Serial.print(currentPitchAngle); Serial.print(",");
}


//get pressures
void rawPressures()
{
  //get static pressure
  //get total pressure
  //get outside air temperature
}


//calculate air data
int calcAirData()
{
  //calculate pressure altitude
  //calculate air speed
  //check stall speed
  //return -1 if error/bad
}


//calculate ultrasonic altitude
int UltrasonicAltitude()
{
  //send signal
  //measure echo
  //calculate distance
  //check validity
  //return ultrasonic altitude
  //if no echo return -1
  
}

/*
//get NMEA GPS strings and parse - tinyGPS
int getGPS()
{
  while (Serial1.available()) //while there is data on port
  {
    int data = Serial1.read(); //get serial data from GPS
    
    if (GPS.encode(data)) //give to tinyGPS - if returns TRUE (complete packet)
    {
      //get GPS info from tinyGPS
      GPS.f_get_position(&latitude, &longitude, &GPSFixAge); //coordinates in decimal
      GPSAltitude = GPS.f_altitude()*3.28084; //+/- altitude in meters to feet
      currentCourse = GPS.f_course(); //course in degrees
      groundSpeed = GPS.f_speed_knots(); //speed in knots

      timeLastGPS = millis(); //record time since last GPS fix

      Serial.print(latitude, 7);
      Serial.print(", ");
      Serial.println(longitude, 7);
      Serial.println(GPSAltitude);
      Serial.println(currentCourse);
      Serial.println("");
    }
  }

  //store faults - timelast, fixAge etc
}
*/


//calc GPS data
void calcGPS()
{
  //calculate distance and heading to home
}

//get battery voltage and current, calc mAh
void batteryStatus()
{
  //read adc for divided voltage
  //read current sensor
  //read ESC temp
  //integrate for mAh
  //check V, I, and T against limits
  //store fault data and return -1 if any limit reached
}


int checkRSSI()
{
  //check RSSI or lost packet mode
  //read RSSI
  //compare against limit
  //store fault data and return -1 if too low
}


//compute PID values
void computePID()
{
  unsigned long currentTime = micros(); //time now using millis
  unsigned long changeTime; //calculated change in time
  
  float pitchError; //pitch error in value
  float pitchDifferential; //pitch differential value
  float rollError; //roll error in value
  float rollDifferential; //roll differnetial value

  float headingError;
  float headingDifferential;

  //loop time
  if (currentTime > lastPID) //calc time since last PID loop
    changeTime = (currentTime - lastPID);
  if (currentTime < lastPID) //if micros overflowed
    changeTime = (currentTime + 4294967295 - lastPID);

  //set max setpoints

  //pitch calc
  pitchError = pitchSetpoint - currentPitchAngle; //pitch calc error in position
  pitchErrorSum += (pitchError * changeTime); //pitch calc integral of error
  pitchDifferential = (pitchError - pitchErrorLast) / changeTime; //pitch calc differential of error

  //roll calc
  rollError = rollSetpoint - currentRollAngle; //roll calc error in position
  rollErrorSum += (rollError * changeTime); //roll calc integral of error
  rollDifferential = (rollError - rollErrorLast) / changeTime; //roll calc differential of error

  //set outputs
  elevatorOutput = (kp * pitchError) + (ki * pitchErrorSum) + (kd * pitchDifferential); //pitch compute PID output
  alrnOutput = (kp * rollError) + (ki * rollErrorSum) + (kd * rollDifferential); //roll compute PID output

  Serial.println(elevatorOutput);
  
  //set last time as loop start time
  lastPID = currentTime;
}



//****************************************MAIN LOOP****************************************
void loop()
{
  //inertial measurements
  rawInert(); //update raw values from MPU  
  currentAngles(); //calc and filter pitch/roll angles - stored globally

  //air data
  rawPressures();
  calcAirData();

  //GPS (every second ish)
  //getGPS();
  //calcGPS();
  
  //current, voltage, esc temp
  batteryStatus();

  //RSSI

  //get mode selected
  //mode = digitalRead(modePin); read signal from RC reciever

  //action on mode
  mode = 2;
  switch (mode)
  {
    case 1: //RC mode
      //digitalWrite(rcSwitchPin) //switch signals to RC
      break;
      
    case 2: //HOLD mode  r
      //pitch = 0, roll = k*heading error, throttle manual
      //calc heading error
      //set setpoints
      computePID();
      elevatorServo.write(90 + elevatorOutput);
      rudderServo.write(90 + rudderOutput);
      //GPS and magnetic heading kept constant
      break;
      
    case 3: //CIRCLE mode
      
      break;
      
    case 4: //RTH mode
      //pitch = 0, roll = k*heading error, throttle = k*(air speed error + const)

      //calc heading error
      //calc speed error
      //set setpoints
      //compute PID
      //set servos
      //GPS and magnetic heading kept constant

      //circle if close to home
      break;
  }
  
 


  //telemetry



  //FOR TESTING
  //Serial.print("Loop Time: ");
  //Serial.println(loopTime);

  //force a 5ms loop (-8us)
  while(micros()-loopStartTime <= 4992)
  {}

  loopTime = micros() - loopStartTime; //calc loop time - TESTING
  loopStartTime = micros(); //find time loop start
}

