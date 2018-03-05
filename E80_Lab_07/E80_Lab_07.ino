/********
Default E80 Lab 01 
Current Author: Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)
Previous Contributors:  Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
                        Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)
*/

/* Libraries */

// general
#include <Arduino.h>
#include <Wire.h>
#include <Pinouts.h>

// 80-specific
#include <SensorGPS.h>
#include <SensorIMU.h>
#include <StateEstimator.h>
#include <Adafruit_GPS.h>
#include <ADCSampler.h>
#include <MotorDriver.h>
#include <Logger.h>
#include <Printer.h>
#include <PControl.h>
#include <SoftwareSerial.h>
#define mySerial Serial1

#define ORIGIN_LAT  34.106465 
#define ORIGIN_LON  -117.712488

// template library
#include <LED.h>

/* Global Variables */

// Motors
MotorDriver motorDriver;

// State Estimator
StateEstimator state_estimator;

// Control
PControl pcontrol;

// GPS
SensorGPS gps;
Adafruit_GPS GPS(&mySerial);

// IMU
SensorIMU imu;

// Logger
Logger logger;
bool keepLogging = true;

// Printer
Printer printer;

// Led
LED led;

// loop start recorder
int loopStartTime;
int current_way_point = 0;



void setup() {
  printer.init();

  /* Initialize the Logger */
  logger.include(&imu);
  logger.include(&gps);
  logger.include(&state_estimator);
  logger.include(&motorDriver);
  logger.init();

  /* Initialise the sensors */
  imu.init();

  mySerial.begin(9600);
  gps.init(&GPS);

  /* Initialize motor pins */
  motorDriver.init();

  /* Done initializing, turn on LED */
  led.init();

  /* Keep track of time */
  printer.printMessage("Starting main loop",10);
  loopStartTime = millis();
}





void loop() {

  /* set the motors to run based on time since the loop started */
  /* loopStartTime is in units of ms */
  /* The motorDriver.drive function takes in 4 inputs arguments m1_power, m2_power, m3_power, m4_power: */
  /*       void motorDriver.drive(int m1_power, int m2_power, int m3_power, int m4_power) */
  /* the value of m!_power can range from -255 to 255 */
  /* Note: we typically avoid m3, it hasn't worked well in the past */



  if (printer.loopTime(loopStartTime)) {
    printer.printToSerial();  // To stop printing, just comment this line out
  }

  if ( pcontrol.loopTime(loopStartTime)) {
    PControl();
  }

  if (imu.loopTime(loopStartTime)) {
    imu.read(); // this is a sequence of blocking I2C read calls
    imu.printState(); // a lot of random information
  }
  
  if (true){//(gps.loopTime(loopStartTime)) {
    gps.read(&GPS); // this is a sequence of blocking I2C read calls
    gps.printState(); // a lot of random information
  }

  if (state_estimator.loopTime(loopStartTime)) {
    LongLatToXY();
    state_estimator.printState(); // a lot of random information
  }
  
  // uses the LED library to flash LED -- use this as a template for new libraries!
  if (led.loopTime(loopStartTime)) {
    led.flashLED();
  }

  if (logger.loopTime(loopStartTime) && keepLogging) {
    keepLogging = logger.log();
  }

}

void PControl() {
  // hard coded way points to track
  float x_desired_list[] = {0, 20, 0 };
  float y_desired_list[] = {0, 0 , 0 };
  int num_way_points = 3;
  float success_radius = 4.0;

  float x_des = x_desired_list[current_way_point];
  float y_des = y_desired_list[current_way_point];
  
  float dist = sqrt(pow(state_estimator.state.x-x_des,2) + pow(state_estimator.state.y-y_des,2));
  if (dist < success_radius && current_way_point < num_way_points)
    current_way_point = current_way_point + 1;

  /*Set P control thrust - students must add code here */
  motorDriver.drive(0,0,0,0);
}


void LongLatToXY(){

  // This function should set the values of state_estimator.state.x, state_estimator.state.y, and state_estimator.state.heading
  // It can make use of the constants RADIUS_OF_EARTH, ORIGIN_LAT, ORIGIN_LON
  // The origin values can be hard coded at the top of this file.
  // You can access the current GPS latitude and longitude readings with gps.state.lat and gps.state.lon
  // You can access the current imu heading with imu.state.heading

  
}

float angleDiff(float a){
  while (a>PI)
    a = a - 2*PI;

  while (a<-PI)
    a = a + 2*PI;

  return a;

}

