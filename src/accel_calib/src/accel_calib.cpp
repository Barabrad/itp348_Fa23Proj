/* 
 * SparkFun ADXL345 Accelerometer Breakout Calibration
 * Original Source: lib/ADXL345_Sparkfun_Particle/examples/SparkFun_ADXL345_Calibration/SparkFun_ADXL345_Calibration.ino
 * Original Author: E.Robert @ SparkFun Electronics
 * Editor: Brad Barakat
 * Edited: Nov 12, 2023 for automation
 *         Dec 07, 2023 for Initial State incorporation
 * Edits: This file should now automatically calculate the gains and offsets
 *        if the user follows the given instructions.
 */

// Include Particle Device OS APIs
#include "Particle.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// Include the accelerometer library
#include "SparkFun_ADXL345.h"
// Include ArduinoJson library for the Initial State input
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#include <ArduinoJson.h> 

typedef unsigned long u_long;

/*********** COMMUNICATION SELECTION ***********/
/*    Comment Out The One You Are Not Using    */
// ADXL345 adxl = ADXL345(10);           // USE FOR SPI COMMUNICATION, ADXL345(CS_PIN);
ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION

/****************** CONSTANTS ******************/
/*                                             */
const int G_FRAC = 16; // The fraction of a G without rounding down to 0 (if 10, then resolution is G/10)
const int NUM_AXES = 3; // Number of axes (x,y,z)
const int NUM_DIRS = NUM_AXES*2; // Number of directions (+/- axes)
const String DIR_TEXT[NUM_DIRS] = {"+x", "-x", "+y", "-y", "+z", "-z"};
const size_t MAX_JSON_SIZE = 1048; // Found to be the limit for publishing
const u_long INTERVAL_READ = 2000; // ms

/****************** VARIABLES ******************/
/*                                             */
float gainX, gainY, gainZ;
float offsetX, offsetY, offsetZ;
u_long prevCalibEpoch = 0;
bool calibDirection = false;

/*************** HELPER FUNCTIONS **************/
/*                                             */
/**
 * @brief  This function waits for Initial State input (this is a blocking function!)
 * @retval  void
 */
void waitForCalibSignal() {
  calibDirection = false;
  while (!calibDirection) {
    Particle.publish("input_controls", ":)", PRIVATE); // Trigger the integration
    delay(INTERVAL_READ);
  } // Waiting for character to be sent to Serial
}

/**
 * @brief  This function finds the raw acceleration along an axis
 * @param  axis_accel : The acceleration in the axis (will be overwritten)
 * @param  dir : The direction of the axis (0-5 for ±{x,y,z})
 * @retval  void
 */
void getAxisAccelForCalib(int& axis_accel, const int& dir) {
  Serial.println("Set the " + DIR_TEXT[dir] + " direction to be with gravity.");
  Serial.println("Press the Calib. Accel. button on Initial State to calibrate direction.");
  waitForCalibSignal();
  Serial.println();
  
  // Get the accelerometer readings
  int x,y,z;                          // Initialize variables that will hold results
  adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store in variables x,y,z
  // Print readings for user's reference
  Serial.println("Raw Accel: " + String(x) + ", " + String(y) + ", " + String(z));
  Serial.println();

  // Set axis_accel
  int axis = dir/2; // Range: {0,1,2} due to int division
  switch (axis) {
    case 0: axis_accel = x; break;
    case 1: axis_accel = y; break;
    case 2: axis_accel = z; break;
  }
}

/**
 * @brief  This function handles the integration response when data is queried from Initial State
 * @param  event : The name of the event
 * @param  data : The data
 * @retval  void
 */
void myHandler(const char *event, const char *data) {
  // Handle the integration response
  //Serial.println("Data: " + String(data));
  StaticJsonDocument<MAX_JSON_SIZE> doc;
  DeserializationError error = deserializeJson(doc, data);
  // Test to see if was successful
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    return;
  }
  // {"cal_acc_epoch":{{cal_acc_but.epoch}},"ang_thresh":{{ang_thresh.value}},"lig_thresh":{{lig_thresh.value}}}
  u_long cal_acc_epoch = doc["cal_acc_epoch"];
  //Serial.println("Prev: " + String(prevCalibEpoch) + ", Curr: " + String(cal_acc_epoch));
  if (cal_acc_epoch != prevCalibEpoch) {calibDirection = true; prevCalibEpoch = cal_acc_epoch;}
}

/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void setup() {
  Serial.begin(9600);               // Start the serial terminal
  // Subscribe to the integration response event
  Particle.subscribe("hook-response/input_controls", myHandler, MY_DEVICES);
  
  adxl.powerOn();                   // Power on the ADXL345

  adxl.setRangeSetting(2);          // Give the range settings
                                      // Accepted values are 2g, 4g, 8g or 16g
                                      // Higher Values = Wider Measurement Range
                                      // Lower Values = Greater Sensitivity
                                      
  adxl.setSpiBit(0);                // Configure the device: 4 wire SPI mode = '0' or 3 wire SPI mode = 1
                                      // Default: Set to 1
                                      // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library 
}

/****************** MAIN CODE ******************/
/*    Accelerometer Calibration and Readings   */
void loop() {
  Serial.println("SparkFun ADXL345 Accelerometer Breakout Calibration");
  Serial.println();
  waitForCalibSignal(); // Trigger the integration to update the epoch
  // Perform calibration
  int dir, pos_accel, neg_accel;
  float offsetAxis, gainAxis;
  for (int axis_i = 0; axis_i < NUM_AXES; axis_i++) {
    // Start with positive direction
    dir = (axis_i*2); // Range: {0,2,4}
    getAxisAccelForCalib(pos_accel, dir);
    // Now do negative direction
    dir++; // Range: {1,3,5}
    getAxisAccelForCalib(neg_accel, dir);
    // Calculate gain and offset (https://learn.sparkfun.com/tutorials/adxl345-hookup-guide)
    // Formula: True = (Raw - Offset)/Gain
    gainAxis = 0.5*((pos_accel - neg_accel)/(1*G_FRAC)); // The 1 in the denominator is there since the accel is 1G
    offsetAxis = (0.5*(pos_accel + neg_accel));
    switch (axis_i) {
      case 0: gainX = gainAxis; offsetX = offsetAxis; break;
      case 1: gainY = gainAxis; offsetY = offsetAxis; break;
      case 2: gainZ = gainAxis; offsetZ = offsetAxis; break;
    }
  }

  // Print the gains to serial
  Serial.println();
  Serial.println("XYZ Gains:\t" + String(gainX,2) + ",\t" + String(gainY,2) + ",\t" + String(gainZ,2));
  Serial.println("XYZ Offsets:\t" + String(offsetX,2) + ",\t" + String(offsetY,2) + ",\t" + String(offsetZ,2));
  Serial.println("Formula: True = (Raw - Offset)/Gain");
  Serial.println();

  // Make an infinite loop to avoid re-calibrating
  int x,y,z,accX,accY,accZ; // Initialize variables for infinite loop
  while (true) {
    Serial.println("Press the Calib. Accel. button on Initial State to measure acceleration.");
    waitForCalibSignal();
    Serial.println();
    
    // Get the accelerometer readings
    adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store in variables x,y,z
    // Apply offset and gain
    accX = (x - offsetX)/gainX;         // Calculating New Values for X, Y and Z
    accY = (y - offsetY)/gainY;
    accZ = (z - offsetZ)/gainZ;

    Serial.println("Calib Accel: " + String(accX) + ", " + String(accY) + ", " + String(accZ));
    Serial.println();
  }
}
