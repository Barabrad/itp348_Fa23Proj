/* 
 * ITP-348 Project
 * Author: Brad Barakat
 * Semester: Fall 2023
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

typedef unsigned long u_long;

#include <cmath>
// Include accelerometer library
#include "SparkFun_ADXL345.h"
// Include ArduinoJson library
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#include <ArduinoJson.h> 


/*********** COMMUNICATION SELECTION ***********/
/*    Comment Out The One You Are Not Using    */
// ADXL345 adxl = ADXL345(10);           // USE FOR SPI COMMUNICATION, ADXL345(CS_PIN);
ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION

/***************** PIN LABELS ******************/
/*                                             */
const int PIN_BUZZER = D2;
const int PIN_LIGHT = A5;

/******************* GLOBALS *******************/
/*                                             */
// General
const float MAX_ANLG = 4095;
const float MIN_ANLG = 0;
u_long currMillis = 0;
enum ArrOp {Plus, Minus, Mult, Div};
// Buzzer variables
const int PITCH_ANG = 440; // Speaker output when accelerometer threshold is tripped
const int PITCH_LIG = 880; // Speaker output when light threshold is tripped
const u_long INTERVAL_TONE = 500; // ms
// Accelerometer variables
float currAngle = 0;
float angThresh = 30;
bool calibAccel = true; // Calibrate right away
float calAccSamps = 5; // This has to be a float for the arrayOperator function
int CAS_i = 0; // calAccSample_i
u_long calAccDelay = 100; // ms
u_long prevAccMillis = 0;
float calAccSums[] = {0,0,0};
u_long prevAccCalEpoch = 0;
const int NUM_AXES = 3;
enum Axis {XAxis, YAxis, ZAxis};
// The gains and offsets are chosen so that the accel output is in (1/16)G increments
const float GAINS[] = {-16, -15.5, -15};
const float OFFSETS[] = {2.5, 7, 14};
float g_vec[] = {0, 0, -1}; // This gets updated when the Calib. Accel. button is pressed
// Light sensor variables
int currLight = 0;
int ligThresh = 3000;
// Publishing and reading
const u_long READ_INTERVAL = 5000; // Get data from Initial State
u_long prevRead = 0;
const u_long APPEND_INTERVAL = 1000; // Add to JSON
u_long prevAppend = 0;
const int APPS_B4_PUB = 8; // The number of items per published JSON
int apps = 0; // Nothing has been appended to the JSON yet
const size_t MAX_JSON_SIZE = 1048; // Found to be the limit for publishing
StaticJsonDocument<MAX_JSON_SIZE> doc;

// Hopefully this page about webhook trigger limits is true:
// https://docs.particle.io/reference/cloud-apis/webhooks/#limits


/************** HELPER FUNCTIONS ***************/
/*                                             */
/**
 * @brief  This function performs arithmetic with two floats
 * @param  a : The first float
 * @param  b : The second float
 * @param  op : The operation (from the ArrOp enum)
 * @retval  res : The result from the expression "a op b"
 */
float TwoNumOp(float a, float b, ArrOp op) {
  float res = 0;
  switch(op) {
    case Plus: res = a + b; break;
    case Minus: res = a - b; break;
    case Mult: res = a * b; break;
    default: res = a / b; break;
  }
  return res;
}

/**
 * @brief  This function performs arithmetic with an array of floats and a float
 * @param  arrA : The array of floats, which will be first in the expression
 * @param  Bi : The float, which will be second in the expression
 * @param  arrRes : The array of floats that will hold the results
 * @param  resLen : The length of the result array
 * @param  op : The operation (from the ArrOp enum)
 * @retval  void
 */
void arrayOperator(float* arrA, float Bi, float* arrRes, int resLen, ArrOp op) {
  // 0 = a+b, 1 = a-b, 2 = a*b, 3 = a/b
  // Check types
  for (int i = 0; i < resLen; i++) {
    float Ai = arrA[i];
    arrRes[i] = TwoNumOp(Ai, Bi, op);
  }
}

/**
 * @brief  This function performs arithmetic with two arrays of floats
 * @param  arrA : The first array of floats
 * @param  arrB : The second array of floats
 * @param  arrRes : The array of floats that will hold the results
 * @param  resLen : The length of the result array
 * @param  op : The operation (from the ArrOp enum)
 * @retval  void
 */
void arrayOperator(float* arrA, float* arrB, float* arrRes, int resLen, ArrOp op) {
  // 0 = a+b, 1 = a-b, 2 = a*b, 3 = a/b
  // Check types
  for (int i = 0; i < resLen; i++) {
    float Ai = arrA[i];
    float Bi = arrB[i];
    arrRes[i] = TwoNumOp(Ai, Bi, op);
  }
}

/**
 * @brief  This function finds the magnitude of a 2D vector
 * @param  x : The x component
 * @param  y : The y component
 * @retval  res : The magnitude of the vector <x,y>
 */
float findVectMag2D(float x, float y) {
  return sqrtf(x*x + y*y);
}

/**
 * @brief  This function finds the acceleration and applies the gains and offsets
 * @param  cxyz : The array that will hold the results: [x, y, z]
 * @retval  void
 */
void measAccel(float* cxyz) {
  int x,y,z; // Initialize variables that will hold results
  adxl.readAccel(&x, &y, &z); // Read the accelerometer values and store in variables x,y,z
  // Calculate values according to found gains and offsets
  cxyz[XAxis] = (x - OFFSETS[XAxis])/GAINS[XAxis];
  cxyz[YAxis] = (y - OFFSETS[YAxis])/GAINS[YAxis];
  cxyz[ZAxis] = (z - OFFSETS[ZAxis])/GAINS[ZAxis];
}

/**
 * @brief  This function tests measAccel()
 * @param  xyz : The array that will hold the results: [x, y, z]
 * @retval  void
 */
void testAccel(float* xyz) {
  measAccel(xyz);
  Serial.println("Acc (in G/16): " + String(xyz[XAxis]) + ", " + String(xyz[YAxis]) + ", " + String(xyz[ZAxis]));
}

/**
 * @brief  This function finds the angle between the current downward vector and the baseline gravity vector
 * @param  cxyz : The array that will hold the results: [x, y, z]
 * @retval  theta : The angle between the aforementioned vectors, in degrees
 */
float findAngle(float* cxyz) {
  // Measure the acceleration
  measAccel(cxyz);
  // Since the angle is with a vector pointing straight down, we can find the magnitude of
  // the adjusted vector's XY plane component
  float g_XY = findVectMag2D(g_vec[XAxis], g_vec[YAxis]);
  float measXY = findVectMag2D(cxyz[XAxis], cxyz[YAxis]);
  float g_mag = findVectMag2D(g_XY, g_vec[ZAxis]);
  float measMag = findVectMag2D(measXY, cxyz[ZAxis]);
  // cos(theta_radians) = (a•b)/(|a|*|b|)
  float theta_r = acosf((g_XY*measXY + g_vec[ZAxis]*cxyz[ZAxis])/(g_mag*measMag));
  return theta_r*180.0/3.141592654; // Convert to degrees
}

/**
 * @brief  This function makes a JSON object that will be part of the data packet sent to Initial State
 * @param  field : The name of the measurement ("angle", "light", etc.)
 * @param  value : The value of the measurement
 * @param  epoch : The epoch of the measurement
 * @retval  void
 */
void makeJsonObj(String field, float value, time32_t epoch) {
  JsonObject obj = doc.createNestedObject();
  obj["key"] = field;
  obj["value"] = value;
  obj["epoch"] = epoch;
}

/**
 * @brief  This function appends or publishes the data packet sent to Initial State, depending on the size
 * @retval  void
 */
void appendOrPublishJson() {
  // Assume the timer logic is in loop()
  time32_t epoch = Time.now(); // Get the epoch
  apps++; // Increment append counter
  // Add the data
  makeJsonObj("angle", currAngle, epoch);
  makeJsonObj("light", currLight, epoch);
  if (apps == APPS_B4_PUB) {
    // Publish the JSON
    String output;
    serializeJson(doc, output);
    //Serial.println(output); // For troubleshooting
    Particle.publish("angle_and_light", output);
    // Make sure to clear the JSON afterwards
    doc.clear();
    apps = 0;
  }
}

/**
 * @brief  This function reads from an analog pin and makes sure the value is within analog limits
 * @param  pin : The analog pin to be read
 * @retval  x : The analog value, within the limits
 */
int analogReadWithinLims(int pin) {
  int x = analogRead(pin);
  x = (x > MAX_ANLG) ? MAX_ANLG : ((x < MIN_ANLG) ? MIN_ANLG : x);
  return x;
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
  ligThresh = doc["lig_thresh"];
  angThresh = doc["ang_thresh"];

  if (cal_acc_epoch != prevAccCalEpoch) {calibAccel = true; prevAccCalEpoch = cal_acc_epoch;}
}

/*************** MAIN FUNCTIONS ****************/
/*                                             */
void setup() {
  Serial.begin(9600);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LIGHT, INPUT);
  // Set up the time zone
  Time.zone(-8);
  // Subscribe to the integration response event
  Particle.subscribe("hook-response/input_controls", myHandler, MY_DEVICES);
  // Set up the accelerometer
  adxl.powerOn();           // Power on the ADXL345
  adxl.setRangeSetting(2);  // Give the range settings
                              // Accepted values are 2g, 4g, 8g or 16g
                              // Higher Values = Wider Measurement Range
                              // Lower Values = Greater Sensitivity
  adxl.setSpiBit(0);        // Configure the device: 4 wire SPI mode = '0' or 3 wire SPI mode = 1
                              // Default: Set to 1
                              // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library
  prevAccCalEpoch = (u_long)(Time.now()); // Set the calibration epoch so it doesn't calibrate again right away after reading from Initial State
}

void loop() {
  int pitch_play = 0; // The output for the speaker
  float xyz[NUM_AXES]; // Initialize variables that will hold results
  
  //testAccel(xyz); delay(1000); // For troubleshooting

  // Find current angle
  //currAngle = random(-10, 30); // Until wiring is done
  currAngle = findAngle(xyz);

  // Find current light reading
  //currLight = 2500 + currAngle; // Until wiring is done
  currLight = analogReadWithinLims(PIN_LIGHT);

  // Calibrate accelerometer's baseline gravity if needed
  if (calibAccel) {
    currMillis = millis();
    if (currMillis - prevAccMillis >= calAccDelay) {
      prevAccMillis = currMillis; CAS_i++; // Update timer and counter
      // Get acceleration reading and add to sum
      measAccel(xyz);
      arrayOperator(calAccSums, xyz, calAccSums, NUM_AXES, Plus);
    }
    if (CAS_i == calAccSamps) {
      calibAccel = false; CAS_i = 0; // Reset boolean and counter
      arrayOperator(calAccSums, calAccSamps, g_vec, NUM_AXES, Div); // Divide sum by sample number
      arrayOperator(calAccSums, 0.0, calAccSums, NUM_AXES, Mult); // Reset values
    }
  }

  // If a threshold is passed, sound the buzzer
  if (currAngle > angThresh) {pitch_play += PITCH_ANG;}
  if (currLight > ligThresh) {pitch_play += PITCH_LIG;}
  if (pitch_play == 0) {noTone(PIN_BUZZER);} else {tone(PIN_BUZZER, pitch_play, INTERVAL_TONE);}

  // When the time comes to publish data
  currMillis = millis();
  if (currMillis - prevAppend >= APPEND_INTERVAL) {
    prevAppend = currMillis;
    appendOrPublishJson();
  }

  // When the time comes to receive data
  currMillis = millis();
  if (currMillis - prevRead >= READ_INTERVAL) {
    prevRead = currMillis;
    Particle.publish("input_controls", ":)", PRIVATE); // Trigger the integration
  }
}
