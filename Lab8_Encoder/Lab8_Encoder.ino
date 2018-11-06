// ===========================================================
//
// Lab8_Encoder.ino
// Description: Arduino encoder tester
// Name: <team member names here>
// Date: <today's date here>
// Class: CMPE-110
// Section: <Lab: section, day, and time here>
//
// ===========================================================
//
#include <RedBot.h>

// 'Start' button
#define BUTTON_PIN 12

// user LED
#define LED_PIN 13

// the left and right wheel encoders
#define L_ENCODER_PIN A2
#define R_ENCODER_PIN 10

// the left, center and right line sensors
#define L_LINE_SENSOR_PIN A3
#define C_LINE_SENSOR_PIN A6
#define R_LINE_SENSOR_PIN A7
#define LINETHRESHOLD 1024      // threshold to know we are on the line: 1 (very light) to 1024 (very dark)

// the front Sensor
#define TRIG_PIN A1
#define ECHO_PIN A0

// Motor
#define MAX_SPEED 255           // Fastest speed
#define MIN_SPEED 1             // Slowest speed

// Define the motors, encoders and sensor objects
RedBotMotors motors;
RedBotEncoder encoder = RedBotEncoder(L_ENCODER_PIN, R_ENCODER_PIN);
RedBotSensor left = RedBotSensor(L_LINE_SENSOR_PIN);
RedBotSensor center = RedBotSensor(C_LINE_SENSOR_PIN);
RedBotSensor right = RedBotSensor(R_LINE_SENSOR_PIN);

// variables for testing line sensors
long    prevlCount = 0;         // previous left motor encoder count
long    prevrCount = 0;         // previous right motor encoder count



void setup() 
{ 
  Serial.begin(19200); //for serial IO to screen
  Serial.println("Welcome to my encoder tester!");

  // make sure the motors are stopped
  stopMotors();

  // now that wheels are stopped, clear the encoder count
  encoder.clearEnc(BOTH);

  pinMode(BUTTON_PIN, INPUT_PULLUP); // setup 'start' button so it is LOW when pressed
  pinMode(LED_PIN, OUTPUT); // setup user LED to be an output

  // setup front sensor's trigger pin adn set to LOW (so it is not on yet)
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
}

void loop()
{
  // uncomment the following line to print the wheel encoder values
  testEncoder();
} 

    
// Function:    testEncoder
// Description: prints the readings from the wheel encoders, if any have changed.
// Inputs:      none
// Returns:     none
void testEncoder() 
{
  long    lCount = 0;     // left motor encoder counts
  long    rCount = 0;     // right motor encoder counts

  lCount = encoder.getTicks(LEFT);
  rCount = encoder.getTicks(RIGHT);

  // see if anything changed
  if ((lCount != prevlCount) || (rCount != prevrCount))
  {
    // remember new counters
    prevlCount = lCount;
    prevrCount = rCount;

    // print new counter values
    Serial.print("counter:    left: ");
    Serial.print(lCount, DEC);
    Serial.print("\tright: ");
    Serial.print(rCount, DEC);
    Serial.println(" counts");
  }

  delay(100);           // get counters a chance to change
}



// Function:    stopMotors
// Description: stops the wheel motors
// Inputs:      none
// Returns:     none
void stopMotors()
{
  motors.brake(); // use .stop() for a coasting stop
  // to control each motor separately
  //motors.leftDrive(0);
  //motors.rightDrive(0);

  Serial.println("stopped");
}
