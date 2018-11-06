// ===========================================================
//
// Lab8_Drive.ino  
// Description: Arduino encoder tester
// Name: John Albrecht and Joseph Ortiz
// Date: October 23, 2018
// Class: CMPE-110
// Section: Section 1, Tuesday, 5:00PM
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



void setup() 
{ 
  Serial.begin(19200); //for serial IO to screen
  Serial.println("Welcome to my distance tester!");

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
  pressToStart();          // wait for 'Start' button to be pressed
  fwd(120);                // drive forward at speed 120
  waitEncoderLeft(270);    // wait 279 counts
  clearEncoders();         // clear encoders
  turnLeft(120);           // turn to the left at speed 120
  waitEncoderLeft(95);    // wait until it turns enough (100 counts)
  fwd(120);                // drive forward at speed 120
  waitEncoderLeft(270);    // wait 279 counts
  clearEncoders();         // clear encoders
  turnLeft(120);           // turn to the left at speed 120
  waitEncoderLeft(95);    // wait until it turns enough (100 counts)
  fwd(120);                // drive forward at speed 120
  waitEncoderLeft(270);    // wait 279 counts
  clearEncoders();         // clear encoders
  turnLeft(120);           // turn to the left at speed 120
  waitEncoderLeft(95);    // wait until it turns enough (100 counts)
  fwd(120);                // drive forward at speed 120
  waitEncoderLeft(270);    // wait 279 counts
  clearEncoders();         // clear encoders
  turnLeft(120);           // turn to the left at speed 120
  waitEncoderLeft(95);    // wait until it turns enough (100 counts)
  stopMotors();            // stop motors
  clearEncoders();         // clear encoders
}



// Function:    waitEncoderLeft
// Description: waits for left wheel encoder to change the desried amount
// Inputs:      counts - number of encoder counts to wait for
// Returns:     none
void waitEncoderLeft(int counts)
{
  long    lCount = 0;     // left motor encoder counts

  while (true)
  {
    // get current value
    lCount = abs(encoder.getTicks(LEFT));

    // print new counter value
    Serial.print("wait left:  ");
    Serial.print(lCount, DEC);
    Serial.print("\twant: ");
    Serial.print(counts, DEC);
    Serial.println("");

    if (lCount >= counts)
    {
        return;
    }
  }
}



// Function:    waitEncoderRight
// Description: waits for right wheel encoder to change the desried amount
// Inputs:      counts - number of encoder counts to wait for
// Returns:     none
void waitEncoderRight(int counts)
{
  long    rCount = 0;     // right motor encoder counts

  while (true)
  {
    // get current value
    rCount = abs(encoder.getTicks(LEFT));

    // print new counter value
    Serial.print("wait right: ");
    Serial.print(rCount, DEC);
    Serial.print("\twant: ");
    Serial.print(counts, DEC);
    Serial.println("");

    if (rCount >= counts)
    {
        return;
    }
  }
}



// Function:    clearEncoders
// Description: clears encoder counts (this stops the motors)
// Inputs:      none
// Returns:     none
void clearEncoders()
{
  stopMotors(); // stop the motors so coutns do not hcange on us
  wait(250);    // give wheels a chance to actually stop

  // clear the encoder counts
  encoder.clearEnc(LEFT);
  encoder.clearEnc(RIGHT);
}



// Function:    pressToStart
// Description: wait for the 'Start' button to be pressed
// Inputs:      none
// Returns:     none
// 
void pressToStart()
{
  unsigned long startTime;
  bool pressed = false;

  Serial.println(""); // print blank line
  Serial.println("Waiting for 'Start' button to be pressed...");

  // keep blinking LED until button is pressed
  while (!pressed)
  {
    // keep LED off for 1 sec, or until button is pressed
    digitalWrite(LED_PIN, LOW);  // turn LED off
    startTime = millis();        // get current time NOTE: the micros() counter will overflow after ~50 days
    while (!pressed && ((millis() - startTime) < 500))
    {
      // see if button is pressed (i.e. LOW)
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        pressed = true;
      }
    }

    // keep LED on for 1 sec, or until button is pressed
    digitalWrite(LED_PIN, HIGH); // turn LED on
    startTime = millis();        // get current time NOTE: the millis() counter will overflow after ~50 days
    while (!pressed && ((millis() - startTime) < 500))
    {
      // see if button is pressed (i.e. LOW)
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        pressed = true;
      }
    }
  }

  // make sure LED is off
  digitalWrite(LED_PIN, LOW);

  // give user a chance to remove their finger
  wait(1000);

  Serial.println(""); // print blank line
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



// Function:    fwd
// Description: set motor speed so robot drives forward
// Inputs:      speed - the speed the robot will move, 1=slow, 255=fast
// Returns:     none
void fwd(int speed)
{ 
  // validate the speed value
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  if (speed > MAX_SPEED) speed = MAX_SPEED;
  
  // display the function we are in and its values
  Serial.print("forward:    ");
  Serial.println(speed, DEC);  
  
  // start robot moving forward
  motors.drive(speed);
  // to control each motor separately
  //motors.leftDrive(speed);
  //motors.rightDrive(speed);
}



// Function:    rev
// Description: set motor speed so robot drives backwards
// Inputs:      speed - the speed the robot will move, 1=slow, 255=fast
// Returns:     none
void rev(int speed)
{
  // validate the speed value
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  if (speed > MAX_SPEED) speed = MAX_SPEED;
  
  // display the function we are in and its values
  Serial.print("backwards:  ");
  Serial.println(speed, DEC);
  
  // start robot moving backward
  motors.drive(-speed);
  // to control each motor separately
  //motors.leftDrive(-speed);
  //motors.rightDrive(-speed);
}



// Function:    turnLeft
// Description: set motor speed so robot turns to the left
// Inputs:      speed - the speed the robot will turn, 1=slow, 255=fast
// Returns:     none
void turnLeft(int speed)
{
  // validate the speed value
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  if (speed > MAX_SPEED) speed = MAX_SPEED;

  // display the function we are in and its values
  Serial.print("turn left:  ");
  Serial.println(speed, DEC);  
  
  // start turning robot to the left
  // to control each motor separately
  motors.pivot(-speed);
  //motors.leftDrive(-speed);
  //motors.rightDrive(speed);
}



// Function:    turnRight
// Description: set motor speed so robot turns to the right
// Inputs:      speed - the speed the robot will turn, 1=slow, 255=fast
// Returns:     none
void turnRight(int speed)
{
  // validate the speed value
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  if (speed > MAX_SPEED) speed = MAX_SPEED;

  // display the function we are in and its values
  Serial.print("turn right: ");
  Serial.println(speed, DEC);
  
  // start turning robot to the right
  motors.pivot(speed);
  // to control each motor separately
  //motors.leftDrive(speed);
  //motors.rightDrive(-speed);
}



// Function:    wait
// Description: wait for the desired length of time, before this function returns
// Inputs:      duration - number of milliseconds before this function returns
// Returns:     none
void wait(int duration)
{ 
  // display the function we are in and its values
  Serial.print("wait:       ");
  Serial.print(duration, DEC);
  Serial.println("ms");

  // wait for the desired time
  delay(duration);
}
