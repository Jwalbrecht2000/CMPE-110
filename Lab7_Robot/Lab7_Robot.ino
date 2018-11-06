// ===========================================================
//
// Lab7_Robot.ino  
// Description: Arduino robot move
// Name: John Albrecht, Joseph Ortiz
// Date: October 16, 2018
// Class: CMPE-110
// Section: Lab 1, Tuesday 5:00PM
//
// ===========================================================
//
// Welcome to the CMPE-110 Robotics Lab!
//
// You will only edit the code in loop() method.
// Do NOT change anything else. However, feel free to look it over.
//
// Functions you can use are:
//    stopNow() - stops the wheel motors
//
//    fwd(speed) - robot goes forward
//         speed - the speed the robot will move, 1=slow, 255=fast
//
//    rev(speed) - robot goes backwards
//         speed - the speed the robot will move, 1=slow, 255=fast
//                           
//    turnLeft(speed) - robot turns left
//         speed - the speed the robot will turn, 1=slow, 255=fast
//
//    turnRight(speed) - robot turns right
//         speed - the speed the robot will turn, 1=slow, 255=fast
//
//    wait(duration) - wait for the desired length of time, before this function returns
//         duration - number of milliseconds before this function returns

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
  Serial.println("Welcome to my motor tester!");

  pinMode(BUTTON_PIN, INPUT_PULLUP); // setup 'start' button so it is LOW when pressed
  pinMode(LED_PIN, OUTPUT); // setup user LED to be an output

  // setup front sensor's trigger pin adn set to LOW (so it is not on yet)
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
} 

// ********* Change the code below to program your robot! *********

void loop() 
{ 
  // the robot will execute commands in this function one at at time.
  // when the last command is executed, the loop() function ends.
  // them the loop() function is immediately called again.

  pressToStart();      // wait for the 'start' button to be pressed
  fwd(128);            // go forward at speed 128
  wait(1000);          // drive for 1000 msec = 1 sec
  turnLeft(200);       // turn left at speed 200
  wait(3000);          // drive for 3 seconds
  stopNow();           // stop driving
  wait(1000);          // do nothing for 1 second
  rev(255);            // go backward at speed 255
  wait(500);           // drive for 1/2 second
  stopNow();           // stop driving
  wait(750);           // do nothing for 0.75 second
  turnRight(100);      // turn right at speed 100
  wait(2000);          // drive for 2 seconds
  fwd(200);            // go forward at speed 200
  wait(1000);          // drive for 1 second
  stopNow();           // stop driving
} 

// ********* Don't change any code below here. (But feel free to look and ask questions!) *********

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



// Function:    stop
// Description: stops the wheel motors
// Inputs:      none
// Returns:     none
void stopNow()
{
  motors.brake(); // use .stopNow() for a coasting stop
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
