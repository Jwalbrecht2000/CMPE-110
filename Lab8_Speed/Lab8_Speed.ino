// ===========================================================
//
// Lab8_Speed.ino  
// Description: Arduino robot speed
// Name: <team member names here>
// Date: <today's date here>
// Class: CMPE-110
// Section: <Lab: section, day, and time here>

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
  // uncomment to test normal speed
  speedNormal();

  // uncomment to test slow speed
  //speedSlow();
}



// Function:    speedSlow
// Description: try different slow sppeds
// Inputs:      none
// Returns:     none
// 
void speedSlow()
{
  int i;               // loop control variable 

  pressToStart();      // wait for the 'start' button to be pressed

  // start at speed 20 and go by 20, up to 100
  for (i = 20; i <= 100; i += 20)
  {
    // blink LED so we know when the next speed will be tried
    digitalWrite(13, HIGH);   // turn the LED on
    Serial.println("HIGH");
    delay(2000);               // wait for 2 seconds
    digitalWrite(13, LOW);    // turn the LED off
    Serial.println("LOW");

    fwd(i);            // go forward at speed i
    wait(2000);        // attempt to drive for 2 sec
    stopMotors();      // stop driving
  }

  stopMotors();        // stop the motors
}



// Function:    speedNormal
// Description: try different normal sppeds
// Inputs:      none
// Returns:     none
// 
void speedNormal()
{
  int i;               // loop control variable 

  // start at speed 80 and go by 20, up to 180
  for (i = 80; i <= 180; i += 20)
  {
    pressToStart();    // wait for the 'start' button to be pressed

    fwd(i);            // go forward at speed i
    wait(2000);        // attempt to drive for 2 sec
    stopMotors();      // stop driving
  }

  stopMotors();        // stop the motors
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
