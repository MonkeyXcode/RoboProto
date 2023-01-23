#include <AFMotor.h>
#include <Servo.h> 
#include <NewPing.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define TRIG_PIN A2 // Pin A4 on the Motor Drive Shield soldered to the ultrasonic sensor
#define ECHO_PIN A3 // Pin A5 on the Motor Drive Shield soldered to the ultrasonic sensor
#define MAX_DISTANCE_POSSIBLE 1000 // sets maximum useable sensor measuring distance to 200cm
#define MAX_SPEED 200 // sets speed of DC traction motors to 180/256 or about 70% of full speed - to get power drain down.
#define MOTORS_CALIBRATION_OFFSET 3 // this sets offset to allow for differences between the two DC traction motors
//#define COLL_DIST 20 // sets distance at which robot stops and reverses to 10cm
#define COLL_DIST 20 // sets distance at which robot stops and reverses to 10cm
#define TURN_DIST COLL_DIST+0 // sets distance at which robot looks away from object (not reverse) to 20cm (10+10)
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE_POSSIBLE); // sets up sensor library to use the correct pins to measure distance.
#define trigPin A2
#define echoPin A3

AF_DCMotor leftMotor(3, MOTOR34_1KHZ); // create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
AF_DCMotor cutterMotor(1, MOTOR12_8KHZ);
AF_DCMotor rightMotor(4, MOTOR34_1KHZ); // create motor #2, using M2 output, set to 1kHz PWM frequency

Servo neckControllerServoMotor;  // create servo object to control a servo 

int cutting = 0; //0= no cutting 1= cutting
int pos = 0; // this sets up variables for use in the sketch (code)
  int maxDist = 0;
  int maxAngle = 0;
  int maxRight = 0;
  int maxLeft = 0;
  int maxFront = 0;
int course = 0;
int curDist = 0;
String motorSet = "";
int speedSet = 0;
float distance_cm;
//char mystr[5] = "cutt"; //String data
int analogPin = A0;
const int averageValue = 10;
long int sensorValue = 0;  // variable to store the sensor value read

float voltage = 0;
float current = 0;
//-------------------------------------------- SETUP LOOP ----------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  cutterstart();
  

  

 // Serial.write(mystr,5); //Write the serial data
 // delay(1);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  tulostaLogo();
  delay(2000);

  textWrite();
  display.setTextSize(3);
  display.setCursor(0, 0);
  display.println("Rambo");
  display.println("Robotti");
  delay(5000);
  //display.println("ROBOOTTI RAMBO");
  delay(2000);
  //display.display();

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  
  neckControllerServoMotor.attach(9);  // attaches the servo on pin 9 (SERVO_2 on the Motor Drive Shield to the servo object 
  neckControllerServoMotor.write(90); // tells the servo to position at 90-degrees ie. facing forward.
  delay(2000); // delay for two seconds
  checkPath(); // run the CheckPath routine to find the best path to begin travel
  motorSet = "FORWARD"; // set the director indicator variable to FORWARD
  neckControllerServoMotor.write(90); // make sure servo is still facing forward
  moveForward(); // run function to make robot move forward
}
//------------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------MAIN LOOP ------------------------------------------------------------------------------
void loop() {
//          Serial.write(mystr,5);
  checkForward(); // check that if the robot is supposed to be moving forward, that the drive motors are set to move forward - this is needed to overcome some issues with only using 4 AA NiMH batteries
  checkPath(); // set ultrasonic sensor to scan for any possible obstacles
  metter();
}
//-------------------------------------------------------------------------------------------------------------------------------------
void checkPath() {

  textWrite();
  display.setTextSize(1);
  display.println("CHECK PATH");
  display.setTextSize(2);
  display.println("DISTANCE ");
  display.println("----------");
  
  display.println(distance_cm);
  display.display();
  
  int curLeft = 0;
  int curFront = 0;
  int curRight = 0;
  int curDist = 0;
  //neckControllerServoMotor.write(144); // set servo to face left 54-degrees from forward
  //neckControllerServoMotor.write(90); // set servo to face left 54-degrees from forward
  neckControllerServoMotor.write(144); // set servo to face left 54-degrees from forward
  delay(120); // wait 120milliseconds for servo to reach position
  //for(pos = 144; pos >= 36; pos-=18)     // loop to sweep the servo (& sensor) from 144-degrees left to 36-degrees right at 18-degree intervals. 
  for(pos = 144; pos >= 36; pos-=18)     // loop to sweep the servo (& sensor) from 144-degrees left to 36-degrees right at 18-degree intervals. 
  {
    //neckControllerServoMotor.write(pos);  // tell servo to go to position in variable 'pos' 
    //delay(90); // wait 90ms for servo to get to position   !!MIKA
    neckControllerServoMotor.write(pos);  // tell servo to go to position in variable 'pos' 
    delay(90); // wait 90ms for servo to get to position 
    checkForward(); // check the robot is still moving forward
    curDist = readPing(); // get the current distance to any object in front of sensor
    if (curDist < COLL_DIST) { // if the current distance to object is less than the collision distance
      checkCourse(); // run the checkCourse function
      break; // jump out of this loop
    }
    if (curDist < TURN_DIST) { // if current distance is less than the turn distance
      changePath(); // run the changePath function
    }
    if (curDist > curDist) {maxAngle = pos;}
    if (pos > 90 && curDist > curLeft) { curLeft = curDist;}
    if (pos == 90 && curDist > curFront) {curFront = curDist;}
    if (pos < 90 && curDist > curRight) {curRight = curDist;}
  }
  maxLeft = curLeft;
  maxRight = curRight;
  maxFront = curFront;
}
//-------------------------------------------------------------------------------------------------------------------------------------
void setCourse() { // set direction for travel based on a very basic distance map, simply which direction has the greatest distance to and object - turning right or left? 
    textWrite();
    display.setTextSize(1.5);
    display.println("SET COURSE");
    display.setTextSize(2);
    display.println("DISTANCE ");
    display.println("----------");
    display.println(distance_cm);
    display.display();
  
    if (maxAngle < 90) {turnRight();}
    if (maxAngle > 90) {turnLeft();}
    maxLeft = 0;
    maxRight = 0;
    maxFront = 0;
}
//-------------------------------------------------------------------------------------------------------------------------------------
void checkCourse() { // we're about to hit something so move backwards, stop, find where the empty path is.
  textWrite();
  display.setTextSize(1.5);
  display.println("CHECK COURCE");
  display.setTextSize(2);
  display.println("DISTANCE ");
  display.println("----------");
  display.println(distance_cm);
  display.display();
  moveBackward();
  //delay(5000);
  delay(2000);
  moveStop();
  setCourse();
}
//-------------------------------------------------------------------------------------------------------------------------------------
void changePath() {
  textWrite();
  display.setTextSize(1.5);
  display.println("CHANGE PATH");
  display.setTextSize(2);
  display.println("DISTANCE ");
  display.println("----------");
  display.println(distance_cm);
  display.display();
  if (pos < 90) {lookLeft();} // if current pos of sensor is less than 90-degrees, it means the object is on the right hand side so look left
  if (pos > 90) {lookRight();} // if current pos of sensor is greater than 90-degrees, it means the object is on the left hand side so look right
}
//-------------------------------------------------------------------------------------------------------------------------------------

int readPing() { // read the ultrasonic sensor distance
  delay(70);
  //unsigned int uS = sonar.ping();
  //unsigned int uS = 0;
  //int cm = uS/US_ROUNDTRIP_CM;
  //int cm = 0;

//Serial.begin (9600);
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
float duration;
//float distance_cm;
float distance_in;

digitalWrite(trigPin, LOW); //PULSE ___|---|___
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

duration = pulseIn(echoPin, HIGH);

distance_cm = (duration/2) / 29.1;
distance_in = (duration/2) / 73.914;

  textWrite();
  display.setTextSize(1.5);
  display.println("Current ");
  display.setTextSize(2);
  display.println("DISTANCE ");
  display.println("----------");
  display.println(distance_cm);
  display.display();
  //delay(4000);
  //cm = distance_cm; 
  //return cm;
  return distance_cm;
}
//-------------------------------------------------------------------------------------------------------------------------------------
void checkForward() {
  textWrite();
  display.setTextSize(1.5);
  display.println("CHECK FORWARD");
  display.setTextSize(2);
  display.println("DISTANCE ");
  display.println("----------");
  display.println(distance_cm);
  display.display();
  if (motorSet=="FORWARD") {leftMotor.run(FORWARD); rightMotor.run(FORWARD); } 
  }     // make sure motors are going forward
//-------------------------------------------------------------------------------------------------------------------------------------
void checkBackward() { 
  textWrite();
  display.setTextSize(1.5);
  display.println("CHECK BACKWARD");
  display.setTextSize(2);
  display.println("DISTANCE ");
  display.println("----------");
  display.println(distance_cm);
  display.display();
  if (motorSet=="BACKWARD") {leftMotor.run(BACKWARD); rightMotor.run(BACKWARD); } 
  } // make sure motors are going backward
//-------------------------------------------------------------------------------------------------------------------------------------

// In some cases, the Motor Drive Shield may just stop if the supply voltage is too low (due to using only four NiMH AA cells).
// The above functions simply remind the Shield that if it's supposed to go forward, then make sure it is going forward and vice versa.

//-------------------------------------------------------------------------------------------------------------------------------------
void moveStop() {
  textWrite();
  display.setTextSize(1.5);
  display.println("MOVE STOP");
  display.setTextSize(2);
  display.println("DISTANCE ");
  display.println("----------");
  display.println(distance_cm);
  display.display();
  leftMotor.run(RELEASE); rightMotor.run(RELEASE);
  }  // stop the motors.
//-------------------------------------------------------------------------------------------------------------------------------------
void moveForward() {
    metter();
    textWrite();
    display.setTextSize(1.5);
    display.println("MOVE FORWARD");
    display.setTextSize(2);
    display.println("DISTANCE ");
    display.println("----------");
  display.println(distance_cm);
    display.display();
    motorSet = "FORWARD";
    leftMotor.run(FORWARD);      // turn it on going forward
    rightMotor.run(FORWARD);      // turn it on going forward
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    leftMotor.setSpeed(speedSet+MOTORS_CALIBRATION_OFFSET);
    rightMotor.setSpeed(speedSet);
    delay(5);
  }
}
//-------------------------------------------------------------------------------------------------------------------------------------
void moveBackward() {
    metter();
    textWrite();
    display.setTextSize(1.5);
    display.println("MOVE BACKWARD");
    display.setTextSize(2);
    display.println("DISTANCE ");
    display.println("----------");
  display.println(distance_cm);
    display.display();
    motorSet = "BACKWARD";
    leftMotor.run(BACKWARD);      // turn it on going forward
    rightMotor.run(BACKWARD);     // turn it on going forward
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    leftMotor.setSpeed(speedSet+MOTORS_CALIBRATION_OFFSET);
    rightMotor.setSpeed(speedSet);
    delay(5);
  }
}  

void cutterstart()
{
  if (cutting == 0)
  {     //motorSet = "FORWARD";
//        Serial.write(mystr,5);
        cutterMotor.run(FORWARD);
       cutterMotor.setSpeed(250);
        cutting = 1;
  }  
}
//-------------------------------------------------------------------------------------------------------------------------------------
void turnRight() {
  textWrite();
  display.setTextSize(1.5);
  display.println("TURN RIGHT");
  display.setTextSize(2);
  display.println("DISTANCE ");
  display.println("----------");
  display.println(distance_cm);
  display.display();
  motorSet = "RIGHT";
  leftMotor.run(FORWARD);      // turn motor 1 forward
  rightMotor.run(BACKWARD);     // turn motor 2 backward
  //delay(4000); // run motors this way for 400ms
  delay(1500); // run motors this way for 400ms
  motorSet = "FORWARD";
  leftMotor.run(FORWARD);      // set both motors back to forward
  rightMotor.run(FORWARD);      
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void turnLeft() {
  textWrite();
  display.setTextSize(1.5);
  display.println("TURN LEFT");
  display.setTextSize(2);
  display.println("DISTANCE ");
  display.println("----------");
  display.println(distance_cm);
  display.display();
  motorSet = "LEFT";
  leftMotor.run(BACKWARD);     // turn motor 1 backward
  rightMotor.run(FORWARD);      // turn motor 2 forward
  //delay(4000); // run motors this way for 400ms
  delay(1500); // run motors this way for 400ms
  motorSet = "FORWARD";
  leftMotor.run(FORWARD);      // turn it on going forward
  rightMotor.run(FORWARD);      // turn it on going forward
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void lookRight() {
  textWrite();
  display.setTextSize(1.5);
  display.println("LOOK RIGHT");
  display.setTextSize(2);
  display.println("DISTANCE ");
  display.println("----------");
  display.println(distance_cm);
  display.display();
rightMotor.run(BACKWARD); delay(400); rightMotor.run(FORWARD);} // looking right? set right motor backwards for 400ms

//-------------------------------------------------------------------------------------------------------------------------------------
void lookLeft() {
    textWrite();
    display.setTextSize(1.5);
  display.println("LOOK LEFT");
  display.setTextSize(2);
  display.println("DISTANCE ");
  display.println("----------");
  display.println(distance_cm);
  display.display();
  leftMotor.run(BACKWARD); delay(800); leftMotor.run(FORWARD);} // looking left? set left motor backwards for 400ms
//-------------------------------------------------------------------------------------------------------------------------------------

void textWrite(){
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
 }

void metter(){
  for (int i = 0; i < averageValue; i++)
  {
    sensorValue += analogRead(analogPin);

    // wait 2 milliseconds before the next loop
   // delay(1);
  }

  sensorValue = sensorValue / averageValue;
  voltage = sensorValue * 5.0 / 1024.0;
  current = (voltage - 2.5) / 0.185;

  Serial.print("ADC Value: ");
  Serial.print(sensorValue);

  Serial.print("   ADC Voltage: ");
  Serial.print(voltage);
  Serial.print("V");

  Serial.print("   Current: ");
  Serial.print(current);
  Serial.println("A");
}

void tulostaLogo(){
/*static const uint8_t logo_bmp2[] =
{ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xff, 0xff, 0xf8, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xe0, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x07, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xfe, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x0f, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x81, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xfc, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xff, 0xff, 
  0xff, 0xff, 0xf0, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 
  0xff, 0xff, 0xc1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x83, 0xff, 0xff, 
  0xff, 0xff, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xff, 0xff, 
  0xff, 0xfc, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x3f, 0xff, 
  0xff, 0xf8, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x1f, 0xff, 
  0xff, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0xff, 
  0xff, 0xc3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe6, 0x4f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc3, 0xff, 
  0xff, 0x87, 0xff, 0xff, 0xff, 0xff, 0xff, 0x88, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe1, 0xff, 
  0xff, 0x0f, 0xff, 0xff, 0xff, 0xfe, 0x03, 0xf0, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xf0, 0xff, 
  0xfe, 0x1f, 0xff, 0xff, 0xff, 0x8f, 0xff, 0xfc, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xf8, 0x7f, 
  0xfc, 0x3f, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x7f, 0xe0, 0x03, 0x30, 0x7f, 0xff, 0xfc, 0x3f, 
  0xf8, 0x7f, 0xff, 0xff, 0x3f, 0x80, 0x08, 0x00, 0x00, 0x00, 0x40, 0x0f, 0xe3, 0xff, 0xfe, 0x1f, 
  0xf8, 0x7f, 0xff, 0x83, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0xfe, 0x1f, 
  0xf0, 0xff, 0xf8, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x0f, 
  0xf0, 0xff, 0x8f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x05, 0xff, 0x0f, 
  0xe1, 0xfe, 0x3c, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0x80, 0x01, 0xff, 0x87, 
  0xe1, 0xff, 0x7c, 0x00, 0x3f, 0xfe, 0x00, 0x0c, 0x00, 0x00, 0x1f, 0xff, 0xc0, 0x01, 0xff, 0x87, 
  0xe1, 0xff, 0x3e, 0x00, 0x7f, 0xfe, 0x00, 0x30, 0x00, 0x00, 0x07, 0xff, 0xe0, 0x05, 0xff, 0x87, 
  0xe1, 0xff, 0x9e, 0x00, 0x3f, 0xf4, 0x00, 0x30, 0x00, 0x00, 0x03, 0xff, 0xc0, 0x0b, 0xff, 0x87, 
  0xe1, 0xff, 0x9e, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x0b, 0xff, 0x87, 
  0xe1, 0xff, 0xce, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xff, 0x87, 
  0xe1, 0xff, 0xe7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xff, 0x87, 
  0xe1, 0xff, 0xf3, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0xff, 0x87, 
  0xf1, 0xff, 0xf9, 0xf0, 0x00, 0x4f, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x1f, 0xff, 0x87, 
  0xf0, 0xff, 0xfc, 0xfe, 0x0f, 0xff, 0xf8, 0xe0, 0x00, 0xcf, 0xff, 0xff, 0xf4, 0x3f, 0xff, 0x0f, 
  0xf0, 0xff, 0xfc, 0x7f, 0xf8, 0x7f, 0xfe, 0x70, 0x01, 0xbf, 0x03, 0xc0, 0x00, 0xff, 0xff, 0x0f, 
  0xf8, 0x7f, 0xff, 0x8f, 0xff, 0xff, 0xff, 0x3c, 0x20, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x1f, 
  0xfc, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xf1, 0xfe, 0x06, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x3f, 
  0xfc, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xc6, 0x01, 0x00, 0x4f, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x3f, 
  0xfe, 0x0f, 0xff, 0xff, 0xff, 0xff, 0x98, 0x00, 0x08, 0x37, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x7f, 
  0xff, 0x07, 0xff, 0xff, 0xff, 0xff, 0x20, 0x00, 0x04, 0x0b, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xff, 
  0xff, 0x83, 0xff, 0xff, 0xff, 0xff, 0x40, 0x00, 0x00, 0x05, 0xff, 0xff, 0xff, 0xff, 0xc1, 0xff, 
  0xff, 0xe1, 0xff, 0xff, 0xff, 0xfe, 0x80, 0x00, 0x02, 0x02, 0xff, 0xff, 0xff, 0xff, 0x87, 0xff, 
  0xff, 0xf0, 0x7f, 0xff, 0xff, 0xfc, 0x80, 0x00, 0x01, 0x02, 0xff, 0xff, 0xff, 0xfe, 0x0f, 0xff, 
  0xff, 0xfc, 0x3f, 0xff, 0xff, 0xf9, 0x80, 0x00, 0x01, 0x01, 0x7f, 0xff, 0xff, 0xfc, 0x1f, 0xff, 
  0xff, 0xfe, 0x0f, 0xff, 0xff, 0xfb, 0x00, 0x00, 0x01, 0x01, 0x3f, 0xff, 0xff, 0xf0, 0x7f, 0xff, 
  0xff, 0xff, 0x83, 0xff, 0xff, 0xfb, 0x00, 0x00, 0x01, 0x00, 0xbf, 0xff, 0xff, 0xc1, 0xff, 0xff, 
  0xff, 0xff, 0xe0, 0xff, 0xff, 0xf8, 0xf0, 0x00, 0x00, 0x06, 0x7f, 0xff, 0xff, 0x07, 0xff, 0xff, 
  0xff, 0xff, 0xf8, 0x1f, 0xff, 0xff, 0xef, 0x80, 0x00, 0x05, 0xff, 0xff, 0xf8, 0x1f, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0x07, 0xff, 0xff, 0xff, 0xe0, 0x00, 0xff, 0xff, 0xff, 0xe0, 0x7f, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xc0, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x03, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xfe, 0x08, 0x17, 0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0x03, 0xfc, 0xfc, 0x7f, 0xf8, 0xff, 0x9f, 0x1f, 0xf3, 0x88, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xdc, 0xf8, 0xfc, 0x8f, 0xf9, 0x8f, 0x0f, 0x39, 0xe1, 0xf9, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0x93, 0xf3, 0x7c, 0xf7, 0xf9, 0x8e, 0x67, 0x27, 0xcc, 0xf9, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0x8c, 0x63, 0x7c, 0xef, 0xf8, 0x7c, 0xf3, 0x08, 0x9e, 0xf9, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0x9c, 0xe7, 0x2c, 0x9f, 0xf8, 0xf8, 0xfb, 0x39, 0x1e, 0x79, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0x93, 0xc7, 0xbc, 0x3f, 0xf8, 0x39, 0xf3, 0x27, 0x3e, 0x79, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0x87, 0xdf, 0xbc, 0x7f, 0xf9, 0x89, 0xcf, 0x1f, 0x31, 0xf9, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0x9f, 0x9f, 0xf8, 0xff, 0xfd, 0xe8, 0x7e, 0x7f, 0x0f, 0xfd, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
  };
/*static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };
*/
#define LOGO_HEIGHT   128
#define LOGO_WIDTH    64

//  display.drawBitmap(0,0,logo_bmp2, LOGO_WIDTH, LOGO_HEIGHT, 1);
  //display.display();
 // delay(1000);

}
