#include <Servo.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
// setup servo
//#define SERVORIGHT   50
//#define SERVOCENTRE  100
//#define SERVOLEFT    150
//#define SERVOPIN     14
//#define TRIGPIN      13    // pin no. 13  is D7 ESP8266
//#define ECHOPIN      12    // pin no. 15  is D8 ESP8266
const int TRIGPIN = 13;  //D7
const int ECHOPIN = 15;  //D8


/*#define RightMotorSpeedPin 5
#define RightMotorDirPin   0
#define LeftMotorSpeedPin  4
#define LeftMotorDirPin    2
*/
int SERVORIGHT = 50;
int SERVOCENTRE = 100;
int SERVOLEFT = 150;
int SERVOPIN = 14;
const int MA1 = 5; //D1
const int MA2 = 16; //D0
const int MB1 = 4; //D2
const int MB2 = 0; //D3
const int ENA = 10;//D5
const int ENB = 12;//D6

int Speed = 900;  // max 1024
int TSpeed = 1020;  //Turning Speed
Servo servo;
#define WIFI_SSID "Ajaja"
#define WIFI_PASS "espajaja"
WiFiUDP UDP;

IPAddress remote_IP(192,168,4,1);
//IPAddress local_IP(192,168,4,1);
//IPAddress gateway(192,168,4,1);
//IPAddress subnet(255,255,255,0);
#define UDP_PORT 4210


void stop()
{
  //Apply speed zero for stopping motors

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(MA1,LOW);
  digitalWrite(MA2,LOW);
  digitalWrite(MB1,LOW);
  digitalWrite(MB2,LOW);
    Serial.println("Stop");
}

void forward()
{
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(MA1,HIGH);
  digitalWrite(MA2,LOW);
  digitalWrite(MB1,HIGH);
  digitalWrite(MB2,LOW);
    Serial.println("forward");
}

void back()
{
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(MA1,LOW);
  digitalWrite(MA2,HIGH);
  digitalWrite(MB1,LOW);
  digitalWrite(MB2,HIGH);
    Serial.println("Back");
}

void left()
{
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(MA1,LOW);
  digitalWrite(MA2,HIGH);
  digitalWrite(MB1,HIGH);
  digitalWrite(MB2,LOW);
    Serial.println("Left");
}

void right()
{
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(MA1,HIGH);
  digitalWrite(MA2,LOW);
  digitalWrite(MB1,LOW);
  digitalWrite(MB2,HIGH);
    Serial.println("right");
}

int stopCount = 0;

int ping()
{
    // send ping
    digitalWrite(TRIGPIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGPIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGPIN, LOW);

    // read echo
    long duration = pulseIn(ECHOPIN, HIGH);

    // convert distance to cm
    unsigned int centimetres = int(duration / 2 / 29.1);

    return centimetres;
}

char scan()
{
    // ping times in microseconds
    unsigned int left_scan, centre_scan, right_scan;
    char choice;

    // scan left
    servo.write(SERVOLEFT);
    //300
    delay(400);
    left_scan = ping();

    // scan right
    servo.write(SERVORIGHT);
    //600
    delay(800);
    right_scan = ping();

    // scan straight ahead
    servo.write(SERVOCENTRE);
    //300
    delay(400);
    centre_scan = ping();

    if (left_scan>right_scan && left_scan>centre_scan)
    {
        choice = 'L';
    }
    else if (right_scan>left_scan && right_scan>centre_scan)
    {
        choice = 'R';
    }
    else {
      choice = 'C';
    }

    return choice;
}

void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);
    Serial.println("RoboProto");

// Begin WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.mode(WIFI_STA);

  // Connecting to WiFi...
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  // Loop continuously while WiFi is not connected
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }

  // Connected to WiFi
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  // Begin UDP port
  UDP.begin(UDP_PORT);
  Serial.print("Opening UDP port ");
  Serial.println(UDP_PORT);

    servo.attach(SERVOPIN);

 pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
   pinMode(MB1, OUTPUT);
    pinMode(MB2, OUTPUT);
     pinMode(ENA, OUTPUT);
      pinMode(ENB, OUTPUT);
    // set the trig pin to output (send sound waves)
    pinMode(TRIGPIN, OUTPUT);

    // set the echo pin to input (receive sound waves)
    pinMode(ECHOPIN, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
    // get distance from obstacle straight ahead

  UDP.beginPacket(remote_IP, UDP_PORT);
  UDP.write('X');
  UDP.endPacket();

    delay(10); //TÄTÄ VOI SÄÄTÄÄ

    unsigned int distance = ping();
    Serial.print("Distance: "); Serial.println(distance);
    if (distance < 30 && distance > 0)
    {
        if (distance < 10)
        {
            // turn around
            UDP.beginPacket(remote_IP, UDP_PORT);
            UDP.write('O');
            UDP.endPacket();

            Serial.println("Turn around...");
           // display.drawString(10, 40, "Turn around...") ;
            back();
            delay(6000); //3000
            left();
            delay(3000);//1000
        }
        else
        {
            // stop both motors
            Serial.println("Motor stop...");
            stop();

            // scan for obstacles
            char turn_direction = scan();

            // turn left/right or ignore and go straight
            if (turn_direction == 'L')
            {
              Serial.println("Turn left...");
                back();
                delay(3000);//1500
                left();
                delay(1500);//500
            }
            else if (turn_direction == 'R')
            {
              Serial.println("Turn right...");
                back();
                delay(3000);//1500
                right();
                delay(1500);//500
            }
            else if (turn_direction == 'C')
            {
              stopCount++;
              if(stopCount > 3){
                stopCount = 0;
                Serial.println("Turn back...");
                back();
                delay(3000);//1500
                right();
                delay(1500);//700
              }

            }
        }
    }
    else
    {
        // no obstacle, keep going forward
        Serial.println("No obstacle, keep going forward...");
        forward();
    }
  //Siirrä alla olevat 3 riviä paikkaan missä haluat lopettaa leikkaamisen
 // UDP.beginPacket(remote_IP, UDP_PORT);
 // UDP.write('O');
 // UDP.endPacket();
}
