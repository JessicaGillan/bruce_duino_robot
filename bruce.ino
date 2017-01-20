/* 
//Make sure to include PI file
// Robot implemented with Wireless Communication using nRF24
// Robot is slave
For more info on the wireless module see:http://arduino.stackexchange.com/questions/3042/i2c-2-way-communication-between-arduino-uno-and-arduino-mega
*/

// Declare all Libraries Used
#include<Encoder.h>     // Motor Driver
#include <TimerOne.h>   // Timing for ultrasonic sensor
#include <NewPing.h>    // Library for ultrasonic sensor
#include <Wire.h>           // I2C Communication
#include <SparkFun_APDS9960.h>   // RGB Sensor library
#include <SPI.h>  // WirelessComm: communication interface with the modem
#include "nRF24L01.h" // WirelessComm: handle this particular modem driver
#include "RF24.h" // WirelessComm: library which helps us to control the radio modem
#include "printf.h" // WirelessComm: Use print details

// Declare Pins
#define M1P1 2  //Encoder L pin 1
#define M1P2 6  //Encoder L pin 2
#define M2P1 5  //Encoder R pin 1
#define M2P2 3  //Encoder R pin 2
#define M1PWM 9 //PWM Left motor
#define M2PWM 10  //PWM Right motor
#define MEn 4    //PWM Enable 
#define D1 7    //Direction of motor rotation L, LOW = forward, HIGH= back
#define D2 8    //Direction of motor rotation R, LOW = forward, HIGH= back

//..............................................................................WIRELESS STUFF.................................................../

#define ADDR 121501163904 
#define CE_PIN 32  //CE pin for Wireless
#define CSN_PIN 33 //CS pin for Wireless
#define cmdF 'F' // UART-command for forward
#define cmdB 'B' // UART-command for back
#define cmdR 'R' // UART-command to turn right
#define cmdL 'L' // UART-command to turn left
#define cmdS 'S' // UART-command to stop
RF24 radio( CE_PIN,CSN_PIN ); // Create an object representing modem connected to Arduino
char command;
//static unsigned long tread;
//..............................................................................SONAR, DISTANCE SENSOR STUFF.................................................../
#define TRIGGER_PIN 11  //Distance sensor signal RX pin
#define ECHO_PIN 12     //Distance sensor signal TX pin
#define MAX_DISTANCE 200 //Distance sensor MAX dist range in cm
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
unsigned int d_min = 40; // Stopping distance for obstacle in cm
unsigned int d_ok = 40; // Min distance away from obstacle to reset movement 
unsigned int dcheck = 0; // Grab distance from sonar call
int obst_flag = 0; // Flag to stop forward movement when obstacle
long millis_count;
long dsampletime = 250;


//..............................................................................FEEDBACK 2WD PI STUFF.................................................../
#define MAX_SPEED  75 // PWM value for movement
Encoder motorL(M1P2, M1P1);   // Declare Wheel Encoders
Encoder motorR(M2P1, M2P2);  // Declare Wheel Encoders

int mLeftPI = 0;
int mRightPI = 0;
bool dLeftPI = HIGH; // HIGH = FORWARD, LOW = BACK
bool dRightPI = HIGH;
float newmotorpositionL = 0;     //variable set to read in the instaneous motor1 position in counts
float newmotorpositionR = 0;    //variable set to read in the instaneous motor2 position in counts
float lastmotorpositionL; //hold the left motor position for next velocity calculation 
float lastmotorpositionR; //hold the left motor position for next velocity calculation  
double i = 10000;              //variable set to compare current time to measure when .01 seconds have passed
float current_time = 0;       //stores the value of time when newmotorposition is read in microseconds 
float last_time = 0;        //stores the time that lastmotorposition was read
float velocityR;           //calculates the velocity of the right motor
float velocity2;          //calculates the velocity of the left motor 

int mvc = 0;  //motor voltage control, control motor speed
float errorL;//compares the desired velocity of the Left motor to the actual velocity
float errorR;//compares the desired velocity of the Right motor to the actual velocity
float IL =0; // This the Integrator that elimates any steady state error for the Left motor control
float IR =0; // This the Integrator that elimates any steady state error for the Left motor control
float uL;// PI Control Left motor
float uR;// PI Control Right motor 
//..............................................................................LIGHT INTERRUPT STUFF.................................................../

SparkFun_APDS9960 apds = SparkFun_APDS9960();
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;
uint16_t threshold = 0;
long lsampletime = 100;
int rgb_flag = 0;
float red=0;
float blue=0;
float green=0;
float delta=0;
//..............................................................................SETUP.................................................../
void setup() {
  // Initialize Pins 
  pinMode(MEn, OUTPUT);    //Motor Enable
  pinMode(M1PWM, OUTPUT); // Left motor driver pwm and en
  pinMode(M2PWM, OUTPUT); // Right motor driver pwm and en

  pinMode(D1, OUTPUT); // output for motor rotation L
  pinMode(D2, OUTPUT); // output for motor rotation R
  
  // Initialize Serial communication
  Serial.begin(115200);
  Serial.print("Go"); 

// Initialize Wireless
  printf_begin();
  radio.begin();
  radio.setChannel(47);
  radio.setPALevel(RF24_PA_MAX);
  radio.openReadingPipe(1,ADDR);
  radio.startListening();
  radio.printDetails();
 //////////////////////////rgb/////////////////////////////
  Serial.println();
  Serial.println(F("--------------------------------"));
  Serial.println(F("SparkFun APDS-9960 - ColorSensor"));
  Serial.println(F("--------------------------------"));
  Serial.println(F("--------------------1------------"));
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
  Serial.println(F("------------------2--------------"));
  // Start running the APDS-9960 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {
    Serial.println(F("Light sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during light sensor init!"));
  }

   // Adjust the Proximity sensor gain
  if ( !apds.setProximityGain(PGAIN_2X) ) {
    Serial.println(F("Something went wrong trying to set PGAIN"));
  }
  
  // Start running the APDS-9960 proximity sensor (no interrupts)
  if ( apds.enableProximitySensor(false) ) {
    Serial.println(F("Proximity sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during sensor init!"));
  }
  
  // Wait for initialization and calibration to finish
  delay(500);
   
 //////////////////////////rgb/////////////////////////////

}
//..............................................................................MAIN.................................................../
void loop() {
  millis_count = millis();
  //////////////////////////rgb Target check/////////////////////////////
  // Read the light levels (ambient, red, green, blue) 10 times a second
  if(  millis_count%lsampletime == 0 ){   
    if (  !apds.readRedLight(red_light) ||          // Took out !apds.readAmbientLight(ambient_light) || since we are not using to help with sample time
          !apds.readGreenLight(green_light) ||
          !apds.readBlueLight(blue_light) ) {
      Serial.println("Error reading light values");
    } else {
//      Serial.print("Ambient: ");
//      Serial.print(ambient_light);
//      Serial.print(" Red: ");
//      Serial.print(red_light);
//      Serial.print(" Green: ");
//      Serial.print(green_light);
//      Serial.print(" Blue: ");
//      Serial.println(blue_light);

      red= red_light;
      blue=blue_light;
      green=green_light;
      delta=red/(red+blue+green);
//      Serial.print(" Delta: ");
//      Serial.println(delta,5);
      
  
      if (((red)/(red+blue+green)) >= 0.43  ){
        digitalWrite(MEn, HIGH);    // Enable motor output
        analogWrite(M1PWM,0 ); // set speed for left motor 
        analogWrite(M2PWM,0 ); // set speed for right motor 
        digitalWrite(D1,LOW ); // set direction of left motor rotation 
        digitalWrite(D2,LOW ); // set direction of right motor rotation
       // PI_2WD(0,0, LOW, LOW);       // Stop the robot 
        rgb_flag = 1;   
    //    Serial.println("DONT GO INTO THE LIGHT");
        
                 
      }
   }
  }
///////////////////////////////////////////////////////
 
  // Check for obstacle 4 times a second, stop if obstacle
  if( millis_count%dsampletime == 0 ){
    //Send a ping, returns the distance in centimeters or 0 (zero) if no ping echo within set distance limit 
    dcheck = sonar.ping_cm();
    if (dcheck <= d_min && dcheck != 0){
      digitalWrite(MEn, HIGH);    // Enable motor output
      analogWrite(M1PWM,0 ); // set speed for left motor 
      analogWrite(M2PWM,0 ); // set speed for right motor 
      digitalWrite(D1,LOW ); // set direction of left motor rotation 
      digitalWrite(D2,LOW ); // set direction of right motor rotation
     // PI_2WD(0,0, LOW, LOW);       // Stop the robot
      obst_flag = 1; // Set the flag 
    }
  }
  else{   
    // If there is not an obstacle in range, then check for wireless data
    if( radio.available() ){  
//      Serial.print("test");                                                         
      radio.read(&command, sizeof(char)); // Read the character available and store in command
//      tread = millis();    // Grab time for time out checking
      if( obst_flag || rgb_flag ){
   //     Serial.println("flag is set");
        if( command == cmdR ){
          obst_flag = 0;
          rgb_flag = 0;
     //     Serial.println("Reset flags");
        }
        command = cmdS;
      }
      else if( command == cmdS ){
        digitalWrite(MEn, HIGH);    // Enable motor output
        analogWrite(M1PWM,0 ); // set speed for left motor 
        analogWrite(M2PWM,0 ); // set speed for right motor 
        digitalWrite(D1,LOW ); // set direction of left motor rotation 
        digitalWrite(D2,LOW ); // set direction of right motor rotation    
      }
     // Serial.println(command);
      SetControl2WD(command);  // Send command for parsing
    }
//    else if((millis() - tread) > 50){// If there is a timeout then stop the robot
//      command = 'S';
//      }
//      SetControl2WD(command);  // Send command for parsing
 }
  // Continuously call PI2WD when no new commands are present to update wheels with feedback control
 //PI_2WD(mLeftPI, mRightPI, dLeftPI, dRightPI);       
}
//..............................................................................PARSE COMMANDS AND SEND TO WHEELS.................................................../
// This function controls the wheels based on commands received
void SetControl2WD(char command){
//  Serial.println("got in control " );
  bool directionL, directionR; // direction of motor rotation L298N
  int valueL, valueR; // M1PWM, M2PWM (0-255)

  if(command == cmdF){
    valueL = MAX_SPEED;
    valueR = MAX_SPEED;
    directionL = HIGH;
    directionR = HIGH;
  }
  else if(command == cmdB){
    valueL = MAX_SPEED;
    valueR = MAX_SPEED;
    directionL = LOW;
    directionR = LOW;
  }
  else  if(command == cmdR){
    valueL = 50;
    valueR = 1;
    directionL = HIGH;
    directionR = HIGH;
  }
  else  if(command == cmdL){
    valueL = 1;
    valueR = 50;
    directionL = HIGH;
    directionR = HIGH;
  }
  else if(command == cmdS){
    valueL = 0;
    valueR = 0;
    directionL = HIGH;
    directionR = HIGH;
  }
  else{
    valueL = 0;
    valueR = 0;
    directionL = HIGH;
    directionR = HIGH;
  }
  // Call function to send values to motor
  PI_2WD(valueL, valueR, directionL, directionR);

//   mLeftPI = valueL;
//   mRightPI = valueR;
//   dLeftPI = directionL;
//   dRightPI = directionR;  
}
