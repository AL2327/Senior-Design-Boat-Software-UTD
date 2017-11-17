#include <TinyGPS++.h>  //library for GPS
#include <Wire.h>       //libary for i2c bus
#include <Adafruit_Sensor.h>  //library for imu
#include <Adafruit_LSM303_U.h> //library for imu
#include <Adafruit_L3GD20_U.h>  //library for imu
#include <Adafruit_9DOF.h>  //library for imu
#include <TaskScheduler.h>  //library for task timing/manager
#include <Servo.h>      //library for servo control
#include <PID_v1.h>     //library for PID calculations
#include <Adafruit_FONA.h>    //library for cellular radio
#include <GOFi2cOLED.h> //library for OLED display
#include <stdlib.h>



//*******************************************
//*******************************************

#define _TASK_SLEEP_ON_IDLE_RUN  //tells scheduler to put processor to sleep if doing nothing.
#define _TASK_TIMECRITICAL //allows designation of a time critical task(s) I haven't used this yet or know how -Aaron. 

//Buzzer pin definition
#define buzzer 34

// Definition for voltage sensor input
#define VoltageSense A0   //pin 14

// Definition for water temperature input
#define WaterTemp A18   //pin 37

// Definition for Salinity Input
#define Salinity A19    //pin 38

// Definition for Air Temperature input
#define AirTempPin A7      //pin 21

// Definition for Humidity
#define HumidityPin A8      //pin 22

//Fona definitions
#define FONA_RX 10
#define FONA_TX 9
#define FONA_RST 20
#define HWSERIAL Serial2

Adafruit_FONA fona = Adafruit_FONA(FONA_RST); //passing value of Fona_RST to fona

// The serial connection to the GPS device
#define ss Serial1

/* Assign a unique ID to the IMU sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_vec_t   orientation;

//*******************************************
//*******************************************
      /*LET US DEFINE SOME VARIABLES*/
//*******************************************
//*******************************************


/*LET US DEFINE SOME VARIABLES*/
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA; /* Update this with the correct SLP for accurate altitude measurements */
unsigned long last = 0UL; // For stats that happen every 5 seconds


/*STEERING PID VARIABLES*/
double pos = 90;    // variable to store the servo position. Set it initially to center (neutral posistion)
double pidoutput = 0; //variable for PID Rudder control output.
double Kp = 100;
double Ki = 10;
double Kd = 0;


/*GPS VARIABLES*/
double WaypointLAT[9];    //current waypoint latitude
double WaypointLONG[9];   //current waypoint longitude
double distanceToWaypoint;  //distance to way point in kilometers
double courseToWaypoint;  //course to way point in degrees
int SatFix;
int WPCount = 0;  //variable for waypoint counter

//End GPS variables.

/*IMU Variables */
double Heading = 0; //Current vessel heading in degress.

/*THROTTLE VARIABLES*/
/*0 = full reverse, 90=STOP, 180=full forward*/
int THRT = 90; //variable to stop commanded throttle posistion.

/*Fona Variables*/
char replybuffer[255];  // this is a large buffer for replies
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
int ReplytoPhoneNumber = 0;
char IncCommand[255];
String ComRcv;
boolean ValidMessage;



/*MCU to MCU Easy transfer variables AKA: Sensor Variables*/
//variables for temperature and salinity sensors
float temperature ;
float humidity;
float dewpoint;
float steinhart;
int SalReading;

//Variables for voltage input calculation
float Vout = 0.0;
float R1 = 70000;
float R2 = 10000;
float Vread = 0;
float Vin = 0.0;

//variable for water level sensor
int FloatSwitch;

//variable to assign a "state of charge designator.
int BatterySOC;


//water temmperature input averaging variables
const int WnumReadings = 50;
int WTempSample[WnumReadings];
int WreadIndex = 0;
int Wtotal = 0;
int WTemp;


//water temmperature input averaging variables
const int SnumReadings = 50;
int STempSample[SnumReadings];
int SreadIndex = 0;
int Stotal = 0;
int STemp;

//air temmperature input averaging variables
const int AnumReadings = 50;
int ATempSample[AnumReadings];
int AreadIndex = 0;
int Atotal = 0;
int ATemp;

//Humidity temmperature input averaging variables
const int HnumReadings = 50;
int HTempSample[HnumReadings];
int HreadIndex = 0;
int Htotal = 0;
int HTemp;


int beeping; //variable to store number of beeps we want

//*******************************************
//*******************************************
            /*Object Declarations*/
//*******************************************
//*******************************************



// The TinyGPS++ object
TinyGPSPlus gps;


//Servo Object for Steering
Servo Rudder;

//Servo Object for Throttle
Servo Throttle;

// Define PID controller for rudder
PID PIDRudder(&Heading, &pidoutput, &courseToWaypoint, Kp, Ki, Kd, DIRECT );

//OLED Object for Display Screen
GOFi2cOLED OLED;


//*******************************************
//*******************************************
            /*Task Manager Set Up*/
//*******************************************
//*******************************************

Scheduler runner;
// Callback methods prototypes
void t1Callback();
void t2Callback();
void t3Callback();

// Tasks
Task t1(500, TASK_FOREVER, &t1Callback, &runner, true);  //adding task to do periodic tasks
Task t2(60000, TASK_FOREVER, &t2Callback, &runner, true);  //adding task do fona
Task t3(5000, TASK_FOREVER, &t3Callback, &runner, true);


void t1Callback() {
  Serial.print("IMU Collection:");
  Serial.println(millis());
  getIMU();
  Steering(courseToWaypoint);
  Motor(THRT);

}

void t2Callback() {
  //FONA('s');

}

void t3Callback() {
  WaypointTEST();
  sensors();
}

//*******************************************
//*******************************************
            /*Setup Loop*/
//*******************************************
//*******************************************


void setup()
{

  pinMode(VoltageSense, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(WaterTemp, INPUT);
  pinMode(Salinity, INPUT);

  Serial.begin(115200);             //Serial connection to USB->Computer Serial Monitor
  ss.begin(9600);                   //Serial connection to GPS

  beep(1);  //a beep to announce I am on.

  /*OLED SETUP*/
  OLED.init(0x3C); //Set the I2C addr for the OLED display
  OLED.display();  //show splashscreen
  delay(250);        //wait 1/4 second...we're just showing that screen works and has power.
  OLED.clearDisplay(); //Clear Display
  OLED.setTextSize(1);  //Set text size to smallest
  OLED.setTextColor(WHITE);  //Set text to black background, color text.
  /*END OLED SETUP*/

  Serial.println("Program Begin...");
  delay(100);

  /* Initialise the imu sensors */
  Serial.println("IMU Initialize...");
  delay(100);
  initSensors();

  /* Initialise Waypoint Array */
  Serial.println("WAYPOINT Setup");
  delay(100);
  Waypoint(WPCount);
  Serial.print("WAYPOINT ");
  Serial.print(WPCount);
  Serial.print(" Set as: LAT: ");
  Serial.print(WaypointLAT[WPCount], 8);
  Serial.print(" LONG: ");
  Serial.print(WaypointLONG[WPCount], 8);
  Serial.println();

  /* Initialise Rudder */
  Rudder.attach(35);  // attaches the servo on pin 35 to the servo object
  beep(3);  //3 beeps to register warning
  Rudder.write(pos);  //send rudder to mid position
  Serial.println("Rudder Sweep. KEEP CLEAR!");

  for (pos = 35; pos <= 145; pos += 1) { // goes from 30 degrees to 150 degrees
    // in steps of 1 degree
    Rudder.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 145; pos >= 35; pos -= 1) { // goes from 150 degrees to 30 degrees
    Rudder.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  pos = 90;
  Rudder.write(pos);  //send rudder to mid position

  /* Initialize Throttle*/
  Serial.println("Throttle Setup. KEEP CLEAR!");
  beep(3);  //3 beeps to register warning
  Throttle.attach(36); //attatch throttle to pin 36 to the servo object.

  /*set throttle from neutral to full foward.*/
  for (THRT = 90; THRT <= 180; THRT += 1) {
    // in steps of 1 degree
    Throttle.write(THRT);
    delay(10);
  }
  /*set throttle from neutral to full reverse.*/
  for (pos = 90; THRT >= 0; THRT -= 1) {
    Throttle.write(THRT);
    delay(10);
  }
  THRT = 90;
  Throttle.write(THRT); //send command to set throttle to 0 or int value 90.
  Serial.println("Throttle Set to ZER0 (NEUTRAL)");

  /* Initialize Steering PID*/
  Serial.println("Steering PID Set to AUTOMATIC");
  delay(100);
  PIDRudder.SetMode(AUTOMATIC);

  /* Initialize Fona */
  Serial.println("Starting FONA...");
  fonasetup();

  /*GPS initialize */
  GPSStart();

  /*Let's get moving!*/
  THRT = 180;
  Throttle.write(THRT); //send command to set throttle
  Serial.println("Throttle Set to FULL!");

  beep(3);  //3 beeps to register warning

  /* Initialize Scheduler */
  Serial.println("Task Handler Start Time Set: ");
  runner.startNow();  //set point-in-time for scheduling start.
  Serial.println(millis());

}

//*******************************************
//*******************************************
               /*MAIN LOOP*/
//*******************************************
//*******************************************


void loop()
{

  /*GET GPS*/
  getGPS();
  /*End GPS*/


  WaterTempSample();
  SalinitySample();
  AirTempSample();
  HumiditySample();
  runner.execute();

}




//*******************************************
//*******************************************
   /*floating variable mapping function*/
//*******************************************
//*******************************************

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
