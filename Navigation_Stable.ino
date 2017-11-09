#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <TaskScheduler.h>
#include <Servo.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>
#include <EasyTransfer.h>


#define _TASK_SLEEP_ON_IDLE_RUN  //tells scheduler to put processor to sleep if doing nothing.
#define _TASK_TIMECRITICAL //allows designation of a time critical task(s) I haven't used this yet or know how -Aaron. 

//Fona definitions
#define FONA_RX 10
#define FONA_TX 9
#define FONA_RST 20
#define HWSERIAL Serial2


/* Assign a unique ID to the IMU sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_vec_t   orientation;


/*LET US DEFINE SOME VARIABLES*/
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA; /* Update this with the correct SLP for accurate altitude measurements */
unsigned long last = 0UL; // For stats that happen every 5 seconds

/*STEERING PID VARIABLES*/
double pos = 90;    // variable to store the servo position. Set it initially to center (neutral posistion)
double pidoutput = 0; //variable for PID Rudder control output.
double Kp = 50;
double Ki = 0;
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
int THRT = 0; //variable to stor commanded throttle posistion.

/*Fona Variables*/
char replybuffer[255];  // this is a large buffer for replies
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;

/*MCU to MCU Easy transfer variables AKA: Sensor Variables*/
//variables for temperature and salinity sensors
float temperature = 29 ;
float humidity = 54;
float dewpoint = 20 ;
float steinhart = 20;
int SalReading = 0;

//variables for voltage
float Vin = 12.4;

//variable for water level sensor
int FloatSwitch = 0;

//variable to assign a "state of charge designator.
int BatterySOC = 2;

/******************************/

Adafruit_FONA fona = Adafruit_FONA(FONA_RST); //passing value of Fona_RST to fona

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
#define ss Serial1

//Servo Object for Steering
Servo Rudder;

//Servo Object for Throttle
Servo Throttle;

// Define PID controller for rudder
PID PIDRudder(&Heading, &pidoutput, &courseToWaypoint, Kp, Ki, Kd, DIRECT );


//**********EASY TRANFER SET UP**********
//create object for mcu to mcu serial communication
EasyTransfer ET;

struct RECEIVE_DATA_STRUCTURE {
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER MICROCONTROLLER

  //variables for temperature and salinity sensors
  float temperature;
  float humidity;
  float dewpoint;
  float steinhart;
  int SalReading;

  //variables for voltage
  float Vin;

  //variable for water level sensor
  int FloatSwitch;

  //variable to assign a "state of charge designator.
  int BatterySOC;
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE sensorData;

//***************************************

Scheduler runner;
// Callback methods prototypes
void t1Callback();
void t2Callback();

// Tasks
Task t1(1000, TASK_FOREVER, &t1Callback, &runner, true);  //adding task to do periodic tasks
Task t2(60000, TASK_FOREVER, &t2Callback, &runner, true);  //adding task do fona


void t1Callback() {
  Serial.print("IMU Collection:");
  Serial.println(millis());
  getIMU();
  Steering(courseToWaypoint);
  //  Serial.print("Throttle Setting:");
  //  Serial.println(millis());
  //  Motor(THRT);
  //  sensors();
}

void t2Callback() {
  //FONA('s');

}

void setup()
{

  Serial.begin(115200);             //Serial connection to USB->Computer Serial Monitor
  ss.begin(9600);                   //Serial connection to GPS
  Serial3.begin(9600);              //Serial communication from arduino reading sensors


  //start the EasyTransfer_TX library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  ET.begin(details(sensorData), &Serial3);

  Serial.println("Program Begin...");
  delay(500);

  /* Initialise the imu sensors */
  Serial.println("IMU Initialise...");
  delay(500);
  initSensors();

  /* Initialise Waypoint Array */
  Serial.println("WAYPOINT Setup");
  delay(500);
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

  Rudder.write(pos);  //send rudder to mid position

  Serial.println("Rudder Amidships.  3 Second Hold.");
  delay(3000);      // wait for 3 seconds after rudder moves.

  Serial.println("Rudder Sweep in 3 seconds. KEEP CLEAR!");
  delay(3000);

  for (pos = 30; pos <= 150; pos += 1) { // goes from 30 degrees to 150 degrees
    // in steps of 1 degree
    Rudder.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 150; pos >= 30; pos -= 1) { // goes from 150 degrees to 30 degrees
    Rudder.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  pos = 90;
  Rudder.write(pos);  //send rudder to mid position
  delay(1000);


  /* Initialize Throttle*/
  Serial.println("Throttle Setup");
  delay(500);
  Throttle.attach(36); //attatch throttle to pin 36 to the servo object.
  Throttle.write(THRT); //send command to set throttle to 0.
  Serial.println("Throttle Set to ZERO");

  /* Initialize Steering PID*/
  Serial.println("Steering PID Set to AUTOMATIC");
  delay(500);
  PIDRudder.SetMode(AUTOMATIC);

  /* Initialize Fona */
  Serial.println("Starting FONA...");
  fonasetup();

  /*GPS initialize */
  Serial.println("Starting GPS... ");

    do {
    Serial.println("Current Number of Satelites in View: ");
    Serial.println(SatFix);
    delay(500);
    } while (SatFix <4);

  /* Initialize Scheduler */
  Serial.println("Task Handler Start Time Set: ");
  runner.startNow();  //set point-in-time for scheduling start.
  Serial.println(millis());

}

void loop()
{

  /*GET GPS*/
  getGPS();
  /*End GPS*/



  /*GET DATA FROM SENSORS*/
  if (ET.receiveData()) {
    //this is how you access the variables. [name of the group].[variable name]
    //variables for temperature and salinity sensors
    temperature = sensorData.temperature;
    humidity = sensorData.humidity;
    dewpoint = sensorData.dewpoint;
    steinhart = sensorData.steinhart;
    SalReading = sensorData.SalReading;
    Vin = sensorData.Vin;
    FloatSwitch = sensorData.FloatSwitch;
    BatterySOC = BatterySOC;
  }




  runner.execute();
    WaypointTEST();
}

void WaypointTEST(){
  
  if (WaypointLAT[WPCount] == 0 and WaypointLONG[WPCount] == 0 ) {
    WPCount = 0;
  }

  if (distanceToWaypoint <= 0.01) {
    WPCount++;
    Serial.println("WAYPOINT COUNTER: ");
    Serial.print(WPCount);
  }
}

