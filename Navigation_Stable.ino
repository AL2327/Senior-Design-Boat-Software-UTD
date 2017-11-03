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


/*LET US DEFINE SOME VARIABLES*/
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA; /* Update this with the correct SLP for accurate altitude measurements */
unsigned long last = 0UL; // For stats that happen every 5 seconds

/*STEERING PID VARIABLES*/
double pos = 90;    // variable to store the servo position. Set it initially to center (neutral posistion)
double pidoutput = 0; //variable for PID Rudder control output.
double Kp = 2;
double Ki = 0;
double Kd = 0;


/*GPS VARIABLES*/
double WaypointLAT[9];
double WaypointLONG[9];
double distanceToWaypoint;  //distance to way point in kilometers
double courseToWaypoint;  //course to way point in degrees
//End GPS variables.

/*IMU Variables */
double Heading = 0; //Current vessel heading in degress.

/*THROTTLE VARIABLES*/
int THRT = 180; //variable to stor commanded throttle posistion.

//Fona Variables
char replybuffer[255];  // this is a large buffer for replies
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;

/******************************/

// The TinyGPS++ object 
TinyGPSPlus gps;

//Servo Object for Steering
Servo Rudder;
//Servo Object for Throttle
Servo Throttle;

// Define PID controller for rudder
PID PIDRudder(&Heading, &pidoutput, &courseToWaypoint, Kp, Ki, Kd, DIRECT );

// The serial connection to the GPS device
#define ss Serial1

Adafruit_FONA fona = Adafruit_FONA(FONA_RST); //passing value of Fona_RST to fona

Scheduler runner;
// Callback methods prototypes
void t1Callback();
void t2Callback();
void t3Callback();

// Tasks
Task t1(1000, TASK_FOREVER, &t1Callback, &runner, true);  //adding task to check IMU
Task t2(500, TASK_FOREVER, &t2Callback, &runner, true);  //adding task to steer vessel and set throttle
Task t3(5000, TASK_FOREVER, &t3Callback, &runner, true); //adding task to talk to fona

void t1Callback() {
  Serial.print("IMU/Steering Collection:");
  Serial.println(millis());
  getIMU();
  Steering(courseToWaypoint);
}

void t2Callback() {
  Steering(courseToWaypoint);
  Motor(THRT);
}

void t3Callback() {
  FONA();
}

void setup()
{

  Serial.begin(115200);
  ss.begin(9600);

  /* Initialise the imu sensors */
  initSensors();

  Serial.println("Program Begin...");//
  runner.startNow();  //set point-in-time for scheduling start.

  Waypoint(); //Call waypoint setup.

  Rudder.attach(35);  // attaches the servo on pin 35 to the servo object

  Rudder.write(pos);  //send rudder to mid position
  delay(3000);      // wait for 3 seconds after rudder moves.

  Throttle.attach(36); //attatch throttle to pin 36 to the servo object.
  Throttle.write(THRT); //send command to set throttle to 0.

  PIDRudder.SetMode(AUTOMATIC);

  fonasetup();

}

void loop()
{
  getGPS();
    
  /*STEERING CODE*/
  //  PIDRudder.Compute();          // PID computation
  //  pos = map(pidoutput, 0, 255, 0, 180);  //map PID output(double) to pos(integer) range
  //  pos = constrain(pos, 60, 120);        //contrain the rudder to +/- 45 deg.
  //  Rudder.write(pos);              // tell servo to go to position in variable 'pos'
  /*END STEERING CODE*/

  runner.execute();
}
