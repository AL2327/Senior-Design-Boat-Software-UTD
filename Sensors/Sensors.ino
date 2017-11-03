/********************************************************************/
// First we include the libraries
#include <OneWire.h>            //onewire communication protocal for waterproof sensor
#include <DallasTemperature.h>  //library for our waterproof temp sensor that hates us.
#include <Sensirion.h>       //library for our air temp/humidity sensor
#include <EasyTransfer.h>    //allows serial communication between microcontrollers

/********************************************************************/
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2

//Definitions for bilge pump
#define Water_Level_Sensor 5
#define Pump A1

// Definition for voltage sensor input
#define VoltageSense A1

// Definition for salinity sensor input
#define salinity A0

//Pin declarations for Sensirion temp sensor
const uint8_t dataPin  =  4;
const uint8_t clockPin =  3;


//variables for temperature and salinity sensors
float temperature = 0;
float humidity = 0;
float dewpoint = 0;
int SalReading;

//variable for water level sensor
int FloatSwitch = 1;

//variable to assign a "state of charge designator.
int BatterySOC = 1;

//Variables for voltage input calculation
float Vout = 0.0;
float R1 = 100000;
float R2 = 10000;
float Vread = 0;
float Vin = 0.0;
/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
Sensirion tempSensor = Sensirion(dataPin, clockPin);

/********************************************************************/
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
/********************************************************************/

//create serial communication object
EasyTransfer ET;

struct SEND_DATA_STRUCTURE {
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER MICROCONTROLLER

  //variables for temperature and salinity sensors
  float temperature = 0;
  float humidity = 0;
  float dewpoint = 0;
  int SalReading;

  //variables for voltage
  float Vin = 0.0;

  //variable for water level sensor
  int FloatSwitch = 1;

  //variable to assign a "state of charge designator.
  int BatterySOC = 1;
};

//give a name to the group of data
SEND_DATA_STRUCTURE sensorData;



void setup(void)
{

  Serial.begin(9600);

  //start the EasyTransfer_TX library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  ET.begin(details(sensorData), &Serial);


  // Start up the library
  sensors.begin();

  //initialize pins
  pinMode(Water_Level_Sensor, INPUT);
  pinMode(Pump, OUTPUT);
  pinMode(VoltageSense, INPUT);

}



void loop(void)
{

  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus

  /********************************************************************/
  Serial.print(" Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperature readings
  Serial.println("DONE");
  /********************************************************************/

  Serial.print("Water Temperature is: ");
  Serial.print(sensors.getTempCByIndex(0)); // Why "byIndex"?
  // You can have more than one DS18B20 on the same bus.
  // 0 refers to the first IC on the wire


  //Read Sensirion sensor
  tempSensor.measure(&temperature, &humidity, &dewpoint);
  Serial.println(" ");
  Serial.print("Air Temperature: ");
  Serial.print(temperature);
  Serial.print(" C, Atm. Humidity: ");
  Serial.print(humidity);
  Serial.print(" %, Dewpoint: ");
  Serial.print(dewpoint);
  Serial.println(" C");

  //Read salinity sensor
  SalReading = analogRead(salinity);
  float sal = SalReading * (5.0 / 1023.0);
  Serial.print("Salinity: ");
  Serial.print( sal);
  Serial.println(" ppt");

  //read float switch state
  FloatSwitch = digitalRead(Water_Level_Sensor);

  //set pump to on/off according to float switch state
  if (FloatSwitch == HIGH) {
    Serial.println( "WATER LEVEL - LOW");
    digitalWrite(Pump, LOW);
  }
  else {
    Serial.println( "WATER LEVEL - HIGH");
    digitalWrite(Pump, HIGH);
  }

  //Read Voltage input
  Vread = analogRead(VoltageSense);

  //Do some ENA Math and output voltage.
  Vout = (Vread * 5.0) / 1024;
  Vin = Vout / (R2 / (R1 + R2));
  Serial.print("Input Voltage = ");
  Serial.print(Vin);
  Serial.print("");

  //assign a battery state of charge number depending on the Vin.  We can use this number to limit certain functions based on the battery's current remaining capacity.

  if (Vin > 12.50)
  {
    BatterySOC = 1;   //our battery is "fully charged"  >80% State of Charge
  }
  else if (Vin <= 12.49 && Vin > 11.96) {
    BatterySOC = 2; // our battery is in its normal operating range 40-80% Charge
  }
  else if (Vin <= 11.95 && Vin > 11.66) {
    BatterySOC = 3; // our battery is low.  It is between 20-40% charged
  }
  else if (Vin <= 1165) {
    BatterySOC = 4; // our battery is under 20% charged. This is critically low.
  }

  ET.sendData();   //Send all our data over the serial port to teensy


}
