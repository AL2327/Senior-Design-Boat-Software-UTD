/********************************************************************/
// First we include the libraries
#include <Sensirion.h>       //library for our air temp/humidity sensor
/********************************************************************/




//Definitions for bilge pump
#define Water_Level_Sensor 7
#define Pump 13


// Definition for salinity sensor input
#define salinity 14


//Pin declarations for Sensirion temp sensor
const uint8_t dataPin  =  4;
const uint8_t clockPin =  3;



//Variables for NTC Thermristor
byte NTCPin = A2;
#define SERIESRESISTOR 10000
#define NOMINAL_RESISTANCE 10000
#define NOMINAL_TEMPERATURE 25
#define BCOEFFICIENT 3950
float ADCvalue;
float Resistance;
float steinhart;

//variables for temperature and salinity sensors
float temperature;
float humidity;
float dewpoint;
uint16_t SalReading;

//variable for water level sensor
uint16_t FloatSwitch;


/********************************************************************/
Sensirion tempSensor = Sensirion(dataPin, clockPin);

/********************************************************************/


void setup(void)
{

  Serial.begin(9600);

  //initialize pins
  pinMode(Water_Level_Sensor, INPUT);
  pinMode(Pump, OUTPUT);
  //  pinMode(VoltageSense, INPUT);


}



void loop(void)
{


  //Read and calculate analog NTC thermisitor
  ADCvalue = analogRead(NTCPin);
  Serial.print("Analog value ");
  Serial.print(ADCvalue);
  Serial.print(" = ");
  //convert this ADC value into resistance
  Resistance = (1023 / ADCvalue) - 1;
  Resistance = SERIESRESISTOR / Resistance;
  Serial.print(Resistance);
  Serial.println(" Ohm");
  //calculate temperature using steinhart formula
  steinhart = Resistance / NOMINAL_RESISTANCE; // (R/Ro)
  steinhart = log(steinhart); // ln(R/Ro)
  steinhart /= BCOEFFICIENT; // 1/B * ln(R/Ro)
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart; // Invert
  steinhart -= 273.15; // convert to C

  Serial.print("Water Temperature ");
  Serial.print(steinhart);
  Serial.println(" oC");

  //math for output
  float WTemp;
  WTemp = constrain(WTemp, 0, 177);
  WTemp = mapfloat(steinhart, -55, 125, 0, 177);

  Serial.print("Water Temperature: Analog Output: ");
  Serial.print(WTemp);
  Serial.println(" PWM Value");
  analogWrite(10, WTemp);
  //end thermristor calculations and output


  //Read Sensirion sensor
  tempSensor.measure(&temperature, &humidity, &dewpoint);
  Serial.println(" ");
  Serial.print("Air Temperature: ");
  Serial.print(temperature);
  Serial.print(" C, Atm. Humidity: ");
  Serial.print(humidity);
  Serial.print(" %, Dewpoint: ");
  Serial.print(dewpoint);
  Serial.print(" C");

  int ATemp;
  ATemp = constrain(ATemp, 0, 177);
  ATemp = mapfloat(temperature, -40, 120, 0, 177);

  Serial.print("Air Temperature: Analog Output: ");
  Serial.print(ATemp);
  Serial.println(" PWM Value");
  analogWrite(5, ATemp);

  int Humid;
  Humid = constrain(Humid, 0, 177);
  Humid = mapfloat(humidity, 0, 100, 0, 177);

  Serial.print("Humidity: Analog Output: ");
  Serial.print(Humid);
  Serial.println(" PWM Value");
  analogWrite(6, Humid);

  //end sht10 output


  //Read salinity sensor
  float sal = analogRead(salinity);
  SalReading = sal * (5.0 / 1023.0);
  Serial.println(" ");
  Serial.println("Salinity: ");
  Serial.print(SalReading);
  Serial.println(" ppt");

  //math for salinity analog output
  int Salinity;
  Salinity = constrain(Salinity, 0, 177);
  Salinity = mapfloat(SalReading, 0, 177, 0, 177);

  Serial.print("Salinity: Analog Output: ");
  Serial.print(Salinity);
  Serial.println(" PWM Value");
  analogWrite(9, Salinity);

  //end salinity


  //read float switch state
  FloatSwitch = digitalRead(Water_Level_Sensor);

  //set pump to on/off according to float switch state
  if (FloatSwitch == HIGH) {
    Serial.println( "WATER LEVEL - LOW");
    digitalWrite(Pump, LOW);
    digitalWrite(12, LOW);
  }
  else {
    Serial.println( "WATER LEVEL - HIGH");
    digitalWrite(Pump, HIGH);
    digitalWrite(12, HIGH);
  }

  delay(2500);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
