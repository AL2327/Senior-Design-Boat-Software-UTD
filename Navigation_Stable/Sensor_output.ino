void sensors() {




  //WTemp=mapfloat(WTemp, 0,1023, 0,177);
  Serial.print("Water Temperature Input ");
  Serial.print(WTemp, 4);

  steinhart = mapfloat(WTemp, 0, 1023, -55, 125);

  Serial.print("Water Temperature ");
  Serial.print(steinhart);
  Serial.println(" oC");

  //end water temp

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


  //salinity parts per thousand
  float SalInput;
  SalInput = analogRead(Salinity);
  SalInput = mapfloat(SalInput, 0, 1023, 0, 177);
  Serial.println("Salinity  PWM Input ");
  Serial.print(SalInput);


  SalReading = mapfloat(SalInput, 0, 177, 0, 177);

  Serial.println(" ");
  Serial.println("Salinity: ");
  Serial.print(SalReading);
  Serial.println(" ppt");

  if (FloatSwitch == HIGH) {
    Serial.println( "WATER LEVEL - LOW");
  }
  else {
    Serial.println( "WATER LEVEL - HIGH");
  }


  //Read Voltage input
  Vread = analogRead(VoltageSense);

  //Do some ENA Math and output voltage.
  Vout = (Vread * 5.0) / 1024;
  Vin = Vout / (R2 / (R1 + R2));
  Serial.println("Input Voltage = ");
  Serial.print(Vin);
  Serial.println("");

  //end voltage

  //Battery State of Charge Declaration

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

  Serial.println("Battery State of Charge: 1: >80%, 2: 40-80%, 3: 20-40%, 4: <20%");
  Serial.print(BatterySOC);
  Serial.println("");

  //end battery SOC

}

