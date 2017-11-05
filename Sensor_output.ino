void sensors() {

  Serial.print("Water Temperature ");
  Serial.print(steinhart);
  Serial.println(" oC");

  Serial.println(" ");
  Serial.print("Air Temperature: ");
  Serial.print(temperature);
  Serial.print(" C, Atm. Humidity: ");
  Serial.print(humidity);
  Serial.print(" %, Dewpoint: ");
  Serial.print(dewpoint);
  Serial.print(" C");

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

  Serial.println("Input Voltage = ");
  Serial.print(Vin);
  Serial.println("");  


  Serial.println("Battery State of Charge: 1: >80%, 2: 40-80%, 3: 20-40%, 4: <20%");
  Serial.print(BatterySOC);
  Serial.println("");
  
}

