void AirTempSample(){

  // subtract the last reading:
  Atotal = Atotal - ATempSample[AreadIndex];
  // read from the sensor:
  ATempSample[AreadIndex] = analogRead(AirTempPin);
  // add the reading to the total:
  Atotal = Atotal + ATempSample[AreadIndex];
  // advance to the next position in the array:
  AreadIndex = AreadIndex + 1;
  // if we're at the end of the array...
  if (AreadIndex >= AnumReadings) {
    // ...wrap around to the beginning:
    AreadIndex = 0;
  }
  // calculate the average:
  ATemp = Atotal / AnumReadings;

  //Serial.println("Air Temp AVG: ");
  //Serial.print(=ATemp);
}
