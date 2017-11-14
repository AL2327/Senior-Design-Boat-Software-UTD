void SalinitySample(){

  // subtract the last reading:
  Stotal = Stotal - STempSample[SreadIndex];
  // read from the sensor:
  STempSample[SreadIndex] = analogRead(Salinity);
  // add the reading to the total:
  Stotal = Stotal + STempSample[SreadIndex];
  // advance to the next position in the array:
  SreadIndex = SreadIndex + 1;
  // if we're at the end of the array...
  if (SreadIndex >= SnumReadings) {
    // ...wrap around to the beginning:
    SreadIndex = 0;
  }
  // calculate the average:
  STemp = Stotal / SnumReadings;

  //Serial.println("Salinity AVG: ");
  //Serial.print(STemp);
}
