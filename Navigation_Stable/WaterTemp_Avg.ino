void WaterTempSample(){
  

  // subtract the last reading:
  Wtotal = Wtotal - WTempSample[WreadIndex];
  // read from the sensor:
  WTempSample[WreadIndex] = analogRead(WaterTemp);
  // add the reading to the total:
  Wtotal = Wtotal + WTempSample[WreadIndex];
  // advance to the next position in the array:
  WreadIndex = WreadIndex + 1;
  // if we're at the end of the array...
  if (WreadIndex >= WnumReadings) {
    // ...wrap around to the beginning:
    WreadIndex = 0;
  }
  // calculate the average:
  WTemp = Wtotal / WnumReadings;

  //Serial.println("Water Temperature AVG: ");
  //Serial.print(WTemp);

}
