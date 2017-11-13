void SensorAVG(){
  

  //water temperature sample and averaging. 
  
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

  //**********************************

  //salinity sensor sample and averaging
  
  // subtract the last reading:
  Stotal = Stotal - SaltSample[SreadIndex];
  // read from the sensor:
  SaltSample[SreadIndex] = analogRead(Salinity);
  // add the reading to the total:
  Stotal = Stotal + SaltSample[SreadIndex];
  // advance to the next position in the array:
  SreadIndex = SreadIndex + 1;
  // if we're at the end of the array...
  if (SreadIndex >= SnumReadings) {
    // ...wrap around to the beginning:
    SreadIndex = 0;
  }
  // calculate the average:
  SalInput = Stotal / SnumReadings;
  

}
