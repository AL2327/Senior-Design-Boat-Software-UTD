void VoltageSample(){
  

  // subtract the last reading:
  Vtotal = Vtotal - VTempSample[WreadIndex];
  // read from the sensor:
  VTempSample[VreadIndex] = analogRead(VoltageSense);
  // add the reading to the total:
  Vtotal = Vtotal + VTempSample[VreadIndex];
  // advance to the next position in the array:
  VreadIndex = VreadIndex + 1;
  // if we're at the end of the array...
  if (VreadIndex >= VnumReadings) {
    // ...wrap around to the beginning:
    VreadIndex = 0;
  }
  // calculate the average:
  VTemp = Vtotal / VnumReadings;

  //Serial.println("Voltage input AVG: ");
  //Serial.print(VTemp);

}
