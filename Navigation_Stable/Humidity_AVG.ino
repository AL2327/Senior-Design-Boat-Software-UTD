void HumiditySample(){

  // subtract the last reading:
  Htotal = Htotal - HTempSample[HreadIndex];
  // read from the sensor:
  HTempSample[HreadIndex] = analogRead(HumidityPin);
  // add the reading to the total:
  Htotal = Htotal + HTempSample[HreadIndex];
  // advance to the next position in the array:
  HreadIndex = HreadIndex + 1;
  // if we're at the end of the array...
  if (HreadIndex >= HnumReadings) {
    // ...wrap around to the beginning:
    HreadIndex = 0;
  }
  // calculate the average:
  HTemp = Htotal / HnumReadings;

  //Serial.println("Humidity AVG: ");
  //Serial.print(=ATemp);
}
