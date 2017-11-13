void WaterTempSample(){
  

  // subtract the last reading:
  total = total - WTempSample[readIndex];
  // read from the sensor:
  WTempSample[readIndex] = analogRead(WaterTemp);
  // add the reading to the total:
  total = total + WTempSample[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;
  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }
  // calculate the average:
  WTemp = total / numReadings;

  //Serial.print("Water Temperature AVG ");
  //Serial.print(WTemp, 4);

}
