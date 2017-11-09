void GPSStart() {


  Serial.println("Starting GPS... ");


  do {

    while (ss.available() > 0)
      gps.encode(ss.read());

    if (gps.satellites.isUpdated())
    {
      //Serial.print(F("SATELLITES Fix Age="));
      //Serial.print(gps.satellites.age());
      //Serial.print(F("ms Value="));
      //Serial.println(gps.satellites.value());
      SatFix = int(gps.satellites.value());
    }

    Serial.println("Current Number of Satelites in View: ");
    Serial.println(SatFix);
    delay(500);
  } while (SatFix < 4);

}

