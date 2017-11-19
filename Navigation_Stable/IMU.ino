void getIMU()
{
  /**************************************************************************/
  /* IMU SECTION */
  /**************************************************************************/


 /* /* Calculate pitch and roll from the raw accelerometer data 
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields 
    Serial.print(F("Roll: "));
    Serial.print(orientation.roll);
    Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    Serial.print(orientation.pitch);
    Serial.print(F("; "));
    Serial.println();
  }

*/
  /*GET HEADING*/
  mag.getEvent(&mag_event);
  accel.getEvent(&accel_event);
  if (dof.magTiltCompensation(SENSOR_AXIS_Z, &mag_event, &accel_event)) {


    if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
    {
      /* 'orientation' should have valid .heading data now */

      Heading = orientation.heading;
      Heading = Heading - 105;
      //Serial.print(F("Heading: "));
      //Serial.print(Heading);
      //Serial.print(F("; "));
      //Serial.println();
    }
  }
  /*End Heading*/

/*
  Serial.println(F(""));

  Serial.print("WAYPOINT ");
  Serial.print(WPCount);
  Serial.print(" Set as: LAT: ");
  Serial.print(WaypointLAT[WPCount], 6);
  Serial.print(" LONG: ");
  Serial.print(WaypointLONG[WPCount], 6);
  Serial.println();

  Serial.print(F("Waypoint Distance="));
  Serial.print(distanceToWaypoint / 1000, 6);
  Serial.print(F(" km Course-to="));
  Serial.print(courseToWaypoint, 6);
  Serial.print(F(" degrees. "));
  Serial.println(F(""));
  Serial.print(F("Waypoint Distance="));
  Serial.print(distanceToWaypoint, 6);
  Serial.print(F(" m."));
  Serial.println(F(""));

  */

}

