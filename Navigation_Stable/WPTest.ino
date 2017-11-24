void WaypointTEST() {

  Serial.println("Testing distance to waypoint... ");

  if (WaypointLAT[WPCount] == 0 and WaypointLONG[WPCount] == 0 ) {
    WPCount = 0;
    Serial.println("Final Waypoint reached. Reseting to Waypoint 0. ");
    Serial.print(WPCount);
  }

  if (distanceToWaypoint <= 7 and distanceToWaypoint>=1) {
    WPCount++;
    Serial.println("WAYPOINT REACHED! COUNTER INCREMENTED TO: ");
    Serial.println(WPCount);
   }
   else {
        Serial.println("Waypoint not yet reached. ");
   }
}

