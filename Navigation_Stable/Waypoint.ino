void Waypoint(int WPCount) {

  /*Setting some test gps waypoints*/
  Serial.println("WAYPOINT COUNTER: ");
  Serial.print(WPCount);
    
  WaypointLAT[0] = 32.993433;
  WaypointLONG[0] = -96.751834;

  WaypointLAT[1] = 32.993342 ;
  WaypointLONG[1] = -96.751828;

  WaypointLAT[2] = 32.994345;
  WaypointLONG[2] = -96.751598;

  WaypointLAT[3] = 32.993421;
  WaypointLONG[3] = -96.751600;

  WaypointLAT[4] = 0;
  WaypointLONG[4] = 0;  

  WaypointLAT[5] = 0;
  WaypointLONG[5] = 0; 
  
  WaypointLAT[6] = 0;
  WaypointLONG[6] = 0; 

  WaypointLAT[7] = 0;
  WaypointLONG[7] = 0; 

  WaypointLAT[8] = 0;
  WaypointLONG[8] = 0; 
}

