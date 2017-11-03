void Steering(double courseToWaypoint) {

  double courseError;

  Serial.println("VESSEL STATUS: Heading: "); //

  Serial.print(Heading);
  Serial.print(" degrees. Course-to=");
  Serial.print(courseToWaypoint, 6);
  Serial.print(F(" degrees. ["));
  Serial.println("");

  courseError = (Heading - courseToWaypoint);

  Serial.println("Course Error: "); //
  Serial.print(courseError);
  Serial.println("");


  Serial.println("RUDDER COMMAND: "); //
  Serial.println("Commanded Rudder Posistion: ");
  Serial.print(pos);
  Serial.println("");


    /*STEERING CODE*/
  PIDRudder.Compute();          // PID computation   
  pos = map(pidoutput, 0, 255, 0, 180);  //map PID output(double) to pos(integer) range
  pos = constrain(pos, 60, 120);        //contrain the rudder to +/- 45 deg. 
  Rudder.write(pos);              // tell servo to go to position in variable 'pos'  
  /*END STEERING CODE*/
}

