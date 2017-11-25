void Command() {

  FONA('r');            //tell fona to check for new message
  delay(50);            //pause 50 milli seconds for it to receive
  //Serial2.write(1);     //tell fona to check messaage 1

  if (ValidMessage = false) {
    return;                         //if no message was received, leave function and continue operation
  }

  //at this point, we know we've received a message.
  ComRcv = String(IncCommand[254]);         //copy our incomming command character array to string.

  Serial.println("Command Received: ");
  Serial.print(ComRcv);

  ComRcv = ComRcv.toUpperCase();            //change our incoming command to all caps to ease string comparison.

  Serial.println("Command converted to ALL CAPS: ");
  Serial.print(ComRcv);

  if (ComRcv.startsWith("STATUS") ) {
    Serial.println("Command Received: STATUS REQUEST");
    FONA('s');
  }

  else if (ComRcv.startsWith("CHANGE WPC")) {
    Serial.println("Command Received: CHANGE WAY POINT COUNTER");
    WPCount = ComRcv.charAt(12);
    FONA('w');
  }

  else if (ComRcv.startsWith("HALT")) {
    Serial.println("Command Received: HALT, send any other command to resume.");
    FONA('h');

    do {
      delay(10000);
      FONA('r');
      delay(50);
      FONA('1');
      THRT = 90; //STOP all motor movement.  We are waiting.
      Throttle.write(THRT); //send command to set throttle
      pos = 90; //set steering to dead ahead.
      Rudder.write(pos);  //send command to steering
    } while (ComRcv.startsWith("HALT"));
  }

  else if (ComRcv.startsWith("RESUME")) {
    Serial.println("Command Received: RESUME");
    FONA('R');
  }

  else if (ComRcv.startsWith("DEMO")) {
    Serial.println("Command Received: DEMONSTRATION");
    FONA('D');
  }

  /*Changing the Latitude and Longitude with different waypoint #*/
  else if (ComRcv.startsWith("CHANGE LL")) {
    Serial.println("Command Received: UPDATE A WAYPOINT'S LAT/LONG");
    FONA('l');
  }

  /*Changing the craft to follow a heading vs to a waypoint*/
  else if (ComRcv.startsWith("HEADING MODE ON"))
  {
    Serial.println("Command Received: HEADING MODE ON");
    FONA('H');
  }

  /*Changing the craft to follow a waypoint vs folling a heading*/
  else if (ComRcv.startsWith("HEADING MODE OFF"))
  {
    Serial.println("Command Received: HEADING MODE OFF");
    FONA('m');
  }

  /*Requesting a list of waypoints */
  else if (ComRcv.startsWith("PRINT WP"))
  {
    Serial.println("Command Received: LIST WAYPOINTS");
    FONA('p');
  }


  //delete sms
  FONA('d');
  delay(50);
  FONA('1');
}

