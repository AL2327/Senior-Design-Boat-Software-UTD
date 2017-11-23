void Command() {

  FONA('r');
  delay(50);
  FONA('1');

  if (ValidMessage = false) {
    return;
  }

  ComRcv = String(IncCommand[254]);
  ComRcv = ComRcv.toUpperCase();

  if (ComRcv.startsWith("STATUS") ) {
    FONA('s');
  }

  else if (ComRcv.startsWith("CHANGE WPC")) {
    WPCount = ComRcv.charAt(12);
    FONA('w');
  }

  else if (ComRcv.startsWith("HALT")) {
    FONA('h');

    do {
      delay(30000);
      FONA('r');
      delay(50);
      FONA('1');
    } while (ComRcv.startsWith("HALT"));
  }

  else if (ComRcv.startsWith("RESUME")) {
    FONA('R');
  }

  else if (ComRcv.startsWith("DEMO")) {
    FONA('D');
  }

  /*Changing the Latitude and Longitude with different waypoint #*/
  else if (ComRcv.startsWith("CHANGE LL")) {
    FONA('l');
  }

  /*Changing the craft to follow a heading vs to a waypoint*/
  else if (ComRcv.startsWith("HEADING MODE ON"))
  {
    FONA('H');
  }

  /*Changing the craft to follow a waypoint vs folling a heading*/
  else if (ComRcv.startsWith("HEADING MODE OFF"))
  {
    FONA('m');
  }

  /*Requesting a list of waypoints */
    else if (ComRcv.startsWith("PRINT WP"))
  {
    FONA('p');
  }


  //delete sms
  FONA('d');
  delay(50);
  FONA('1');
}

