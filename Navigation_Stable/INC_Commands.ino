void Command() {

  FONA('r');
  delay(50);
  FONA('1');

  if (ValidMessage = false) {
    return;
  }

  ComRcv = String(IncCommand[255]);
  ComRcv.toUpperCase();

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
    delay(1000);
    beep(3);

    pos = 90;
    Rudder.write(pos);  //send rudder to mid position
    for (pos = 35; pos <= 145; pos += 1) { // goes from 35 degrees to 145 degrees
      // in steps of 1 degree
      Rudder.write(pos);              // tell servo to go to position in variable 'pos'
      delay(10);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 145; pos >= 35; pos -= 1) { // goes from 145 degrees to 35 degrees
      Rudder.write(pos);              // tell servo to go to position in variable 'pos'
      delay(10);                       // waits 15ms for the servo to reach the position
    }
    pos = 90;
    Rudder.write(pos);  //send rudder to mid position
    
    beep(3);
    /*set throttle to "wiggle"*/
    for (THRT = 90; THRT <= 100; THRT += 1) {
      // in steps of 1 degree
      Throttle.write(THRT);
      delay(10);
    }
    /*set throttle from neutral to full reverse.*/
    for (pos = 90; THRT >= 80; THRT -= 1) {
      Throttle.write(THRT);
      delay(10);
    }
    THRT = 90;
    Throttle.write(THRT);
    beep(1);
    FONA('s');
  }


}

