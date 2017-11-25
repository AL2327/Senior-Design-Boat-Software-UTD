void FONA(char command) {
  /*Serial.print(F("FONA> "));
    while (! Serial.available() ) {
    if (fona.available()) {
    Serial.write(fona.read());
    }
    }*/

  //char command = Serial.read()
  //char command = 's';
  Serial.println(command);


  switch (command) {
    case '?': {
        //        printMenu();
        break;
      }

    case 'a': {
        // read the ADC
        uint16_t adc;
        if (! fona.getADCVoltage(&adc)) {
          Serial.println(F("Failed to read ADC"));
        } else {
          Serial.print(F("ADC = ")); Serial.print(adc); Serial.println(F(" mV"));
        }
        break;
      }

    case 'b': {
        // read the battery voltage and percentage
        uint16_t vbat;
        if (! fona.getBattVoltage(&vbat)) {
          Serial.println(F("Failed to read Batt"));
        } else {
          Serial.print(F("VBat = ")); Serial.print(vbat); Serial.println(F(" mV"));
        }


        if ( fona.getBattPercent(&vbat)) {
          Serial.println(F("Failed to read Batt"));
        } else {
          Serial.print(F("VPct = ")); Serial.print(vbat); Serial.println(F("%"));
        }

        break;
      }

    case 'U': {
        // Unlock the SIM with a PIN code
        char PIN[5];
        flushSerial();
        Serial.println(F("Enter 4-digit PIN"));
        readline(PIN, 3);
        Serial.println(PIN);
        Serial.print(F("Unlocking SIM card: "));
        if (! fona.unlockSIM(PIN)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        break;
      }

    case 'C': {
        // read the CCID
        fona.getSIMCCID(replybuffer);  // make sure replybuffer is at least 21 bytes!
        Serial.print(F("SIM CCID = ")); Serial.println(replybuffer);
        break;
      }

    case 'i': {
        // read the RSSI
        uint8_t n = fona.getRSSI();
        int8_t r;

        Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(": ");
        if (n == 0) r = -115;
        if (n == 1) r = -111;
        if (n == 31) r = -52;
        if ((n >= 2) && (n <= 30)) {
          r = map(n, 2, 30, -110, -54);
        }
        Serial.print(r); Serial.println(F(" dBm"));

        break;
      }

    case 'n': {
        // read the network/cellular status
        uint8_t n = fona.getNetworkStatus();
        Serial.print(F("Network status "));
        Serial.print(n);
        Serial.print(F(": "));
        if (n == 0) Serial.println(F("Not registered"));
        if (n == 1) Serial.println(F("Registered (home)"));
        if (n == 2) Serial.println(F("Not registered (searching)"));
        if (n == 3) Serial.println(F("Denied"));
        if (n == 4) Serial.println(F("Unknown"));
        if (n == 5) Serial.println(F("Registered roaming"));
        break;
      }




    case 'H': {

        //HEADING MODE ON
        HeadingMode = true;

        String temp = ComRcv.remove(0, 16); //this should trim off "HEADING_MODE_ON_" leaving us with just the heading value

        long newcourseToWaypoint = temp.toInt();
        HeadingModeCourse = int(newcourseToWaypoint);

        String msg1 = String(newcourseToWaypoint);
        String msg = "Overriding Waypoint Navigation to hold heading: " + msg1 ;

        char sendto[21], message[300];
        ltoa(ReplytoPhoneNumber, sendto, 10) ;
        for (int i = 0; i < msg.length() ; i++)
        {
          msg.toCharArray(message, 300);

        }

        flushSerial();
        if (! fona.sendSMS(sendto, message)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }

        break;
      }


    case 'm': {
        // Set Heading mode OFF

        HeadingMode = false;

        String msg1 = String(WPCount);
        String msg = "Navigation Mode Enabled, proceeding to Waypoint :" + msg1 ;

        char sendto[21], message[300];
        ltoa(ReplytoPhoneNumber, sendto, 10) ;
        for (int i = 0; i < msg.length() ; i++)
        {
          msg.toCharArray(message, 300);

        }

        flushSerial();
        if (! fona.sendSMS(sendto, message)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }
        break;
      }

    /*** SMS ***/

    case 'N': {
        // read the number of SMS's!
        int8_t smsnum = fona.getNumSMS();
        if (smsnum < 0) {
          Serial.println(F("Could not read # SMS"));
        } else {
          Serial.print(smsnum);
          Serial.println(F(" SMS's on SIM card!"));
        }
        break;
      }

    case 'r': {
        // read an SMS
        flushSerial();
        Serial.print(F("Read #"));
        //uint8_t smsn = readnumber();
        int smsnum = 1;
        Serial2.write(smsnum);     //tell fona to check messaage 1
        Serial.print(F("\n\rReading SMS #")); Serial.println(smsnum);

        // Retrieve SMS sender address/phone number.
        if (! fona.getSMSSender(smsnum, replybuffer, 250)) {
          Serial.println("Failed!");
          ValidMessage = false;
          break;
        }
        Serial.print(F("FROM: ")); Serial.println(replybuffer);

        ReplytoPhoneNumber = int(replybuffer);      //save phone number of message sender

        // Retrieve SMS value.
        uint16_t smslen;
        if (! fona.readSMS(smsnum, replybuffer, 250, &smslen)) { // pass in buffer and max len!
          Serial.println("Failed!");
          ValidMessage = false;
          break;
        }
        Serial.print(F("***** SMS #")); Serial.print(smsnum);
        Serial.print(" ("); Serial.print(smslen); Serial.println(F(") bytes *****"));
        Serial.println(replybuffer);
        Serial.println(F("*****"));

        IncCommand[254] = replybuffer[254];           //this is our received command
        ValidMessage = true;

        break;
      }

    case 'd': {
        // delete an SMS
        flushSerial();
        Serial.print(F("Delete #"));
        uint8_t smsn = readnumber();

        Serial.print(F("\n\rDeleting SMS #")); Serial.println(smsn);
        if (fona.deleteSMS(smsn)) {
          Serial.println(F("OK!"));
          SMSCheck = false;
        } else {
          Serial.println(F("Couldn't delete"));
          SMSCheck = true;
        }
        break;
      }

    case 's': {
        // send an SMS!


        String msg1 = String(Heading);
        String msg2 = String(courseToWaypoint, 6);
        String msg3 = String(distanceToWaypoint / 1000, 6);
        String msg4 = String(WaypointLAT[WPCount], 6);
        String msg5 = String(WaypointLONG[WPCount], 6);
        String msg6 = String(WPCount, 3);
        String msg7 = String(ATemp, 5);
        String msg8 = String(HTemp, 2);
        String msg9 = String(dewpoint, 4);
        String msg10 = String(steinhart, 6);
        String msg11 = String(STemp, 6);
        String msg12 = String(Vin);
        String msg13 = String(BatterySOC);


        String msg = "WP#: " + msg6 + " \n " + "H: " + msg1 + " \n " + "C2WP: " + msg2 + "\n " + "D2WP: " + msg3 + " \n " + "Lat: " + msg4 + "\n " + "Long: " + msg5 + "\n " + "AT: " + msg7 + "\n " + "AH: "
                     + msg8 + " \n " + "DP: " + msg9 + " \n " + "WT: " + msg10 + " \n " + "SAL: " + msg11 + " \n " + "V: " + msg12 + " \n " + "SoC: " + msg13;

        char sendto[21] = "4698883192", message[300] ;
        for (int i = 0; i < msg.length() ; i++)
        {
          msg.toCharArray(message, 300);

        }

        Serial.println("!!!!!!!MESSAGE IS!!!!!!");
        Serial.println(message);
        flushSerial();
        /*Serial.print(F("Send to #"));
          readline(sendto, 20);
          Serial.println(sendto);
          Serial.print(F("Type out one-line message (140 char): "));
          readline(message, 140);
          Serial.println(message);*/
        if (! fona.sendSMS(sendto, message)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }

        break;
      }

    case 'w': {
        // send an SMS reply to acknowledge waypoint update!

        String msg1 = String(WPCount, 3);
        String msg = "WP# Changed to : " + msg1 + ".";

        char sendto[21], message[300];
        ltoa(ReplytoPhoneNumber, sendto, 10) ;
        for (int i = 0; i < msg.length() ; i++)
        {
          msg.toCharArray(message, 300);

        }

        //Serial.println("!!!!!!!MESSAGE IS!!!!!!");
        //Serial.println(message);
        flushSerial();
        if (! fona.sendSMS(sendto, message)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }

        break;
      }

    case 'h': {
        // send an SMS reply to acknowledge halt command!

        String msg = "VESSEL HALTED.";

        char sendto[21], message[300];
        ltoa(ReplytoPhoneNumber, sendto, 10) ;
        for (int i = 0; i < msg.length() ; i++)
        {
          msg.toCharArray(message, 300);

        }

        //Serial.println("!!!!!!!MESSAGE IS!!!!!!");
        //Serial.println(message);
        flushSerial();
        if (! fona.sendSMS(sendto, message)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }

        break;
      }


    case 'R': {
        // send an SMS reply to acknowledge resume command!

        String msg = "VESSEL RESUMING.";

        char sendto[21], message[300];
        ltoa(ReplytoPhoneNumber, sendto, 10) ;
        for (int i = 0; i < msg.length() ; i++)
        {
          msg.toCharArray(message, 300);

        }

        //Serial.println("!!!!!!!MESSAGE IS!!!!!!");
        //Serial.println(message);
        flushSerial();
        if (! fona.sendSMS(sendto, message)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }

        break;
      }


    case 'D': {
        // send an SMS reply to acknowledge demo command!

        String msg = "DEMO MODE.";

        char sendto[21], message[300];
        ltoa(ReplytoPhoneNumber, sendto, 10) ;
        for (int i = 0; i < msg.length() ; i++)
        {
          msg.toCharArray(message, 300);

        }

        //Serial.println("!!!!!!!MESSAGE IS!!!!!!");
        //Serial.println(message);
        flushSerial();
        if (! fona.sendSMS(sendto, message)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }


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
        break;
      }





    /*
        /*send an SMS reply to Latitude and Logitude change command!
        case 'l': {

            //Message template Change ll 5 -96.75182,-96.75182

            //String msg1 = String(Waypoint, 5);
            String msg1 = String(Waypoint(5));
            String msg2 = String(WaypointLAT[5]);
            String msg3 = String(WaypointLONG[5]);

            String msg = "Latitude and Longitude Changed of waypoint number " + msg1 + " to : " + " " + msg2 + " , " + msg3 ;

            char sendto[21], message[300];
            ltoa(ReplytoPhoneNumber, sendto, 10) ;
            for (int i = 0; i < msg.length() ; i++)
            {
              msg.toCharArray(message, 300);

            }

            //Serial.println("!!!!!!!MESSAGE IS!!!!!!");
            //Serial.println(message);
            flushSerial();
            if (! fona.sendSMS(sendto, message)) {
              Serial.println(F("Failed"));
            } else {
              Serial.println(F("Sent!"));
            }

            break;
          }*/



    case 'p': {
        // send an SMS list of waypoints  ***********I still gotta do this one**********

        String msg1 = String(WaypointLAT[0], 6);
        String msg2 = String(WaypointLONG[0], 6);
        String msg3 = String(WaypointLAT[1], 6);
        String msg4 = String(WaypointLONG[1], 6);
        String msg5 = String(WaypointLAT[2], 6);
        String msg6 = String(WaypointLONG[2], 6);
        String msg7 = String(WaypointLAT[3], 6);
        String msg8 = String(WaypointLONG[3], 6);
        String msg9 = String(WaypointLAT[4], 6);
        String msg10 = String(WaypointLONG[4], 6);
        String msg11 = String(WaypointLAT[5], 6);
        String msg12 = String(WaypointLONG[5], 6);
        String msg13 = String(WaypointLAT[6], 6);
        String msg14 = String(WaypointLONG[6], 6);        
        String msg15 = String(WaypointLAT[7], 6);
        String msg16 = String(WaypointLONG[7], 6);        
        String msg17 = String(WaypointLAT[8], 6);
        String msg18 = String(WaypointLONG[8], 6);
        String msg19 = String(WaypointLAT[9], 6);
        String msg20 = String(WaypointLONG[9], 6);        
                
        String msg = "WP#: 0 \n LAT: " + msg1 + " \n LONG: " + msg2 + "\n \n "  +
                     "WP#: 1" + " \n " + "LAT: " + msg3 + " \n " + "LONG: " + msg4 + "\n " + "\n " +
                     "WP#: 2" + " \n " + "LAT: " + msg5 + " \n " + "LONG: " + msg6 + "\n " + "\n " +                     
                     "WP#: 3" + " \n " + "LAT: " + msg7 + " \n " + "LONG: " + msg8 + "\n " + "\n " +
                     "WP#: 4" + " \n " + "LAT: " + msg9 + " \n " + "LONG: " + msg10 + "\n " + "\n " +                                          
                     "WP#: 5" + " \n " + "LAT: " + msg11 + " \n " + "LONG: " + msg12 + "\n " + "\n " +
                     "WP#: 6" + " \n " + "LAT: " + msg13 + " \n " + "LONG: " + msg14 + "\n " + "\n " +
                     "WP#: 7" + " \n " + "LAT: " + msg15 + " \n " + "LONG: " + msg16 + "\n " + "\n " +
                     "WP#: 8" + " \n " + "LAT: " + msg17 + " \n " + "LONG: " + msg18 + "\n " + "\n " +
                     "WP#: 9" + " \n " + "LAT: " + msg19 + " \n " + "LONG: " + msg20 + "\n " + "\n ";


        
        char sendto[21], message[700];
        ltoa(ReplytoPhoneNumber, sendto, 10) ;
        for (int i = 0; i < msg.length() ; i++)
        {
          msg.toCharArray(message, 700);

        }
        break;
        
      }


    case 'W': {
        // Post data to website
        uint16_t statuscode;
        int16_t length;
        char url[80];
        char data[80];

        flushSerial();
        Serial.println(F("NOTE: in beta! Use simple websites to post!"));
        Serial.println(F("URL to post (e.g. httpbin.org/post):"));
        Serial.print(F("http://")); readline(url, 79);
        Serial.println(url);
        Serial.println(F("Data to post (e.g. \"foo\" or \"{\"simple\":\"json\"}\"):"));
        readline(data, 79);
        Serial.println(data);

        Serial.println(F("****"));
        if (!fona.HTTP_POST_start(url, F("text/plain"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length)) {
          Serial.println("Failed!");
          break;
        }
        while (length > 0) {
          while (fona.available()) {
            char c = fona.read();

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            UDR0 = c;
#else
            Serial.write(c);
#endif

            length--;
            if (! length) break;
          }
        }
        Serial.println(F("\n****"));
        fona.HTTP_POST_end();
        break;
      }
    /*****************************************/

    case 'S': {
        Serial.println(F("Creating SERIAL TUBE"));
        while (1) {
          while (Serial.available()) {
            delay(1);
            fona.write(Serial.read());
          }
          if (fona.available()) {
            Serial.write(fona.read());
          }
        }
        break;
      }

    default: {
        Serial.println(F("Unknown command"));
        //printMenu();
        break;
      }
  }
  // flush input
  flushSerial();
  while (fona.available()) {
    Serial.write(fona.read());
  }

}

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

char readBlocking() {
  while (!Serial.available());
  return Serial.read();
}
uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) {
    //Serial.print(c);
  }
  Serial.print(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    Serial.print(c);
    x *= 10;
    x += c - '0';
  }
  return x;
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) {
      //Serial.println(F("SPACE"));
      break;
    }

    while (Serial.available()) {
      char c =  Serial.read();

      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored
          continue;

        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }

    if (timeoutvalid && timeout == 0) {
      //Serial.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}

