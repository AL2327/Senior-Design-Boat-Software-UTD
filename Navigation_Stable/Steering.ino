void Steering(double courseToWaypoint) {

  //double courseError;

  //Serial.println("VESSEL STATUS: Heading: "); //

  //Serial.print(Heading);
  //Serial.print(" degrees. Course-to=");
  //Serial.print(courseToWaypoint, 6);
  //Serial.print(F(" degrees. ["));
  //Serial.println("");

  int courseError = (Heading - courseToWaypoint);

  Serial.println("Course Error: "); //
  Serial.print(courseError);
  Serial.println("");

  int heading = (int)Heading % 360;           //which way we are going as integer mod 360
  int bearing = (int)courseToWaypoint % 360;  //which way we want to go as integer mod 360

  int dir = ((360 - ((int)heading - (int)bearing)) % 360);


  //  Serial.println("DIRECTION: "); //
  //  Serial.print(dir);
  //  Serial.println("");


  if (dir < 180) {
    if (dir < 30) {
      Serial.println("DIR: < 180 and < 30 "); //
      Serial.print(dir);
      Serial.println("");
      pos = abs((180- ((180 - dir)) / 2));

    }
    else {
      Serial.println("DIR: < 180 and > 30 "); //
      Serial.print(dir);
      Serial.println("");
      pos = 145;

    }
  }
  else if (dir >= 180) {
    if ((360 - dir) > 30)
    {
      Serial.println("DIR: >= 180 and (360-dir) > 30 "); //
      Serial.print(dir);
      Serial.println("");
      pos  = 35;
    }
    else
    {
      Serial.println("DIR: >= 180 and (360-dir) < 30 "); //
      Serial.print(dir);
      Serial.println("");
      pos = abs(90 - dir / 2);
    }
  }

  else if (dir == 360 or dir == 0) {
    pos = 90;
  }

  //pos = constrain(pos, 35, 145);
  Rudder.write(pos);

  Serial.println("RUDDER COMMAND: "); //
  Serial.println("Commanded Rudder Posistion: 35-145, 90=Straight ");
  Serial.print(pos);
  Serial.println("");


  /*
    if(courseError <= -149 && courseError >= -179)
    {
          pos = 150;        //make it turn full left
    }

    else if(courseError >= -148 && courseError <=-100)
    {
                pos=pos+30;
    }
    else if(courseError >= -99  && courseError <= -30)
    {
                pos=pos+15;
    }

    else if(courseError >=-29 && courseError <=-10)
    {
                pos=pos+10;
    }

    else if(courseError >=9 && courseError <0)
    {
                pos=pos+3;
    }

    //*Other direction
    if(courseError >= 149 && courseError <= 179)
    {
          pos = 150;
    }

    else if(courseError <= 148 && courseError >=100)
    {
          pos=pos-30;
    }
    else if(courseError <= 99  && courseError >= 30)
    {
          pos=pos-15;
    }

    else if(courseError <=29 && courseError >=10)
    {
          pos=pos-10;
    }

    else if(courseError <=9 && courseError >0)
    {
          pos=pos-3;
    }

  */




  /*  /*STEERING CODE
    PIDRudder.Compute();          // PID computation
    pos = map(pidoutput, 0, 255, 0, 180);  //map PID output(double) to pos(integer) range
    pos = constrain(pos, 45, 135);        //contrain the rudder to +/- 30 deg.
    Rudder.write(pos);              // tell servo to go to position in variable 'pos'

    Serial.println("PID OUTPUT: ");
    Serial.print(pidoutput);
    Serial.println("");
    Serial.println("RUDDER OUTPUT: ");
    Serial.print(pos);
    Serial.println("");
    /*END STEERING CODE*/

}

