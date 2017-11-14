void Steering(double courseToWaypoint) {

  //double courseError;

  //Serial.println("VESSEL STATUS: Heading: "); //

  //Serial.print(Heading);
  //Serial.print(" degrees. Course-to=");
  //Serial.print(courseToWaypoint, 6);
  //Serial.print(F(" degrees. ["));
  //Serial.println("");

  float courseError = (Heading - courseToWaypoint);

  Serial.println("Course Error: "); //
  Serial.print(courseError);
  Serial.println("");




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

/*Other direction*/
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

else if(courseError == 0){
    pos=90;
}

pos=constrain(pos, 30, 150);
Rudder.write(pos);

  Serial.println("RUDDER COMMAND: "); //
  Serial.println("Commanded Rudder Posistion: ");
  Serial.print(pos);
  Serial.println("");

  
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

