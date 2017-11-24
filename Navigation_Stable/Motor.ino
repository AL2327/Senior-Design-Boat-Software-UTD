void Motor(int THRT){


  //NOTES: 
  //THROTTLE IS FROM 0 - 180
  //0 IS MAXIMUM REVERSE
  //90 IS STOP
  //180 IS MAXIMUM FORWARD
  //THIS MEANS WE WILL BE USING 90-180 AS OUR THROTTLE OUTPUTS


  if (BatterySOC = 1)   //our battery is >80% charged.  We can use some power. 
  {
    THRT=168;    //this is approx 75% of maximum throttle. 
  }
  else if (BatterySOC = 2) // our battery is in its normal operating range 40-80% Charge
  {
    THRT=135;    //this is 50% throttle.  What I presume to be an ok cruising speed. 
  }
  else if (BatterySOC = 3) // our battery is low.  It is between 20-40% charged
  {
    THRT=112;   //this is approx 25% of throttle. We want to move...but not too fast now to conserve power. 
  }
  else if (BatterySOC = 4) // our battery is under 20% charged. This is critically low.
  {
    THRT=90;  //STOP all motor movement.  We are waiting. 
  }


  
  Throttle.write(THRT); //send command to set throttle
  
  Serial.println("MOTOR COMMAND: "); //
  Serial.println("MOTOR POWER (0 - 180) "); //
  Serial.println("Reverse 90 -> Neutral 0 -> Foward 180");
  Serial.print(THRT);
  Serial.println("");
  
    
  }
