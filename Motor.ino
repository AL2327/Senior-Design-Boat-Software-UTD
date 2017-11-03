void Motor(int THRT){
  
  Throttle.write(THRT); //send command to set throttle
  
  Serial.println("MOTOR COMMAND: "); //
  Serial.println("MOTOR POWER (0 - 180) "); //
  Serial.print(THRT);
  Serial.println("");
  Throttle.write(THRT); //send command to set throttle
    
  }
