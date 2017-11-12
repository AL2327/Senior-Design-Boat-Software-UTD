void beep(int beeping) {

  int i;

  for (i = 0; i < beeping; i++) {
    digitalWrite(buzzer, HIGH);
    delay(50);
    digitalWrite(buzzer, LOW);
    delay(50);
  }

}

