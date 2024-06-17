void Check_EEPROM() {
  if (MEMORY.read(1) != 12) {
    delay(50);
    MEMORY.write(1, 12);
  }
}
/*
void EEPROM_READ() {
  Kp = MEMORY.readFloat(70);
  delay(EEPROM_Delay);
  Ki = MEMORY.readFloat(75);
  delay(EEPROM_Delay);
  Kd = MEMORY.readFloat(80);
  delay(EEPROM_Delay);
  SteerCoef = MEMORY.read(11);
  delay(EEPROM_Delay);
  ACCELERATION = MEMORY.read(14);
  delay(EEPROM_Delay);
  SPEED = MEMORY.read(17);
  delay(EEPROM_Delay);
}


void EEPROM_WRITE() {
  MEMORY.writeFloat(70, Kp);  //kp
  delay(EEPROM_Delay);
  MEMORY.writeFloat(75, Ki);  //ki
  delay(EEPROM_Delay);
  MEMORY.writeFloat(80, Kd);  //kd
  delay(EEPROM_Delay);
  MEMORY.write(11, SteerCoef);  //steer
  delay(EEPROM_Delay);
  MEMORY.write(14, ACCELERATION);  //Accel
  delay(EEPROM_Delay);
  MEMORY.write(17, SPEED);  //max speed
  delay(EEPROM_Delay);
}
*/