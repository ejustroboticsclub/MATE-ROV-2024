void Serial_Debug() {

  Serial.print("Date & Time: ");
  Serial.print(rtc.year());
  Serial.print('/');
  Serial.print(rtc.month());
  Serial.print('/');
  Serial.print(rtc.day());

  Serial.print(" (");
  Serial.print(daysOfTheWeek[rtc.dayOfWeek() - 1]);
  Serial.print(") ");

  Serial.print(rtc.hour());
  Serial.print(':');
  Serial.print(rtc.minute());
  Serial.print(':');
  Serial.print(rtc.second());

  Serial.print("  Company Number: ");
  Serial.print(Company_Number);
  Serial.print("  ");
  Serial.print("Float State: ");
  Serial.print(ActiveState);
  Serial.print("  ");
  Serial.print("Depth: ");
  Serial.println(float_data.DEPTH);
}


void Print_Arrays() {
    for (int A = 0, MEM = 0, Hour = 0, Minute = 200, Second = 400, STAMP = 600; (A < 100) && (MEM < 3200) && (Hour < 200) && (Minute < 400) && (Second < 600) && (STAMP < 800); A++, Hour += 2, Minute += 2, Second += 2, STAMP += 2, MEM += 32) {
    
    Serial.print("Stamp: ");
    Serial.print(EEPROM.read(STAMP));
    Serial.print("  Time: ");
    Serial.print(TIME_SYNC_H[A]);
    Serial.print(":");
    Serial.print(TIME_SYNC_M[A]);
    Serial.print(":");
    Serial.print(TIME_SYNC_S[A]);
    Serial.print("  ");
    Serial.print("Depth: ");
    Serial.print(Depth_Data[A]);

    Serial.print("    EEPROM OUTPUT --->  ");

    Serial.print("  Stamp: ");
    Serial.print(EEPROM.read(STAMP));
    Serial.print("  Time: ");
    Serial.print(EEPROM.read(Hour));
    Serial.print(":");
    Serial.print(EEPROM.read(Minute));
    Serial.print(":");
    Serial.print(EEPROM.read(Second));
    Serial.print("  ");
    Serial.print("Depth: ");
    Serial.println(MEMORY.readFloat(MEM));
    
    Saving_Indicate();
  }
}