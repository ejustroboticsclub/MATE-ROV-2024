void Transmit_Data() {
    delay(50);
    float_data.Year = rtc.year();
    float_data.Month = rtc.month();
    float_data.Day = rtc.day();
    float_data.Hour = rtc.hour();
    float_data.Minute = rtc.minute();
    float_data.Second = rtc.second();
    float_data.Company_No = Company_Number;

    float_data.DEPTH = Depth;
    float_data.TrialHour = 0;
    float_data.TrialMinute = 0;
    float_data.TrialSecond = 0;
    float_data.Stamp_Number = 0;

    float_data.timingInterval = tempMillis;

    radio.stopListening();
    radio.write(&float_data, sizeof(FLOAT_DATA));
}

void Recieve_Data() {
  radio.startListening();
  if (radio.available()) {
    radio.read(&station_data, sizeof(STATION_DATA));
    digitalWrite(LinkIndicator, HIGH);
    Retrieve = station_data.Retrieve_State;
  }
  else {
  digitalWrite(LinkIndicator, LOW);
  }
}