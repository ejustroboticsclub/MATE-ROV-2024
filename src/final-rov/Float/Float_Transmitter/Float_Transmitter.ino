#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "Arduino.h"
#include "uRTCLib.h"
#include <AccelStepper.h>
#include <EEPROM.h>

#include "AT24CX.h"
#include "Declarations.h"

void setup() {

  Serial.begin(9600);
  Serial.println("E-JUST Robotics FLOAT Starting . . .");

  radio.begin();
  Wire.begin();
  delay(200);
  URTCLIB_WIRE.begin();

  pinMode(Activation_Indicator, OUTPUT);

  pinMode(RadioIndicator,OUTPUT);// define LEDpin as output
  digitalWrite(RadioIndicator,RadioState);// set initial state

  pinMode(LinkIndicator,OUTPUT);// define LEDpin as output
  digitalWrite(LinkIndicator,LOW);// set initial state

  pinMode(Stepper_Enable, OUTPUT);
  //rtc.set(40, 10, 7, 7, 20, 4, 24);
  //rtc.set(second, minute, hour, dayOfWeek, dayOfMonth, month, year)
  //set day of week (1=Sunday, 7=Saturday)

  radio.stopListening();
  radio.openWritingPipe(addresses[1]);     // 00002
  radio.openReadingPipe(1, addresses[0]);  // 00001
  radio.setPALevel(RF24_PA_MAX, 1);
  radio.setDataRate(RF24_2MBPS);
  radio.setRetries(0, 10);  // delay, count

  digitalWrite(Stepper_Enable, HIGH);
  Engine.setMaxSpeed(Stepper_MaxSpeed);             // Set maximum speed value for the stepper
  Engine.setAcceleration(Stepper_MaxAcceleration);  // Set acceleration value for the stepper
  Engine.setCurrentPosition(0);                     // Set the current position to 0 steps


  Check_EEPROM();
  //EEPROM_READ();

  delay(500);
  Serial.println("Float Started, Pending Activation");
}
void loop() {

  rtc.refresh();
  Transmit_Data();
  Recieve_Data();
  Activate = station_data.Activation;
  Request_Depth();
  Serial_Debug();
  
  
  

  while (Activate == 1) {
    digitalWrite(Stepper_Enable, LOW);
    digitalWrite(Activation_Indicator, HIGH);
    digitalWrite(RadioIndicator, LOW);
    ActiveState = "Activated";

    for (int A = 0, MEM = 0, Hour = 0, Minute = 200, Second = 400, STAMP = 600; (A < 50) && (MEM < 1600) && (Hour < 100) && (Minute < 300) && (Second < 500) && (STAMP < 700); A++, Hour += 2, Minute += 2, Second += 2, STAMP += 2, MEM += 32) {

      Engine.setCurrentPosition(0);
      Engine.runToNewPosition((Max_Distance / 50));
      Engine.setSpeed(Max_Distance / 10);

      delay(Increment_Delay);
      Request_Depth();
      Depth_Data[A] = 11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111;

      rtc.refresh();
      TIME_SYNC_H[A] = rtc.hour();
      TIME_SYNC_M[A] = rtc.minute();
      TIME_SYNC_S[A] = rtc.second();
      Stamp_Number[A] = A;

      MEMORY.writeFloat(MEM, Depth);
      EEPROM.update(Hour, rtc.hour());
      EEPROM.update(Minute, rtc.minute());
      EEPROM.update(Second, rtc.second());
      EEPROM.update(STAMP, A);
      delay(EEPROM_Delay);
    }


    for (int A = 50, I = 0, MEM = 1600, Hour = 100, Minute = 300, Second = 500, STAMP = 700; (A < 100) && (I < Max_Distance) && (MEM < 3200) && (Hour < 200) && (Minute < 400) && (Second < 600) && (STAMP < 800); I += (Max_Distance / 50), A++, Hour += 2, Minute += 2, Second += 2, STAMP += 2, MEM += 32) {

      Engine.setCurrentPosition(0);
      Engine.runToNewPosition(-(Max_Distance / 50));
      Engine.setSpeed(Max_Distance / 10);

      delay(Increment_Delay);

      Depth = Request_Depth();
      Depth_Data[A] = Depth;

      rtc.refresh();
      TIME_SYNC_H[A] = rtc.hour();
      TIME_SYNC_M[A] = rtc.minute();
      TIME_SYNC_S[A] = rtc.second();
      Stamp_Number[A] = A;

      MEMORY.writeFloat(MEM, Depth);
      EEPROM.update(Hour, TIME_SYNC_H[A]);
      EEPROM.update(Minute, TIME_SYNC_M[A]);
      EEPROM.update(Second, TIME_SYNC_S[A]);
      EEPROM.update(STAMP, A);
      delay(EEPROM_Delay);
    }
    digitalWrite(Activation_Indicator, LOW);
    Print_Arrays();
    ActiveState = "Deactivated";
    digitalWrite(Stepper_Enable, HIGH);
    Engine.setCurrentPosition(0);
    Recieve_Data();
    Activate = 0;
  }


if (Retrieve == true){
  for (int A = 0, MEM = 0, Hour = 0, Minute = 200, Second = 400, STAMP = 600; (A < 50) && (MEM < 1600) && (Hour < 100) && (Minute < 300) && (Second < 500) && (STAMP < 700); A++, Hour += 2, Minute += 2, Second += 2, STAMP += 2, MEM += 32) {

    rtc.refresh();
      float_data.Year = rtc.year();
    float_data.Month = rtc.month();
    float_data.Day = rtc.day();
    float_data.Hour = rtc.hour();
    float_data.Minute = rtc.minute();
    float_data.Second = rtc.second();
    float_data.Company_No = Company_Number;

    float_data.DEPTH = MEMORY.readFloat(MEM);
    float_data.TrialHour = EEPROM.read(Hour);
    float_data.TrialMinute = EEPROM.read(Minute);
    float_data.TrialSecond = EEPROM.read(Second);
    float_data.Stamp_Number = EEPROM.read(STAMP);

    float_data.timingInterval = tempMillis;

    radio.stopListening();
    radio.write(&float_data, sizeof(FLOAT_DATA));
    delay(100);
    Recieve_Data();
    Radio_Indicate();
  }

  for (int A = 50, I = 0, MEM = 1600, Hour = 100, Minute = 300, Second = 500, STAMP = 700; (A < 100) && (I < Max_Distance) && (MEM < 3200) && (Hour < 200) && (Minute < 400) && (Second < 600) && (STAMP < 800); I += (Max_Distance / 50), A++, Hour += 2, Minute += 2, Second += 2, STAMP += 2, MEM += 32) {

    rtc.refresh();
      float_data.Year = rtc.year();
    float_data.Month = rtc.month();
    float_data.Day = rtc.day();
    float_data.Hour = rtc.hour();
    float_data.Minute = rtc.minute();
    float_data.Second = rtc.second();
    float_data.Company_No = Company_Number;

    float_data.DEPTH = MEMORY.readFloat(MEM);
    float_data.TrialHour = EEPROM.read(Hour);
    float_data.TrialMinute = EEPROM.read(Minute);
    float_data.TrialSecond = EEPROM.read(Second);
    float_data.Stamp_Number = EEPROM.read(STAMP);

    float_data.timingInterval = tempMillis;

    radio.stopListening();
    radio.write(&float_data, sizeof(FLOAT_DATA));
    delay(100);
    Recieve_Data();
    Radio_Indicate();
  }
  Retrieve = false;
}

  char State = Serial.read();
  if (State == 'A') {
    Activate = 1;
  } else if (State == 'D') {
    Activate = 0;
  } 
  digitalWrite(RadioIndicator, LOW);
}
