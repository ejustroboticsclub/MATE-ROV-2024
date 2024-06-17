#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(9, 10);  // CE, CSN

//const byte address[20] = "a629B7$zh05Cdbaa1807";
const byte addresses[][6] = { "00001", "00002" };
char State;
char daysOfTheWeek[7][12] = { "Saturday", "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday" };

unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 200;
unsigned long tempMillis;

float Depth_Data[100];
int TIME_SYNC_H[100];
int TIME_SYNC_M[100];
int TIME_SYNC_S[100];

boolean Activation = 0;

struct FLOAT_DATA {
  int Year;
  int Month;
  int Day;
  int Hour;
  int Minute;
  int Second;
  int Company_No;
  int Stamp_Number;
  int TrialHour;
  int TrialMinute;
  int TrialSecond;
  float DEPTH;
  unsigned long timingInterval;
};
struct STATION_DATA {
  int Activation;
  bool Retrieve_State;
};
FLOAT_DATA float_data;
STATION_DATA station_data;

bool Available = false;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(addresses[0]);     // 00001
  radio.openReadingPipe(1, addresses[1]);  // 00002
  radio.setPALevel(RF24_PA_MAX, 1);
  radio.setDataRate(RF24_2MBPS);
  radio.setRetries(0, 10);
  Serial.println("E-JUST Robotics Station Starting . . .");
  delay(500);
  pinMode(8, INPUT_PULLUP);
}
void loop() {
  currentMillis = millis();
  Check_Serial();
  
  radio.startListening();
  if (radio.available()) {
    radio.read(&float_data, sizeof(FLOAT_DATA));
  }

  //Print_Arrays();
  View_Data();

  

  if(digitalRead(8) == 0){
    station_data.Activation = 1;
  }
  else {
  station_data.Activation = 0;
  }
  radio.stopListening();
  // radio.write(&station_data, sizeof(station_data));
    radio.write(&station_data, sizeof(station_data));
    station_data.Activation = 0;

  }


void Print_Arrays() {
  for (int i = 0; i < 100; i++) {
    Serial.print("Time: ");
    Serial.print(TIME_SYNC_H[i]);
    Serial.print(":");
    Serial.print(TIME_SYNC_M[i]);
    Serial.print(":");
    Serial.print(float_data.Second);
    Serial.print("  ");
    Serial.print("Depth: ");
    Serial.println(Depth_Data[i]);
  }
}

void View_Data() {

  Serial.print("Date & Time: ");
  Serial.print(float_data.Year);
  Serial.print('/');
  Serial.print(float_data.Month);
  Serial.print('/');
  Serial.print(float_data.Day);
  Serial.print("  ");
  Serial.print(float_data.Hour);
  Serial.print(':');
  Serial.print(float_data.Minute);
  Serial.print(':');
  Serial.print(float_data.Second);
  Serial.print("  Company Number: ");
  Serial.print(float_data.Company_No);
  Serial.print("  -  Float State: ");
  Serial.print(station_data.Activation);
  Serial.print("  -   Stamp: ");
  Serial.print(float_data.Stamp_Number);
  Serial.print("  Time: ");
  Serial.print(float_data.TrialHour);
  Serial.print(":");
  Serial.print(float_data.TrialMinute);
  Serial.print(":");
  Serial.print(float_data.TrialSecond);
  Serial.print("  Depth: ");
  Serial.print(float_data.DEPTH);
  Serial.print("  Retrieve: ");
  Serial.println(station_data.Retrieve_State);


  //Serial.println(Data.timingInterval);
}

void Check_Serial() {

  State = Serial.read();
  if (State == 'A') {
    station_data.Activation = 1;
  } else if (State == 'D') {
    station_data.Activation = 0;
  }

  if (State == 'R') {
    station_data.Retrieve_State = true;
  } else if (State == 'N') {
    station_data.Retrieve_State = false;
  }
}