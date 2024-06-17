

#define Company_Number 6901

#define Increment_Delay 1     
//ms
#define EEPROM_Delay 3  //EEPROM Write/Read Delay

float Depth_Data[100];
int TIME_SYNC_H[100];
int TIME_SYNC_M[100];
int TIME_SYNC_S[100];
int Stamp_Number[100];

float Stepper_MaxSpeed = 20000000;
float Stepper_MaxAcceleration = 1000000;
long Max_Distance = 90000;  //For Rotations

#define Activation_Indicator 2
#define Stepper_Enable 6

#define RadioIndicator 4
int RadioState = HIGH;// initial state of LED

#define LinkIndicator 3

int SavingState = LOW;// initial state of LED

bool Retrieve = false;

long rememberTime=0;// this is used by the code
long rememberTime2=0;

AT24CX MEMORY;

uRTCLib rtc(0x68);
RF24 radio(9, 10);             // CE, CSN
AccelStepper Engine(1, 7, 8);  // (Typeof driver: with 2 pins, STEP, DIR)

//const byte address[20] = "a629B7$zh05Cdbaa1807";
const byte addresses[][6] = { "00001", "00002" };

char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

boolean Activation = 0;

String ActiveState;
int Activate = 0;

float Depth;

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

unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 200;
unsigned long tempMillis;

bool Available = false;
