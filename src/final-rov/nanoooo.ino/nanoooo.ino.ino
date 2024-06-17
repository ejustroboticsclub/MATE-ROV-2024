#include <SPI.h>
#include <Servo.h>
//ROS///////////////////////////////////////
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>

////////////////////////////////////////////

const int clock = 9;
const int calibrationReadings = 50;
float depth_cm;

float atmospheric_pressure = ;
long PCOMP;
float Offset ;
Servo servos[6];

//ROS///////////////////////////////////////////////

void thrustersCallback(const std_msgs::Int32MultiArray& msg);
void gripperLCallback(const std_msgs::Bool& msg);
void gripperRCallback(const std_msgs::Bool& msg);


ros::NodeHandle nh;

std_msgs::Float64 depth_msg;  // Declare depth_msg globally

ros::Subscriber<std_msgs::Int32MultiArray> thrusters_sub("/ROV/thrusters", &thrustersCallback);
ros::Subscriber<std_msgs::Bool> gripper_r_sub("/ROV/gripper_r", &gripperRCallback);
ros::Subscriber<std_msgs::Bool> gripper_l_sub("/ROV/gripper_l", &gripperLCallback);

ros::Publisher depth_pub("/ROV/depth", &depth_msg);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void resetsensor() //this function keeps the sketch a little shorter
{
  SPI.setDataMode(SPI_MODE0);
  SPI.transfer(0x15);
  SPI.transfer(0x55);
  SPI.transfer(0x40);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float measure() {
  TCCR1B = (TCCR1B & 0xF8) | 1 ; //generates the MCKL signal
  analogWrite (clock, 128) ;
  resetsensor();//resets the sensor - caution: afterwards mode = SPI_MODE0!
  //Calibration word 1
  unsigned int word1 = 0;
  unsigned int word11 = 0;
  SPI.transfer(0x1D); //send first byte of command to get calibration word 1
  SPI.transfer(0x50); //send second byte of command to get calibration word 1
  SPI.setDataMode(SPI_MODE1); //change mode in order to listen
  word1 = SPI.transfer(0x00); //send dummy byte to read first byte of word
  word1 = word1 << 8; //shift returned byte
  word11 = SPI.transfer(0x00); //send dummy byte to read second byte of word
  word1 = word1 | word11; //combine first and second byte of word
  resetsensor();//resets the sensor
  //Calibration word 2; see comments on calibration word 1
  unsigned int word2 = 0;
  byte word22 = 0;
  SPI.transfer(0x1D);
  SPI.transfer(0x60);
  SPI.setDataMode(SPI_MODE1);
  word2 = SPI.transfer(0x00);
  word2 = word2 << 8;
  word22 = SPI.transfer(0x00);
  word2 = word2 | word22;
  resetsensor();//resets the sensor
  //Calibration word 3; see comments on calibration word 1
  unsigned int word3 = 0;
  byte word33 = 0;
  SPI.transfer(0x1D);
  SPI.transfer(0x90);
  SPI.setDataMode(SPI_MODE1);
  word3 = SPI.transfer(0x00);
  word3 = word3 << 8;
  word33 = SPI.transfer(0x00);
  word3 = word3 | word33;
  resetsensor();//resets the sensor
  //Calibration word 4; see comments on calibration word 1
  unsigned int word4 = 0;
  byte word44 = 0;
  SPI.transfer(0x1D);
  SPI.transfer(0xA0);
  SPI.setDataMode(SPI_MODE1);
  word4 = SPI.transfer(0x00);
  word4 = word4 << 8;
  word44 = SPI.transfer(0x00);
  word4 = word4 | word44;
  long c1 = word1 >> 1;
  long c2 = ((word3 & 0x3F) << 6) | ((word4 & 0x3F));
  long c3 = (word4 >> 6) ;
  long c4 = (word3 >> 6);
  long c5 = (word2 >> 6) | ((word1 & 0x1) << 10);
  long c6 = word2 & 0x3F;
  resetsensor();//resets the sensor
  //Temperature:
  unsigned int tempMSB = 0; //first byte of value
  unsigned int tempLSB = 0; //last byte of value
  unsigned int D2 = 0;
  SPI.transfer(0x0F); //send first byte of command to get temperature value
  SPI.transfer(0x20); //send second byte of command to get temperature value
  delay(35); //wait for conversion end
  SPI.setDataMode(SPI_MODE1); //change mode in order to listen
  tempMSB = SPI.transfer(0x00); //send dummy byte to read first byte of value
  tempMSB = tempMSB << 8; //shift first byte
  tempLSB = SPI.transfer(0x00); //send dummy byte to read second byte of value
  D2 = tempMSB | tempLSB; //combine first and second byte of value
  resetsensor();//resets the sensor
  //Pressure:
  unsigned int presMSB = 0; //first byte of value
  unsigned int presLSB = 0; //last byte of value
  unsigned int D1 = 0;
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  SPI.transfer(0x0F); //send first byte of command to get pressure value
  SPI.transfer(0x40); //send second byte of command to get pressure value
  delay(35); //wait for conversion end
  SPI.setDataMode(SPI_MODE1); //change mode in order to listen
  presMSB = SPI.transfer(0x00); //send dummy byte to read first byte of value
  presMSB = presMSB << 8; //shift first byte
  presLSB = SPI.transfer(0x00); //send dummy byte to read second byte of value
  D1 = presMSB | presLSB;
  const long UT1 = (c5 * 8) + 20224;
  const long dT = (D2 - UT1);
  const long OFF = (c2 * 4) + (((c4 - 512) * dT) / 4096);
  const long SENS = c1 + ((c3 * dT) / 1024) + 24576;
  PCOMP = ((((SENS * (D1 - 7168)) / 16384) - OFF) / 32) + 250;
  // Density of water (kg/m^3)
  const float density_water = 1000.0;
  // Acceleration due to gravity (m/s^2)
  const float gravity = 9.81;

  // Calculate depth underwater in meters
  float depth_meters = PCOMP / (density_water * gravity);
  // Convert depth from meters to centimeters
  depth_cm = depth_meters * 10000.0 - 1 ;

  return  depth_cm ;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float calibrate() {
  float sum = 0;

  // Iterate over the calibration readings
  for (int i = 0; i < calibrationReadings; i++) {
    // Measure function call here
    float measurement = measure(); // Assuming measure() returns a float

    // Add the measurement to the sum
    sum += measurement;
  }

  // Calculate the average
  float calibratedPressure = sum / calibrationReadings;

  // Return the calibrated pressure value
  return calibratedPressure;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void setup() {
  Serial.begin(9600);
  SPI.begin(); //see SPI library details on arduino.cc for details
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32); //divide 16 MHz to communicate on 500 kHz
  pinMode(clock, OUTPUT);
  delay(100);

  //thrusters/////////////////////////////////////////////////////////
  for (int i = 2; i <= 7; i++) {
    pinMode(i, OUTPUT);
  }

  for (int i = 0; i < 6; i++) {
    servos[i].attach(2 + i);
    servos[i].writeMicroseconds(1500); // .
    delay(7000);
  }
  //////////////////////////////////////////////////////////////
  Offset = calibrate();

  //ROS//////////////////////////
  nh.initNode();

  // Advertise publishers
  nh.advertise(depth_pub);


  // Subscribe to topics
  nh.subscribe(thrusters_sub);
  nh.subscribe(gripper_r_sub);
  nh.subscribe(gripper_l_sub);
}


void loop()
{
  depth_cm = measure();

  depth_cm -= Offset;

  // Standard atmospheric pressure in mbar


  // Print depth underwater
  Serial.print("Depth underwater: ");
  Serial.print(depth_cm);
  Serial.println(" cm");


  //ROS///////////////////////////////////////////////////

  // Publish the message
  depth_msg.data = depth_cm;
  depth_pub.publish(&depth_msg);
  nh.spinOnce();
  // Add a delay to control the publishing rate

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gripperLCallback(const std_msgs::Bool& msg) {
  // Handle gripperL message here
  if (msg.data) {
    // If gripperL message is true, set pin A0 high
    digitalWrite(A0, HIGH);
  } else {
    // If gripperL message is false, set pin A0 low
    digitalWrite(A0, LOW);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void thrustersCallback(const std_msgs::Int32MultiArray& msg) {
  // Handle thrusters message here
  for (int i = 0; i < msg.data_length; i++) {
    // Set servo position based on thruster speed
    int speed = msg.data[i];
    if (speed >= 1100 && speed <= 1900) {
      servos[i].writeMicroseconds(speed);
      delay(100); // Add a small delay between each servo write to prevent overload
    } else {
      Serial.println("Invalid thruster speed!");
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gripperRCallback(const std_msgs::Bool& msg) {
  // Handle gripperR message here
  if (msg.data) {
    // If gripperR message is true, set pin 1A high
    digitalWrite(A1, HIGH);
  } else {
    // If gripperR message is false, set pin 1A low
    digitalWrite(A1, LOW);
  }
}
