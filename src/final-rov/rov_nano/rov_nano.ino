#include <SPI.h>
//ROS///////////////////////////////////////
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>

////////////////////////////////////////////

const int clockPin = 9;
const int calibrationReadings = 50;
const float density_water = 1000.0;
const float gravity = 9.81;
const byte hexes[] = {0x50, 0x60, 0x90, 0xA0};
long PCOMP = 0;
float Offset = 0.0;
float depth_cm = 0.0;


//ROS///////////////////////////////////////////////

ros::NodeHandle nh;

std_msgs::Float64 depth_msg;  // Declare depth_msg globally

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
  static unsigned int words[4] = {0};
  static byte bytes[4] = {0};

  TCCR1B = (TCCR1B & 0xF8) | 1 ; //generates the MCKL signal
  analogWrite (clockPin, 128) ;
  resetsensor();//resets the sensor - caution: afterwards mode = SPI_MODE0!
  for(int i = 0; i < 4; i++)
  {
    SPI.transfer(0x1D); //send first byte of command to get calibration word
    SPI.transfer(hexes[i]); //send second byte of command to get calibration word
    SPI.setDataMode(SPI_MODE1); //change mode in order to listen
    words[i] = SPI.transfer(0x00); //send dummy byte to read first byte of word
    words[i] = words[i] << 8; //shift returned byte
    bytes[i] = SPI.transfer(0x00); //send dummy byte to read second byte of word
    words[i] = words[i] | bytes[i]; //combine first and second byte of word
    resetsensor();//resets the sensor
  }
  
  long c1 = words[0] >> 1;
  long c2 = ((words[2] & 0x3F) << 6) | (words[3] & 0x3F);
  long c3 = words[3] >> 6;
  long c4 = words[2] >> 6;
  long c5 = (words[1] >> 6) | ((words[0] & 0x1) << 10);
  long c6 = words[1] & 0x3F;
  resetsensor();//resets the sensor
  //Temperature:
  SPI.transfer(0x0F); //send first byte of command to get temperature value
  SPI.transfer(0x20); //send second byte of command to get temperature value
  delay(35); //wait for conversion end
  bytes[0] = SPI.transfer(0x00);
  bytes[1] = SPI.transfer(0x00);
  int D2 = (bytes[0] << 8) | bytes[1];
  resetsensor();
  //Pressure:
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  SPI.transfer(0x0F); //send first byte of command to get pressure value
  SPI.transfer(0x40); //send second byte of command to get pressure value
  delay(35); //wait for conversion end
  bytes[0] = SPI.transfer(0x00);
  bytes[1] = SPI.transfer(0x00);
  int D1 = (bytes[0] << 8) | bytes[1];
  
  const long UT1 = (c5 * 8) + 20224;
  const long dT = (D2 - UT1);
  const long OFF = (c2 * 4) + (((c4 - 512) * dT) / 4096);
  const long SENS = c1 + ((c3 * dT) / 1024) + 24576;
  PCOMP = ((((SENS * (D1 - 7168)) / 16384) - OFF) / 32) + 250;
  // Density of water (kg/m^3)
  // Acceleration due to gravity (m/s^2)

  // Calculate depth underwater in meters
  float depth_meters = PCOMP / (density_water * gravity);
  // Convert depth from meters to centimeters
  depth_cm = depth_meters * 10000.0 - 1 ;

  return  depth_cm ;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float calibrate() {
  float sum = 0;

  for (int i = 0; i < calibrationReadings; i++) {
    sum += measure();
  }

  return sum / calibrationReadings;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void setup() {
  SPI.begin(); //see SPI library details on arduino.cc for details
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32); //divide 16 MHz to communicate on 500 kHz
  pinMode(clockPin, OUTPUT);
  delay(100);

  Offset = calibrate();

  //ROS//////////////////////////
  nh.initNode();

  // Advertise publishers
  nh.advertise(depth_pub);
}


void loop()
{ 
    // It always measure the depth no matter what the value of the calibrate flag, only the activate flag matters.
  depth_cm = measure();
  depth_cm -= Offset;
  // Standard atmospheric pressure in mbar


// Print depth underwater
//  Serial.print("Depth underwater: ");
//  Serial.print(depth_cm);
//  Serial.println(" cm");


  //ROS///////////////////////////////////////////////////

  // Publish the message
  depth_msg.data = depth_cm;
  depth_pub.publish(&depth_msg);
  nh.spinOnce();
  delay(10);
  // Add a delay to control the publishing rate

}
