#include <OneWire.h>
#include <DallasTemperature.h>
#include <ros.h>
#include <std_msgs/Float64.h>

#define ONE_WIRE_BUS 9

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

float Celsius = 0;

ros::NodeHandle nh;
std_msgs::Float64 temp_msg;
ros::Publisher temp_pub("/ROV/temperature", &temp_msg);

void setup() {
  sensors.begin();
  nh.initNode();
  nh.advertise(temp_pub);
}

void loop() {
  sensors.requestTemperatures();

  Celsius = sensors.getTempCByIndex(0);
  
  temp_msg.data = Celsius;
  temp_pub.publish(&temp_msg);
  nh.spinOnce();
  delay(10);
}
