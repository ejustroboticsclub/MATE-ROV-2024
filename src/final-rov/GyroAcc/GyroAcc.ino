#include <Arduino_LSM6DS3.h>
#include <cmath> // Include the math library
#include <ros.h>
#include <sensor_msgs/Imu.h>

const float pi = M_PI; // Assign the value of pi from the math library

// Constants for gyroscope sensitivity
const float gyroSensitivity = 1.0; // Gyroscope sensitivity: 500 degrees per second

// Variables to store gyroscope data
float gyroX, gyroY, gyroZ;
float accX, accY, accZ;

// Variables for angle calculation
float angleX = 0.0; // Angle around X-axis
float angleY = 0.0; // Angle around Y-axis
float angleZ = 0.0; // Angle around Z-axis

// Calibration variables
const int calibrationReadings = 3000;
float gyroXOffset = 0.0;
float gyroYOffset = 0.0;
float gyroZOffset = 0.0;
float accXOffset = 0.0;
float accYOffset = 0.0;
float accZOffset = 0.0;

// Time variables for integrating gyroscope data
unsigned long prevTime = 0; // Previous time stamp
unsigned long currTime = 0; // Current time stamp

ros::NodeHandle node_handle;

// Initialize an 32-bit float array that hosts the imu readings in the form of [gX, gY, gZ, accX, accY, accZ]
sensor_msgs::Imu imu_readings; 

// Initialize a publisher over the /imu/data topic
ros::Publisher publisher("/ROV/imu", &imu_readings);

void setup() {
      // Initialize ROS node and advertise the publisher
  node_handle.initNode();
  node_handle.advertise(publisher);
  
//  Serial.begin(9600);
//  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }


//  
//  Serial.print("Gyroscope sample rate = ");
//  Serial.print(IMU.gyroscopeSampleRate());
//  Serial.println(" Hz");
//  Serial.println();
//  Serial.println("Angle (degrees)");
//  Serial.println("X\tY\tZ");

  // Perform calibration
  calibrate();

  // Initialize previous time
  prevTime = millis();


}

void loop() {
  // Read gyroscope data
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    // Remove calibration offsets
    gyroX -= gyroXOffset;
    gyroY -= gyroYOffset;
    gyroZ -= gyroZOffset;

    // Calculate time elapsed since last loop iteration
    currTime = millis();
    float dt = (currTime - prevTime) / 1000.0; // Convert to seconds
    // Update previous time
    prevTime = currTime;
    
    // Integrate gyroscope data to calculate angle change
    angleX += gyroX * dt / gyroSensitivity * 0.99;
    angleY += gyroY * dt / gyroSensitivity * 0.99;
    angleZ += gyroZ * dt / gyroSensitivity * 0.99;



//    // Print angles to serial monitor
//    Serial.print("Gyro:\t");
//    Serial.print(angleX);
//    Serial.print('\t');
//    Serial.print(angleY);
//    Serial.print('\t');
//    Serial.println(angleZ);

    
    imu_readings.angular_velocity.x = gyroX;
    imu_readings.angular_velocity.y = gyroY;
    imu_readings.angular_velocity.z = gyroZ;
  }

  // Read accelerometer data
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
//    
//    Serial.print("Accel:\t");
//    Serial.print(accX);
//    Serial.print('\t');
//    Serial.print(accY);
//    Serial.print('\t');
//    Serial.println(accZ);

    imu_readings.linear_acceleration.x = accX;
    imu_readings.linear_acceleration.y = accY;
    imu_readings.linear_acceleration.z = accZ;
  }
  
  imu_readings.orientation.x = angleX;
  imu_readings.orientation.y = angleY;
  imu_readings.orientation.z = angleZ;
  
  // Publish the imu readings to the /imu/data topic
  publisher.publish(&imu_readings);

  // Call any callback functions that haven't been called
  node_handle.spinOnce();
}

void calibrate() {
  Serial.println("Calibrating...");

  float sumX = 0.0;
  float sumY = 0.0;
  float sumZ = 0.0;
  float sumaccX = 0.0;
  float sumaccY = 0.0;
  float sumaccZ = 0.0;
  

  for (int i = 0; i < calibrationReadings; i++) {
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gyroX, gyroY, gyroZ);
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;
      delay(15);
    }
  }

  gyroXOffset = sumX / calibrationReadings;
  gyroYOffset = sumY / calibrationReadings;
  gyroZOffset = sumZ / calibrationReadings;

  Serial.println("Calibration complete.");
}
