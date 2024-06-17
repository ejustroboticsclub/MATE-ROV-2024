

void resetsensor() //this function keeps the sketch a little shorter
{
  SPI.setDataMode(SPI_MODE0);
  SPI.transfer(0x15);
  SPI.transfer(0x55);
  SPI.transfer(0x40);
}


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
  const long TEMP = 200 + ((dT * (c6 + 50)) / 1024);
  const long OFF = (c2 * 4) + (((c4 - 512) * dT) / 4096);
  const long SENS = c1 + ((c3 * dT) / 1024) + 24576;
  PCOMP = ((((SENS * (D1 - 7168)) / 16384) - OFF) / 32) + 250;
  float TEMPREAL = TEMP / 10;
  // Density of water (kg/m^3)
  const float density_water = 1000.0;
  // Acceleration due to gravity (m/s^2)
  const float gravity = 9.81;

  // Calculate depth underwater in meters
  float depth_meters = (PCOMP - atmospheric_pressure) / (density_water * gravity);
  // Convert depth from meters to centimeters
  depth_cm = depth_meters * 10000.0 - 1 ;

  return  depth_cm ;
}

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
