#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   
   Connections (For default I2C)
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 5V DC
   Connect GROUND to common ground

   History
   =======
   2014/JULY/25  - First version (KTOWN)
*/
   
/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

/* Or, use Hardware SPI:
  SCK -> SPI CLK
  SDA -> SPI MOSI
  G_SDO + XM_SDO -> tied together to SPI MISO
  then select any two pins for the two CS lines:
*/

#define LSM9DS0_XM_CS 2
#define LSM9DS0_GYRO_CS 3
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

/* Or, use Software SPI:
  G_SDO + XM_SDO -> tied together to the MISO pin!
  then select any pins for the SPI lines, and the two CS pins above
*/

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_SCLK, LSM9DS0_MISO, LSM9DS0_MOSI, LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t accel, mag, gyro, temp;
  
  lsm.getSensor(&accel, &mag, &gyro, &temp);
  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");
  
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  Serial.println("");
}

void loop(void) 
{  
  /* Get a new sensor event */ 
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  
  // print out gyroscopic data
  Serial1.print("G: "); Serial1.print(gyro.gyro.x); Serial1.print(" ");
  Serial1.print(gyro.gyro.y); Serial1.print(" ");
  Serial1.print(gyro.gyro.z); Serial1.println(" ");

  Serial.print("G: "); Serial.print(gyro.gyro.x); Serial.print(" ");
  Serial.print(gyro.gyro.y); Serial.print(" ");
  Serial.print(gyro.gyro.z); Serial.println(" ");
  
  // print out accelleration data
  Serial1.print("A: "); Serial1.print(accel.acceleration.x); Serial1.print(" ");
  Serial1.print(accel.acceleration.y); Serial1.print(" ");
  Serial1.print(accel.acceleration.z); Serial1.println(" ");

  Serial.print("A: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  Serial.print(accel.acceleration.y); Serial.print(" ");
  Serial.print(accel.acceleration.z); Serial.println(" ");

  // print out magnetometer data
  Serial1.print("M: "); Serial1.print(mag.magnetic.x); Serial1.print(" ");
  Serial1.print(mag.magnetic.y); Serial1.print(" ");
  Serial1.print(mag.magnetic.z); Serial1.println(" ");

  Serial.print("M: "); Serial.print(mag.magnetic.x); Serial.print(" ");
  Serial.print(mag.magnetic.y); Serial.print(" ");
  Serial.print(mag.magnetic.z); Serial.println(" ");

  if (Serial.available()) //USB
  { // If data comes in from serial monitor, send it out to XBee
  Serial1.write(Serial.read()); //XBee/UART1/pins 0 and 1

  }
  if (Serial1.available())   //XBee/UART1/pins 0 and 1
  { // If data comes in from XBee, send it out to serial monitor
    Serial.write(Serial1.read());  //Serial port
  }
  
  delay(50);
}
