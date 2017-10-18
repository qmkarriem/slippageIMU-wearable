/*****************************************************************
photonIMU.ino
Quran Karriem, October 2017
https://github.com/sparkfun/SparkFun_LSM9DS1_Particle_Library

Sends LSM9DS1 IMU data over UDP. Requires SFE_LSM9DS1 library by Jim Lindblom

If you're using a breakout, the pin-out is as follows:
	LSM9DS1 --------- Photon
	 SCL -------------- D1 (SCL)
	 SDA -------------- D0 (SDA)
	 VDD ------------- 3.3V
	 GND ------------- GND
(CSG, CSXM, SDOG, and SDOXM should all be pulled high.
Jumpers on the breakout board will do this for you.)

D
*****************************************************************/
#include "SparkFunLSM9DS1.h"
#include "math.h"


LSM9DS1 imu;

#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
// Calculate magnetic field at your location: http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -9.1 // Declination (degrees) in Durham, NC, October 2017.
unsigned int localPort = 8888;
int remotePort = 8888;
UDP udp;
const size_t bufferSize = 16; // Make this bigger if you have more data!
unsigned char buffer[bufferSize];
IPAddress remoteIP(152,3,43,196);
void setup()
{
  Serial.begin(115200);
  udp.begin(0);
  Serial.println(WiFi.localIP()); // Print your device IP Address via serial
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
}

void loop()
{
  printGyro();  // Print "G: gx, gy, gz"
  printAccel(); // Print "A: ax, ay, az"
  printMag();   // Print "M: mx, my, mz"

  // Print the heading and orientation for fun!
  // Call print attitude. The LSM9DS1's magnetometer x and y
  // axes are opposite to the accelerometer, so my and mx are
  // substituted for each other.
  printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
  Serial.println();
  delay(PRINT_SPEED);
}

void printGyro()
{
  imu.readGyro();
  int value = imu.calcGyro(imu.gx);
  buffer[0] = value >> 8;
  buffer[1] = (value & 0xff);
  if (udp.sendPacket(buffer, bufferSize, remoteIP, remotePort) >= 0) {
    // Success
    #ifdef SERIAL_DEBUG
      Serial.printlnf("%d", value);
    #endif
  }
  else {
    #ifdef SERIAL_DEBUG
      Serial.printlnf("send failed");
    #endif
            // On error, wait a moment, then reinitialize UDP and try again.
    delay(1000);
    udp.begin(0);
  }
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(" ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(" ");
  Serial.println(imu.calcGyro(imu.gz), 2);
  //Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printAccel()
{
  imu.readAccel();
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(" ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(" ");
  Serial.println(imu.calcAccel(imu.az), 2);
  //Serial.println(" g");
#elif defined PRINT_RAW
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}

void printMag()
{
  imu.readMag();

  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(" ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(" ");
  Serial.println(imu.calcMag(imu.mz), 2);
  //Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(
float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * M_PI / 180;

  if (heading > M_PI) heading -= (2 * M_PI);
  else if (heading < -M_PI) heading += (2 * M_PI);
  else if (heading < 0) heading += 2 * M_PI;

  // Convert everything from radians to degrees:
  heading *= 180.0 / M_PI;
  pitch *= 180.0 / M_PI;
  roll  *= 180.0 / M_PI;

  Serial.print("Pitch: "); Serial.println(pitch, 2);
  Serial.print("Roll: "); Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}
