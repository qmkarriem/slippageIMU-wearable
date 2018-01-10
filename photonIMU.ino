/*****************************************************************
photonIMU.ino
Quran Karriem, October 2017


Sends LSM9DS1 IMU data over UDP. Requires SFE_LSM9DS1 library by Jim Lindblom
https://github.com/sparkfun/SparkFun_LSM9DS1_Particle_Library

*****************************************************************/
#include "SparkFunLSM9DS1.h"
#include "math.h"

LSM9DS1 imu;

#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

#define PRINT_CALCULATED
#define PRINT_SPEED 100 // 50 ms between prints (experienced occasional dropouts at 25 ms)
#define DECLINATION -9.1 // Declination (degrees) in Durham, NC, October 2017. Calculated via http://www.ngdc.noaa.gov/geomag-web/#declination

unsigned int localPort = 8888;
int remotePort = 8888;
UDP udp;
const size_t bufferSize = 128; // Make this bigger if you have more data!
char buffer[bufferSize];
char IPString[40];
IPAddress remoteIP(192,168,0,10);

// receive a new port from Particle cloud Terminal
void updateRemotePort(const char *event, const char *data) {
  remotePort = atoi(data);
  Particle.publish("remotePortCallback", remotePort);
}

//receive a new IP address from Particle cloud Terminal
void updateRemoteIP(const char *event, const char *data) {
  unsigned char IPHandler[4] = {0}; //need to parse into . separated values
  size_t index = 0;
  while (*data){
    if (isdigit((unsigned char)*data)){
      IPHandler[index] *= 10;
      IPHandler[index] += *data - '0';
    } else {
      index++;
    }
    data++;
  }
  sprintf(IPString, "%i, %i, %i, %i", IPHandler[0], IPHandler[1], IPHandler[2], IPHandler[3]);
  remoteIP = IPHandler;
  Particle.publish("remoteIPCallback", IPString);
  Particle.publish("remoteIPCallback", String(remoteIP));
}

void setup(){
  Particle.subscribe("getRemoteIP-SLIPPAGE5", updateRemoteIP);
  Particle.subscribe("getRemotePort-SLIPPAGE5", updateRemotePort);
  Serial.begin(115200);
  udp.begin(0);
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  imu.begin();
}

void loop(){
  printGyro();  // Print "G: gx, gy, gz"
  printAccel(); // Print "A: ax, ay, az"
  printMag();   // Print "M: mx, my, mz"
  printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
  delay(PRINT_SPEED);
}

void printGyro(){
  imu.readGyro();
  int ret = snprintf(buffer, bufferSize, "G: %f %f %f", imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz));

  if (udp.sendPacket(buffer, bufferSize, remoteIP, remotePort) >= 0) {
    // Success
    #ifdef SERIAL_DEBUG
      Serial.printlnf("%d", buffer);
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

void printAccel(){
  imu.readAccel();
  int ret = snprintf(buffer, bufferSize, "A: %f %f %f", imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az));

  if (udp.sendPacket(buffer, bufferSize, remoteIP, remotePort) >= 0) {
    // Success
    #ifdef SERIAL_DEBUG
      Serial.printlnf("%d", buffer);
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

void printMag(){
  imu.readMag();
  int ret = snprintf(buffer, bufferSize, "M: %f %f %f", imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));

  if (udp.sendPacket(buffer, bufferSize, remoteIP, remotePort) >= 0) {
    // Success
    #ifdef SERIAL_DEBUG
      Serial.printlnf("%d", buffer);
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
void printAttitude(float ax, float ay, float az, float mx, float my, float mz){
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
  int ret = snprintf(buffer, bufferSize, "Pitch: %f", pitch);

  if (udp.sendPacket(buffer, bufferSize, remoteIP, remotePort) >= 0) {
    // Success
    #ifdef SERIAL_DEBUG
      Serial.printlnf("%d", buffer);
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
  ret = snprintf(buffer, bufferSize, "Roll: %f", roll);

  if (udp.sendPacket(buffer, bufferSize, remoteIP, remotePort) >= 0) {
    // Success
    #ifdef SERIAL_DEBUG
      Serial.printlnf("%d", buffer);
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
  ret = snprintf(buffer, bufferSize, "Heading: %f", heading);

  if (udp.sendPacket(buffer, bufferSize, remoteIP, remotePort) >= 0) {
    // Success
    #ifdef SERIAL_DEBUG
      Serial.printlnf("%d", buffer);
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
  Serial.print("Pitch: "); Serial.println(pitch, 2);
  Serial.print("Roll: "); Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}
