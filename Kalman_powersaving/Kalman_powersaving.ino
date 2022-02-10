#include <SPI.h>
#include <Arduino_LSM6DS3.h>
#include <Wire.h>

#include <Arduino_LSM6DS3.h>
#include <Firebase_Arduino_WiFiNINA.h>
#include <RTCZero.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

RTCZero rtc;

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

const byte READ = 0b11111111;     // SCP1000's read command
const byte WRITE = 0b01111111;   // SCP1000's write command

/* IMU Data */
float ax, ay, az;
float gx, gy, gz;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

String path = "/IMU_LSM6DS3_kalman";
String jsonStr;
const byte seconds=0;
const byte minutes=0;
const byte hours=0;
const byte day=0;
float milli;

// power-saving measure testing
#define BUTTON_PIN              10
#define LED_PIN                 LED_BUILTIN

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);
  Wire.begin();
  Serial.println();

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }
  delay(10);

  Serial.print("Sensor sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();

  rtc.begin();
  rtc.setHours(hours);
  rtc.setMinutes(minutes);
  rtc.setSeconds(seconds);
  milli=millis();

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(ay, az) * RAD_TO_DEG;
    double pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(ay / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
    double pitch = atan2(-ax, az) * RAD_TO_DEG;
  #endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

  
// power-saving measure testing
  pinMode( BUTTON_PIN, INPUT_PULLUP );
  attachInterrupt( digitalPinToInterrupt( BUTTON_PIN ), buttonHandler, CHANGE );
}

void loop() {
  float millisec, seconds, minutes, hours;
  IMU.readAcceleration(ax,ay,az);
  IMU.readGyroscope(gx, gy, gz);

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(ay, az) * RAD_TO_DEG;
    double pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(ay / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
    double pitch = atan2(-ax, az) * RAD_TO_DEG;
  #endif

  double gyroXrate = gx / 131.0; // Convert to deg/s
  double gyroYrate = gy / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");

  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.print(gz); Serial.print("\t");

  Serial.print("\t");
#endif

#if 1 // Set to 1 to activate
  Serial.print(roll); Serial.print("\t");
  Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");


  Serial.print("\r\n");
#endif


    millisec=millis()-milli;
    seconds=rtc.getSeconds();
    minutes=rtc.getMinutes();
    hours=rtc.getHours();
    
    delay(500);
}

// powersaving measures
void buttonHandler( void )
{
  Serial.println( "WHOA HEY BUTTON PUSHED" ); // probably need to debounce eventually 
    #ifdef RESTRICT_PITCH // Eq. 25 and 26
      double roll  = atan2(ay, az) * RAD_TO_DEG;
      double pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    #else // Eq. 28 and 29
      double roll  = atan(ay / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
      double pitch = atan2(-ax, az) * RAD_TO_DEG;
    #endif

    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;
}