#include <SPI.h>
#include <Arduino_LSM6DS3.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
//#include <RTCZero.h>
#include <Firebase_Arduino_WiFiNINA.h>
#include "config.h"

#include <Adafruit_SleepyDog.h>

#include <ArduinoUniqueID.h>
#include <RTClib.h>

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

FirebaseData firebaseData;
//RTCZero zrtc;
RTC_DS3231 rtc;

const byte READ = 0b11111111;     // SCP1000's read command
const byte WRITE = 0b01111111;   // SCP1000's write command

#ifdef TESTING_CONNECTION
  const char firebase_host[] = FIREBASE_HOST;
  const char firebase_auth[] = FIREBASE_AUTH;
  const char wifi_ssid[] = WIFI_SSID;
  const char wifi_password[] = WIFI_PASSWORD;
#endif

String path = "/IMU_LSM6DS3";
String jsonStr;
const byte seconds=0;
const byte minutes=0;
const byte hours=0;

//const byte day=0;
const int GMT=-5;
float milli;
String nodeName;
String day,month,year;
String serialID="";


// power-saving measure testing
#define CALIBRATION_BUTTON              10
#define WAKEUP_BUTTON                   9
#define RTC_INTERRUPT                   8

/* IMU Data */
float ax, ay, az;
float gx, gy, gz;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Serial up");
  Wire.begin();
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  delay(10);

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");

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

  #ifdef TESTING_CONNECTION
  Serial.print("Connecting to WiFi...");
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    status = WiFi.begin(wifi_ssid, wifi_password);
    Serial.print(".");
    delay(300);
  }
  Serial.print(" IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  #endif


  rtc.begin();
  //rtc.setHours(hours);
  //rtc.setMinutes(minutes);
  //rtc.setSeconds(seconds);
  milli=millis();

  #ifdef TESTING_CONNECTION
  // Variable to represent epoch
  unsigned long epoch;
 
  // Variable for number of tries to NTP service
  int numberOfTries = 0, maxTries = 1000000000000;
  
   // Get epoch
  do {
    epoch = WiFi.getTime();
    numberOfTries++;
  }
 
  while ((epoch == 0));
 
  if (numberOfTries == maxTries) {
    Serial.print("NTP unreachable!!");
    while (1);
  }
 
  else {
    Serial.print("Epoch received: ");
    Serial.println(epoch);
    rtc.setEpoch(epoch);
    Serial.println();
  }
  #endif
  
  #ifdef TESTING_CONNECTION
    //Serial.println("step 1");
    Firebase.begin(firebase_host, firebase_auth, wifi_ssid, wifi_password);
    //Serial.println("step 2");
    Firebase.reconnectWiFi(true);
    //Serial.println("step 3");
  #endif
  
  #ifdef TESTING_POWER

    // initializing the rtc
    if(!rtc.begin()) {
        Serial.println("Couldn't find RTC!");
        Serial.flush();
        while (1) delay(10);
    }

    if(rtc.lostPower()) {
        // this will adjust to the date and time at compilation
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    //we don't need the 32K Pin, so disable it
    rtc.disable32K();
    
    pinMode( CALIBRATION_BUTTON, INPUT_PULLUP );
    attachInterrupt( digitalPinToInterrupt( CALIBRATION_BUTTON ), calibButtonHandler, RISING );
    pinMode( WAKEUP_BUTTON, INPUT_PULLUP);
    attachInterrupt( digitalPinToInterrupt( WAKEUP_BUTTON), wakeupButtonHandler, RISING);
    pinMode( RTC_INTERRUPT, INPUT_PULLUP);
    attachInterrupt( digitalPinToInterrupt( RTC_INTERRUPT), rtcButtonHandler, FALLING);

    rtc.clearAlarm(1);
    rtc.clearAlarm(2);
    
    // stop oscillating signals at SQW Pin
    // otherwise setAlarm1 will fail
    rtc.writeSqwPinMode(DS3231_OFF);

    // turn off alarm 2 (in case it isn't off already)
    // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
    rtc.disableAlarm(2);

    // schedule an alarm 10 seconds in the future
    if(!rtc.setAlarm1(
      rtc.now() + TimeSpan(10),
      DS3231_A1_Second // alarm sends a ping every minute
    )) {
        Serial.println("Error, alarm wasn't set!");
    }else {
        Serial.println("Alarm will happen in 10 seconds!");
    }
  #endif

  #ifdef TROUBLESHOOTING_TEXT
    Serial.print("Troubleshooting message mode is ON.");
  #else
    Serial.print("Troubleshooting message mode is OFF.");

  #endif

  //Serial.println("flag");

  //Getting serial ID in HEX
  UniqueIDdump(Serial);
  for(size_t i = 0; i < UniqueIDsize; i++){
    serialID=serialID+String((UniqueID[i]), HEX);
  }
  path=serialID;
  #ifdef TESTING_CONNECTION
  if (Firebase.setString(firebaseData, String("Devices") + "/" + path, path)) {
      Serial.println(firebaseData.dataPath() + " = " + path);
  }
  else {
      Serial.println("Error: " + firebaseData.errorReason());
    }
  #endif
}
 
void loop() {

  float millisec, seconds, minutes, hours;

   char date[10] = "hh:mm:ss";

  // resetting SQW and alarm 1 flag
  // using setAlarm1, the next alarm could now be configurated
  if(rtc.alarmFired(1)) {
    rtc.now().toString(date);
    Serial.print(date);
    rtc.clearAlarm(1);
    Serial.println(" Alarm cleared");
  }

  
  IMU.readAcceleration(ax,ay,az);
  IMU.readGyroscope(gx, gy, gz);
  millisec=millis()-milli;
  seconds=rtc.getSeconds();
  minutes=rtc.getMinutes();
  hours=rtc.getHours();
  day=String(rtc.getDay());
  month=String(rtc.getMonth());
  year=String(2000+rtc.getYear());
  
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

  #ifdef TESTING_CONNECTION
     /*Send data to firebase*/

    if (Firebase.setFloat(firebaseData, path + "/1-setDouble/roll", roll)) {
      Serial.println(firebaseData.dataPath() + " = " + roll);}

    if (Firebase.setFloat(firebaseData, path + "/1-setDouble/gyroAngleX", gyroXangle)) {
      Serial.println(firebaseData.dataPath() + " = " + gyroXangle);}
    if (Firebase.setFloat(firebaseData, path + "/1-setDouble/compAngleX", compAngleX)) {

      Serial.println(firebaseData.dataPath() + " = " + compAngleX);
    }*/
    
    if (Firebase.setFloat(firebaseData, path + "/1-setDouble/hour", hours)) {
      Serial.println(firebaseData.dataPath() + " = " + hours);
    }
    if (Firebase.setFloat(firebaseData, path + "/1-setDouble/minute", minutes)) {
      Serial.println(firebaseData.dataPath() + " = " + minutes);
    }
    if (Firebase.setFloat(firebaseData, path + "/1-setDouble/seconds", seconds)) {
      Serial.println(firebaseData.dataPath() + " = " + seconds);
    }
    if (Firebase.setFloat(firebaseData, path + "/1-setDouble/milliseconds", millisec)) {
      Serial.println(firebaseData.dataPath() + " = " + millisec);
    }
    if (Firebase.setFloat(firebaseData, path + "/1-setDouble/kalmanAngleX", kalAngleX)) {
      Serial.println(firebaseData.dataPath() + " = " + kalAngleX);
    }
    /*if (Firebase.setFloat(firebaseData, path + "/1-setDouble/pitch", pitch)) {
      Serial.println(firebaseData.dataPath() + " = " + pitch);
    }

    if (Firebase.setFloat(firebaseData, path + "/1-setDouble/gyroAngleY", gyroYangle)) {
      Serial.println(firebaseData.dataPath() + " = " + gyroYangle);}
    if (Firebase.setFloat(firebaseData, path + "/1-setDouble/compAngleY", compAngleY)) {

      Serial.println(firebaseData.dataPath() + " = " + compAngleY);
    }*/

    if (Firebase.setFloat(firebaseData, path + "/1-setDouble/kalmanAngleY", kalAngleY)) {
      Serial.println(firebaseData.dataPath() + " = " + kalAngleY);}
    if (Firebase.setFloat(firebaseData, path + "/1-setDouble/timeOfData", timer)) {

      Serial.println(firebaseData.dataPath() + " = " + timer);} 
  

    //Set up the JSON string to push to firebase.
    //jsonStr= "{\"Roll(angles)\":"+String(roll, 6)+",\"gyroAngleX\":" + String(gyroXangle,6) +",\"compAngleX\":" + String(compAngleX,6) +",\"kalAngleX\":" + String(kalAngleX,6) +
   // ",\"Pitch\":" + String(pitch,6) +",\"gyroAngleY\":" + String(gyroYangle,6) +",\"compAngleY\":" + String(compAngleY,6) +",\"kalAngleY\":" + String(kalAngleY,6) +"}";
   
    jsonStr= "{\"kalAngleX\":" + String(kalAngleX,6) + ",\"kalAngleY\":" + String(kalAngleY,6) +",\"Hours\":" + String(hours,6) +",\"Minutes\":" + String(minutes,6) +",\"Seconds\":" + String(seconds,6) +
    ",\"Milliseconds\":" + String(millisec,6) +"}";
    
    if (Firebase.pushJSON(firebaseData, path + "/"+year+"-"+month+"-"+day, jsonStr)) {
      Serial.println(firebaseData.dataPath() + " = " + firebaseData.pushName());
    }
     
    else {
      Serial.println("Error: " + firebaseData.errorReason());
    }
  #endif
    

  // raw data (virtually useless for our purposes)
  #if 0 // Set to 1 to activate
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
  
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");
  
    Serial.print("\t");
  #endif

  #ifdef TESTING_KALMAN
    // calculated data 
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
    delay(2);
  #endif

  #ifdef TESTING_POWER
    int sleepMS = Watchdog.sleep();
  #endif
}

void calibButtonHandler( void )
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200) 
  {

  // sanity message
  #ifdef TROUBLESHOOTING_TEXT
    Serial.println( "Calibration button pushed" ); // probably need to debounce eventually 
  #endif

    #ifdef RESTRICT_PITCH // Eq. 25 and 26
      double roll  = atan2(ay, az) * RAD_TO_DEG;
      double pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    #else // Eq. 28 and 29
      double roll  = atan(ay / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
      double pitch = atan2(-ax, az) * RAD_TO_DEG;
    #endif
    
    // actually performing the calibration 
    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;
  #endif
}


    #ifdef TESTING_POWER
      Serial.println("WOAH BUTTON PUSHED");
      kalmanX.setAngle(roll); // Set starting angle
      kalmanY.setAngle(pitch);
      gyroXangle = roll;
      gyroYangle = pitch;
      compAngleX = roll;
      compAngleY = pitch;
    #endif
  }
  last_interrupt_time = interrupt_time;
  /*Serial.println( "WHOA HEY BUTTON PUSHED" ); // probably need to debounce eventually 
    #ifdef RESTRICT_PITCH // Eq. 25 and 26
      double roll  = atan2(ay, az) * RAD_TO_DEG;
      double pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    #else // Eq. 28 and 29
      double roll  = atan(ay / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
      double pitch = atan2(-ax, az) * RAD_TO_DEG;
    #endif

    #ifdef TESTING_POWER
      kalmanX.setAngle(roll); // Set starting angle
      kalmanY.setAngle(pitch);
      gyroXangle = roll;
      gyroYangle = pitch;
      compAngleX = roll;
      compAngleY = pitch;
    #endif*/

void wakeupButtonHandler( void )
{
  #ifdef TROUBLESHOOTING_TEXT
    Serial.println( "Wakeup button pushed" ); // probably need to debounce eventually 
  #endif

  #ifdef TESTING_POWER
    //
  #endif
}

void rtcButtonHandler( void )
{
  
  #ifdef TROUBLESHOOTING_TEXT
    Serial.print( "RTC Activated ");
  #endif
  Serial.print("\n"); // probably need to debounce eventually 
  #ifdef TESTING_POWER
    //
  #endif

}
