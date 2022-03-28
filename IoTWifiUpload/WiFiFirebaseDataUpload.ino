/*
  Project: Send Data to Firebase Using Arduino Nano 33 IoT
  Board: Arduino Nano 33 IoT
   
  External libraries:
  - Arduino_LSM6DS3 by Arduino V1.0.0
  - Firebase Arduino based on WiFiNINA by Mobizt V1.1.4
 */

#include <Arduino_LSM6DS3.h>
#include <Firebase_Arduino_WiFiNINA.h>
#include <RTCZero.h>

#define FIREBASE_HOST "test2-529b3-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "upS4WguJxNFu8HU7RykiGGufdx44HIPbfV3Z8VC6"
#define WIFI_SSID "BELL038"
#define WIFI_PASSWORD "5ED12D1FD376"

FirebaseData firebaseData;
RTCZero rtc;

String path = "/IMU_LSM6DS3";
String jsonStr;
const byte seconds=0;
const byte minutes=0;
const byte hours=0;
const byte day=0;
float milli;

void setup()
{
  Serial.begin(9600);
  delay(1000);
  Serial.println();

  Serial.print("Initialize IMU sensor...");
  if (!IMU.begin()) {
    Serial.println(" failed!");
    while (1);
  }
  Serial.println(" done");
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  Serial.print("Connecting to WiFi...");
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print(".");
    delay(300);
  }
  Serial.print(" IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  rtc.begin();
  rtc.setHours(hours);
  rtc.setMinutes(minutes);
  rtc.setSeconds(seconds);
  milli=millis();
  
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, WIFI_SSID, WIFI_PASSWORD);
  Firebase.reconnectWiFi(true);

  
}

void loop()
{
  float Ax, Ay, Az, Gx,Gy,Gz, millisec, seconds, minutes, hours;

  // Read IMU acceleration data
  if (IMU.accelerationAvailable()& IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(Ax, Ay, Az);
    IMU.readGyroscope(Gx,Gy,Gz);
    millisec=millis()-milli;
    seconds=rtc.getSeconds();
    minutes=rtc.getMinutes();
    hours=rtc.getHours();
    
    // Send data to Firebase with specific path
    if (Firebase.setFloat(firebaseData, path + "/1-setFloat/Time(ms)", millisec)) {
      Serial.println(firebaseData.dataPath() + " = " + millisec);
    }
    if (Firebase.setFloat(firebaseData, path + "/1-setFloat/Time(s)", seconds)) {
      Serial.println(firebaseData.dataPath() + " = " + seconds);
    }
    if (Firebase.setFloat(firebaseData, path + "/1-setFloat/Time(min)", minutes)) {
      Serial.println(firebaseData.dataPath() + " = " + minutes);
    }
    if (Firebase.setFloat(firebaseData, path + "/1-setFloat/Time(hours)", hours)) {
      Serial.println(firebaseData.dataPath() + " = " + hours);
    }
    if (Firebase.setFloat(firebaseData, path + "/1-setFloat/Ax", Ax)) {
      Serial.println(firebaseData.dataPath() + " = " + Ax);
    }
    if (Firebase.setFloat(firebaseData, path + "/1-setFloat/Ay", Ay)) {
      Serial.println(firebaseData.dataPath() + " = " + Ay);
    }
    if (Firebase.setFloat(firebaseData, path + "/1-setFloat/Az", Az)) {
      Serial.println(firebaseData.dataPath() + " = " + Az);
    }
    if (Firebase.setFloat(firebaseData, path + "/1-setFloat/Gx", Gx)) {
      Serial.println(firebaseData.dataPath() + " = " + Gx);
    }
    if (Firebase.setFloat(firebaseData, path + "/1-setFloat/Gy", Gy)) {
      Serial.println(firebaseData.dataPath() + " = " + Gy);
    }
    if (Firebase.setFloat(firebaseData, path + "/1-setFloat/Gz", Gz)) {
      Serial.println(firebaseData.dataPath() + " = " + Gz);
    }

    // Push data using pushJSON
    jsonStr = "{\"Time(ms)\":" +String(millisec,6)+ ",\"Time(s)\":" + String(seconds,6) + ",\"Time(min)\":" + String(minutes,6) + ",\"Time(hours)\":" + String(hours,6) + ",\"Ax\":" + String(Ax,6) + ",\"Ay\":" + String(Ay,6) + ",\"Az\":" + String(Az,6) + ",\"Gx\":" + String(Gx,6) + ",\"Gy\":" + String(Gy,6) + ",\"Gz\":" + String(Gz,6) +"}";

    if (Firebase.pushJSON(firebaseData, path + "/2-pushJSON", jsonStr)) {
      Serial.println(firebaseData.dataPath() + " = " + firebaseData.pushName());
    }
    else {
      Serial.println("Error: " + firebaseData.errorReason());
    }

    Serial.println();
    delay(200);
  }
}
