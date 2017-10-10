/*
   ESP8266-Wemos-Shields
   Firmware for a Wemos d1 mini device using Wemos shields with MQTT and WiFi capabilities.

   Supports OTA update
   Mats Ekberg (C) 2017 GNU GPL v3

   Supports sensors (has to be recompiled if change):


   Runs on this harware:
   https://wiki.wemos.cc/products:d1:d1_mini

   Uses these libraries:


   Flashed via USB/OTA in Arduino IDE with these parameters:
   Board:       Generic ESP8285 Module
   Flash size:  1M (64K SPIFFS)

*/

// DO EDIT
#define CONFIG_VERSION "WESH002"
// END - DO EDIT


// DO NOT CHANGE
#include "sensorlibs.h"
#include "support/wifi-manager.h"
#include "support/mqtt-support.h"

#include "topics.h"
#include "support/wifi-manager.cpp"
#include "support/mqtt-support.cpp"
// END - DO NOT CHANGE


// DEFINE TOPICS HERE
String sensorTopic;         // published with sensordata
//String accelActionSTopic;   // published when the switch is touched


// LED2812
#define LED2812_PIN 4  // GPIO4, pin 19, D2 (SDA)

/* LED Matrix
  MLED* matrixLED = NULL;
  boolean matrixLEDInit = false;
  #define MATRIX_MAXLEN 64*2
  boolean updateMatrix = false;
  char matrixData[MATRIX_MAXLEN];
  uint8_t matrixLen = 0;
*/

// DHT stuff
#define DHTPIN 14
//#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTTYPE DHT21   // DHT 21 (AM2301)
DHT dht(DHTPIN, DHTTYPE);
float humid = NAN;
float temp = NAN;


//
// MQTT message arrived, decode
//
void mqttCallbackHandle(char* topic, byte* payload, unsigned int length) {
  Serial.print(F("MQTT sub: "));
  Serial.println(topic);
}

//
// Handle short touch
//
void shortPress() {
  desiredRelayState = !desiredRelayState; //Toggle relay state.
  sendGroupEventTopic = false;
  sendEvent = true;
  noOfConfigTouches = 0;
}

//
// Handle long touch
//
void longPress() {
  desiredRelayState = !desiredRelayState; //Toggle relay state.
  sendGroupEventTopic = true;
  sendEvent = true;
  noOfConfigTouches = 0;
}

//
// Handle looong config touch
//
void configWifiPress() {
  noOfConfigTouches++;
  if (noOfConfigTouches >= CONFIG_TOUCHES_COUNT)
    configWifi = true;
}


//
// This is executed on touch
//
void buttonChangeCallback() {
  if (digitalRead(0) == 1) {

    // Button has been released, trigger one of the two possible options.
    if (millis() - millisSinceChange > CONFIG_WIFI_PRESS_MS) {
      configWifiPress();
    }
    else if (millis() - millisSinceChange > LONG_PRESS_MS) {
      longPress();
    }
    else if (millis() - millisSinceChange > SHORT_PRESS_MS) {
      shortPress();
    }
    else {
      //Too short to register as a press
    }
  }
  else {
    //Just been pressed - do nothing until released.
  }
  millisSinceChange = millis();
}


//
// This routine handles state changes and MQTT publishing
//
void handleStatusChange() {

  /* Update LED matrix
    if (updateMatrix)
    {
    if (matrixLED == NULL)
    {
      Serial.println(F("Init matrix"));
      matrixLED = new MLED(5); //set intensity=5
      //delay(10);
      matrixLED->clear();
    }
    if (matrixLen == 0)
    {
      Serial.println(F("Clear matrix"));
      matrixLED->clear();
    }
    else
    {
      Serial.print(F("Matrix len: "));
      Serial.println(matrixLen);

      matrixLED->clear();
      for (uint8_t p = 0; p < matrixLen; p += 2)
      {
        uint8_t x = matrixData[p] & 7;
        uint8_t y = matrixData[p + 1] & 7;
        Serial.print(F("Set matrix: "));
        Serial.print(x);
        Serial.print(F(","));
        Serial.println(y);
        matrixLED->dot(x, y);
        delay(1);
      }
    }
    matrixLED->display();
    // it only works a few seconds is using the same instance all the time :(
    matrixLED = NULL;
    updateMatrix = false;
    } */

  // publish relay state, pong, event and status messages
  mqttPublish();

  if (sendSensors)
  {
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();

    if (!isnan(temp))
    {
      Serial.print(F("MQTT pub: "));
      Serial.print(temp);
      Serial.print(F(" to "));
      Serial.println(sensorTempTopic);
      json["temp"] = String(temp).c_str();
    }
    else
    {
      Serial.println(F("No temp data"));
    }
    if (!isnan(humid))
    {
      Serial.print(F("MQTT pub: "));
      Serial.print(humid);
      Serial.print(F(" to "));
      Serial.println(sensorHumidTopic);
      json["humid"] = String(humid).c_str();
    }
    else
    {
      Serial.println(F("No humidity data"));
    }
    String jsonStr;
    json.printTo(jsonStr);
    client.publish(sensorTopic.c_str(), jsonStr.c_str());
    sendSensors = false;
  }
}


//
// callback to create custom topics
//
void mqttCallbackCreateTopics() {
  sensorTopic = String(F("sensor/")) + custom_unit_id.getValue() + String(F("/value"));
  //matrixActionSTopic = String(F("action/")) + custom_unit_id.getValue() + String(F("/matrix"));
  //accelActionSTopic = String(F("action/")) + custom_unit_id.getValue() + String(F("/accel"));
  // pointer of topics

  //subscribedTopics[0] = &matrixActionSTopic;
  //subscribedTopics[0] = &accelActionSTopic;
  noSubscribedTopics = 0;
}


//
////////// SETUP //////////
//
void setup() {
  Serial.begin(115200);
  Serial.println(F("Initialising"));
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); //LED off.

  // setup wifi
  wifiSetup(CONFIG_VERSION, false);

  // setup mqtt
  mqttSetup();

  dht.begin();

  // Enable interrupt for button press
  Serial.println(F("Enabling touch switch interrupt"));
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonChangeCallback, CHANGE);
}


//
////////// LOOP //////////
//
void loop() {

  // handle wifi
  wifiLoop();

  // handle mqtt
  mqttLoop();

  // Check MQTT connection
  if (millis() - lastMQTTCheck >= MQTT_CHECK_MS) {
    uptime += MQTT_CHECK_MS / 1000;
    mqttCheckConnection();
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    humid = dht.readHumidity();
    // Read temperature as Celsius (the default)
    temp = dht.readTemperature();
    lastMQTTCheck = millis();
    sendSensors = true;
  }



  // Handle any state change and MQTT publishing
  handleStatusChange();

  delay(50);
}
