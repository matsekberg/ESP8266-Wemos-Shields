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
#define CONFIG_VERSION "WESH002"

#include "sensorlibs.h"
#include <ArduinoOTA.h>

#define LONG_PRESS_MS 1000
#define SHORT_PRESS_MS 100
#define CONFIG_WIFI_PRESS_MS 5000
#define CONFIG_TOUCHES_COUNT 3
#define MQTT_CHECK_MS 15000

#define OTA_PASS "UPDATE_PW"
#define OTA_PORT 8266

#define BUTTON_PIN  0  // GPIO0, pin 18, D3
#define LED_PIN     2  // GPIO2, pin 17, D4
#define RELAY_PIN   5  // GPIO5, pin 20, D1 (SCL)
#define LED2812_PIN 4  // GPIO4, pin 19, D2 (SDA)

// support stuff
#include "support/includes.h"
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long uptime = 0;
#include "topics.h"
#include "support/mqtt-support.cpp"
#include "support/wifi-manager.cpp"


// Wemos Matrix
//   shield: https://wiki.wemos.cc/products:d1_mini_shields:matrix_led_shield
//   library: https://github.com/wemos/WEMOS_Matrix_LED_Shield_Arduino_Library
// Wemos OLED shield: https://wiki.wemos.cc/products:d1_mini_shields:oled_shield



volatile int desiredRelayState = 0;
volatile int relayState = 0;
volatile unsigned long millisSinceChange = 0;
volatile int noOfConfigTouches = 0;

volatile boolean sendGroupEventTopic = false;
volatile boolean configWifi = false;
volatile boolean sendEvent = true;
boolean sendStatus = true;
boolean sendPong = false;


// LED Matrix
MLED* matrixLED = NULL;
boolean matrixLEDInit = false;
#define MATRIX_MAXLEN 64*2
boolean updateMatrix = false;
char matrixData[MATRIX_MAXLEN];
uint8_t matrixLen = 0;

// DHT stuff
#define DHTPIN 14

//#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTTYPE DHT21   // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);
float humid = NAN;
float temp = NAN;
boolean sendSensors = false;

// ADXL345 stuff
Adafruit_ADXL345_Unified* accel = NULL;


unsigned long lastMQTTCheck = -MQTT_CHECK_MS; //This will force an immediate check on init.
bool printedWifiToSerial = false;

//
// MQTT message arrived, decode
// Ok payload: 1/on, 0/off, X/toggle, S/status
//
void MQTTcallback(char* topic, byte* payload, unsigned int length) {
  Serial.print(F("MQTT sub: "));
  Serial.println(topic);

  // Relay actions
  if (!strcmp(topic, actionSTopic.c_str()) || !strcmp(topic, groupActionSTopic.c_str()))
  {
    if ((char)payload[0] == '1' || ! strncasecmp_P((char *)payload, "on", length))
    {
      desiredRelayState = 1;
    }
    else if ((char)payload[0] == '0' || ! strncasecmp_P((char *)payload, "off", length))
    {
      desiredRelayState = 0;
    }
    else if ((char)payload[0] == 'X' || ! strncasecmp_P((char *)payload, "toggle", length))
    {
      desiredRelayState = !desiredRelayState;
    }
    else if ((char)payload[0] == 'S' || ! strncasecmp_P((char *)payload, "status", length))
    {
      sendStatus = true;
    }
  }

  // LED matrix actions
  if (!strcmp(topic, matrixActionSTopic.c_str()) && (length % 2 == 0) && (length <= MATRIX_MAXLEN))
  {
    if (length > 0)
    {
      strncpy((char*)matrixData, (const char*)payload, length);
    }
    matrixLen = length;
    updateMatrix = true;
  }

   // ADXL345 actions
   
  // Ping action
  if (!strcmp(topic, pingSTopic.c_str()))
  {
    sendPong = true;
  }
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

  // Relay state is updated via the interrupt *OR* the MQTT callback.
  if (relayState != desiredRelayState) {
    Serial.print(F("Chg state to "));
    Serial.println(desiredRelayState);

    digitalWrite(RELAY_PIN, desiredRelayState);
    relayState = desiredRelayState;
    sendStatus = true;
  }

  // Update LED matrix
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
  }

   // ADXL345
   if (1==2) 
   {
      if (accel345 == NULL)
      {
         accel345 = new Adafruit_ADXL345_Unified(12345);
         if(!accel.begin())
             /* There was a problem detecting the ADXL345 ... check your connections */
             accel345 = NULL;
            Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
          } 
           else
          {
              /* Set the range to whatever is appropriate for your project */
              accel345.setRange(ADXL345_RANGE_16_G);
              // displaySetRange(ADXL345_RANGE_8_G);
              // displaySetRange(ADXL345_RANGE_4_G);
              // displaySetRange(ADXL345_RANGE_2_G);
          }
      }
      else
      {
         /* Get a new sensor event */ 
         sensors_event_t event; 
         accel345.getEvent(&event);
 
         /* Display the results (acceleration is measured in m/s^2) */
         Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
         Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
         Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
      }
   }
   
  if (sendPong)
  {
    Serial.print(F("MQTT pub: "));
    String meta = getDeviceMeta(CONFIG_VERSION);
    Serial.print(meta);
    Serial.print(F(" to "));
    Serial.println(pongMetaTopic);
    client.publish(pongMetaTopic.c_str(), meta.c_str());
    sendPong = false;
  }

  // publish event if touched
  if (sendEvent) {
    const char* payload = (relayState == 0) ? "0" : "1";
    Serial.print(F("MQTT pub: "));
    Serial.print(payload);
    Serial.print(F(" to "));
    if (sendGroupEventTopic) {
      Serial.println(groupEventTopic);
      client.publish(groupEventTopic.c_str(), payload);
    } else {
      Serial.println(eventTopic);
      client.publish(eventTopic.c_str(), payload);
    }
    sendEvent = false;
  }

  // publish state when requested to do so
  if (sendStatus) {
    const char* payload = (relayState == 0) ? "0" : "1";
    Serial.print(F("MQTT pub: "));
    Serial.print(payload);
    Serial.print(F(" to "));
    Serial.println(statusTopic);
    client.publish(statusTopic.c_str(), payload);
    sendStatus = false;
  }

  if (sendSensors)
  {
    if (!isnan(temp))
    {
      Serial.print(F("MQTT pub: "));
      Serial.print(temp);
      Serial.print(F(" to "));
      Serial.println(sensorTempTopic);
      client.publish(sensorTempTopic.c_str(), String(temp).c_str());
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
      client.publish(sensorHumidTopic.c_str(), String(humid).c_str());
    }
    else
    {
      Serial.println(F("No humidity data"));
    }
    sendSensors = false;
  }
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

  initWifiManager(CONFIG_VERSION, false);

  // after wifi and parameters are configured, create publish topics
  eventTopic = String(F("event/")) + custom_unit_id.getValue() + String(F("/switch"));
  groupEventTopic = String(F("event/")) + custom_group_id.getValue() + String(F("/switch"));
  statusTopic = String(F("status/")) + custom_unit_id.getValue() + String(F("/relay"));
  sensorTempTopic = String(F("sensor/")) + custom_unit_id.getValue() + String(F("/temp"));
  sensorHumidTopic = String(F("sensor/")) + custom_unit_id.getValue() + String(F("/humid"));
  pongStatusTopic = String(F("pong/")) + custom_unit_id.getValue() + String(F("/status"));
  pongMetaTopic = String(F("pong/")) + custom_unit_id.getValue() + String(F("/meta"));
  // and subscribe topic
  actionSTopic = String(F("action/")) + custom_unit_id.getValue() + String(F("/relay"));
  groupActionSTopic = String(F("action/")) + custom_group_id.getValue() + String(F("/relay"));
  pingSTopic = String(F("ping/nodes"));
  matrixActionSTopic = String(F("action/")) + custom_unit_id.getValue() + String(F("/matrix"));
  accelActionSTopic = String(F("action/")) + custom_unit_id.getValue() + String(F("/accel"));
  // pointer of topics
  subscribedTopics[0] = &pingSTopic;
  subscribedTopics[1] = &actionSTopic;
  subscribedTopics[2] = &groupActionSTopic;
  subscribedTopics[3] = &matrixActionSTopic;
  subscribedTopics[4] = &accelActionSTopic;
  noSubscribedTopics = 5;

  client.setServer(custom_mqtt_server.getValue(), atoi(custom_mqtt_port.getValue()));
  client.setCallback(MQTTcallback);

  // OTA setup
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setHostname(custom_unit_id.getValue());
  ArduinoOTA.setPassword(OTA_PASS);
  ArduinoOTA.begin();

  dht.begin();

  // Enable interrupt for button press
  Serial.println(F("Enabling touch switch interrupt"));
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonChangeCallback, CHANGE);
}


//
////////// LOOP //////////
//
void loop() {
  // If we haven't printed WiFi details to Serial port yet, and WiFi now connected,
  // do so now. (just the once)
  if (!printedWifiToSerial && WiFi.status() == WL_CONNECTED) {
    Serial.println(F("WiFi connected"));
    Serial.println(F("IP address: "));
    Serial.println(WiFi.localIP());
    printedWifiToSerial = true;
  }

  // Check MQTT connection
  if (millis() - lastMQTTCheck >= MQTT_CHECK_MS) {
    uptime += MQTT_CHECK_MS / 1000;
    checkMQTTConnection();
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    humid = dht.readHumidity();
    // Read temperature as Celsius (the default)
    temp = dht.readTemperature();
    lastMQTTCheck = millis();
    sendSensors = true;
  }

  // Handle any pending MQTT messages
  client.loop();

  // Handle any pending OTA SW updates
  ArduinoOTA.handle();

  // Handle any state change and MQTT publishing
  handleStatusChange();

  // Handle looong touch to reconfigure all parameters
  if (configWifi) {
    espClient.stop();
    delay(1000);
    initWifiManager(CONFIG_VERSION, true);
  }

  delay(50);
}
