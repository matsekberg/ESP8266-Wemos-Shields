#include <WiFiManager.h>

////// config values
//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40] = "10.0.1.50";
char mqtt_port[6] = "1883";
char mqtt_user[24] = "";
char mqtt_pass[24] = "";
char unit_id[16] = "wesh0";
char group_id[16] = "weshgrp0";

// The extra parameters to be configured (can be either global or just in the setup)
// After connecting, parameter.getValue() will get you the configured value
// id/name placeholder/prompt default length
//WiFiManagerParameter custom_mqtt_server = NULL;




// these are defined after wifi connect and parameters are set (in setup())
String sensorTempTopic;    // publish temp sensor value
String sensorHumidTopic;   // publish humidity sensor value
String sensorPressTopic;   // publish pressure sensor value
String matrixActionSTopic; // subscribed to change LED matrix data "SXYXYXY", S=0 off, S=1 = on, x/y = 0..7
String accelActionSTopic; // subscribed to get accel data

#define MAX_SUBSCRIBED_TOPICS 6
String* subscribedTopics[MAX_SUBSCRIBED_TOPICS];
uint8_t noSubscribedTopics = 0;

