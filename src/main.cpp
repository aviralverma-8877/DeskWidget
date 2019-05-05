#if defined(ESP8266)
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#else
#include <WiFi.h>
#endif

#include <ESPAsyncWebServer.h>     //Local WebServer used to serve the configuration portal
#include <ESPAsyncWiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}

#include <AsyncMqttClient.h>

#include <Ticker.h>
#include <Nokia_LCD.h>
#define MQTT_HOST "MQTT Server"
#define MQTT_PORT 1883
//MQTT Cred
#define MQTT_UNAME "MQTT USERNAME"
#define MQTT_PASS "MQTT PSK"

#define leftButton 33
#define rightButton 26
#define led 25
#define backled 4
#define MQTTIN "tableClock-input"
#define MQTTOUT "tableClock-output"


Ticker tickerLEDonButtonPress;
Ticker bkledTicker;
Ticker ledTicker;
Ticker tickerLeftButtonPressed;
Ticker tickerRightButtonPressed;
Ticker tickerNoButtonPressed;

TimerHandle_t mqttReconnectTimer;

byte lastPressed = 0;
bool ledStatus = HIGH;
byte totalOptions = 5;
byte menu = 0;

AsyncWebServer server(80);
DNSServer dns;
AsyncMqttClient mqtt;

Nokia_LCD lcd(5 /* CLK */, 18 /* DIN */, 19 /* DC */, 21 /* CE */, 22 /* RST */);

void leftButtonPressed();
void NobuttonPressed();
void rightButtonPressed();
void printOnLCD(String option);
void turnOffBackled();
void inSetupMode(AsyncWiFiManager *myWiFiManager);
void savedAndConnected();
void backledAlert();
void MQTT_connect(bool sessionPresent);
void toggleLEDon();
void toggleLEDoff();
void connectToMqtt();
void onMqttPublish(uint16_t packetId);
void onMqttMessage(char* topic, String payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);

void leftButtonPressed()
{
  if(lastPressed != leftButton)
     if(digitalRead(leftButton))
     {
        lastPressed = leftButton;
        backledAlert();
        toggleLEDon();
        mqtt.publish(MQTTOUT, 1, true, "left");
     }
}

void NobuttonPressed()
{
  if(!digitalRead(leftButton))
    if(!digitalRead(rightButton))
    {
      lastPressed = 0;
    }
}


void rightButtonPressed()
{
  if(lastPressed != rightButton)
   if(digitalRead(rightButton))
   {
      lastPressed = rightButton;
      backledAlert();
      toggleLEDon();
      mqtt.publish(MQTTOUT, 1, true, "right");
   }   
}

void printOnLCD(String option)
{
  lcd.clear(); 
  lcd.println(option.c_str()); 
}


void turnOffBackled()
{
  digitalWrite(backled, LOW);
}
void savedAndConnected()
{
  //staring the startup sequence.  
   printOnLCD("Setup Completed.\nConnected to:\n"+WiFi.SSID());
   connectToMqtt();
}
void MQTT_connect(bool sessionPresent) {
  Serial.println("MQTT Connected");
  uint16_t packetIdSub = mqtt.subscribe(MQTTIN, 2);
}
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqtt.connect();
}

void backledAlert()
{
  digitalWrite(backled, HIGH);
  bkledTicker.once(15, turnOffBackled);
}

void toggleLEDon()
{
  digitalWrite(led, HIGH);
  ledTicker.once(0.01,toggleLEDoff);
}
void toggleLEDoff()
{
  digitalWrite(led, LOW);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
}

void onMqttUnsubscribe(uint16_t packetId) {
}

void onMqttMessage(char* topic, String payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String msg = "";
  for(int i=0; i<len; i++)
    msg += payload[i];
  printOnLCD(msg);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("MQTT Publish");
}

void inSetupMode(AsyncWiFiManager *myWiFiManager)
{
  printOnLCD("In Setup Mode.\n\nWiFi AP:\nTable Clock");
}


void setup() {
// init serial for debug:
   Serial.begin(115200);
   Serial.println("Starting...");
// init pins
   pinMode(led, OUTPUT);
   pinMode(leftButton, INPUT);
   pinMode(rightButton, INPUT);
   pinMode(backled, OUTPUT);
   
//init LCD display
   lcd.begin();
 // Set the contrast
   lcd.setContrast(50);  // Good values are usualy between 40 and 60
 // Clear the screen
   lcd.clear();
//check button pressed
   tickerLeftButtonPressed.attach_ms(10, leftButtonPressed);
   tickerRightButtonPressed.attach_ms(10, rightButtonPressed);
   tickerNoButtonPressed.attach_ms(5, NobuttonPressed);

   mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
   mqtt.onConnect(MQTT_connect);
   mqtt.onDisconnect(onMqttDisconnect);
   mqtt.onSubscribe(onMqttSubscribe);
   mqtt.onUnsubscribe(onMqttUnsubscribe);
   mqtt.onMessage(onMqttMessage);
   mqtt.onPublish(onMqttPublish);
   mqtt.setCredentials(MQTT_UNAME, MQTT_PASS);
   mqtt.setServer(MQTT_HOST, MQTT_PORT);


//init wifi manager
   printOnLCD("Trying WiFi...");
   AsyncWiFiManager wifiManager(&server,&dns);
   wifiManager.setAPCallback(inSetupMode);
   wifiManager.setSaveConfigCallback(savedAndConnected);
   if(digitalRead(leftButton))
   {
     if(!wifiManager.startConfigPortal("Table Clock"))
     {
      ESP.restart();
     }
   }
   else
   {
    if(!wifiManager.autoConnect("Table Clock"))
    {
        ESP.restart();
    }
   }
   savedAndConnected();
}

void loop() {
}
