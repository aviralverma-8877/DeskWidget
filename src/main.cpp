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
#include "graphicsLCD.h"
#include <ArduinoJSON.h>

#define lcd_RST_pin 22 // LCD HX1230 pins, (no pin6) Change to your desire pins
#define lcd_CS_pin 21
#define lcd_DIN_pin 18
#define lcd_CLK_pin 5
#define MQTT_HOST "iot-connect.in"
#define MQTT_PORT 1883
//MQTT Cred
#define MQTT_UNAME "iotconnect"
#define MQTT_PASS "iot-12345"

#define leftButton 33
#define rightButton 26
#define led 25
#define backled 4
#define touch 13

#define MQTTATT "tableClock-att"
#define MQTTIN "tableClock-input"
#define MQTTOUT "tableClock-output"



Ticker tickerTouch;
Ticker bkledTicker;
Ticker ledTicker;
Ticker tickerLeftButtonPressed;
Ticker tickerRightButtonPressed;
Ticker tickerNoButtonPressed;
Ticker tickerNotification;
Ticker tickerAttendence;

TimerHandle_t mqttReconnectTimer;

byte lastPressed = 0;
bool ledStatus = HIGH;
bool notification = false;
bool touched_button = false;
bool touched_button_confirm = false;
byte totalOptions = 5;
byte menu = 0;

AsyncWebServer server(80);
DNSServer dns;
AsyncMqttClient mqtt;

graphicsLCD lcd(lcd_RST_pin,lcd_CS_pin,lcd_DIN_pin,lcd_CLK_pin);

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
void checkNotification();
void attendence();

void leftButtonPressed()
{
  if(lastPressed != leftButton)
     if(digitalRead(leftButton))
     {
        lastPressed = leftButton;
        backledAlert();
        toggleLEDon();
        mqtt.publish(MQTTOUT, 1, true, "left");
        notification = false;
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

int touchValue;
int touchCount = 0;
void touched()
{
  if(!touched_button_confirm)
  {
    if(touchCount == 0)
    {
      touchValue = touchRead(touch);
    }
    else
    {
        touchValue = (touchValue+touchRead(touch))/2;
    }
    if(touchCount == 10)
    {
      touchCount = 0;
      //lcd.print(touchValue);
      touched_button_confirm = true;
    }
    else
    {
      touchCount++;
    }
    
  }
  if(touched_button_confirm)
  {
    
    if(touchValue<=35)
    {
      //lcd.print(touchValue);
      if(!touched_button)
      {        
        touched_button = true;
        mqtt.publish(MQTTOUT, 1, true, "touch");
        backledAlert();
      }
    }
    else
    {
      touched_button = false;
    }
    touched_button_confirm = false;
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
      notification = false;
   }   
}

void printOnLCD(String str)
{
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, str);
  if(error)
  {
    lcd.line(1);
    int str_len = str.length() + 1;     
    char char_array[str_len];    
    str.toCharArray(char_array, str_len);    
    lcd.print(char_array);    
  }
  else
  {
    JsonObject obj = doc.as<JsonObject>();
    int line = obj[String("L")];
    for(int i=0; i<=10; i++)
    {
      if(i <= line)
      {
        String l = obj[String(i)];
        int str_len = l.length() + 1;     
        char char_array[17] = "                ";   
        lcd.line(i);
        lcd.print(char_array); 
        l.toCharArray(char_array, str_len);    
        lcd.line(i);
        lcd.print(char_array);
      }
      else
      {
        char char_array[17] = "                ";   
        lcd.line(i);
        lcd.print(char_array);
      }
      
    }  
  }
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
  tickerNotification.attach_ms(500, checkNotification);
  tickerAttendence.attach(5,attendence);
}
void connectToMqtt() {
  if(WiFi.status() == WL_DISCONNECTED)
  {
    ESP.restart();
  }
  else
  {
    Serial.println("Connecting to MQTT...");
    mqtt.connect();
  }
}

void backledAlert()
{
  digitalWrite(backled, HIGH);
  bkledTicker.once(5, turnOffBackled);
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
  if(strcmp(msg.c_str(),"ALERT")==0)
  {
    backledAlert();
    notification = true;
  }
  if(strcmp(msg.c_str(),"NOALERT")==0)
  {
    notification = false;
  }
  else
    printOnLCD(msg);
}

void onMqttPublish(uint16_t packetId) {
  //Serial.println("MQTT Publish");
}

void inSetupMode(AsyncWiFiManager *myWiFiManager)
{
  printOnLCD("In Setup Mode.\n\nWiFi AP:\nTable Clock");
}

void checkNotification()
{
  if(notification)
  {
    digitalWrite(led, !digitalRead(led));
  }
  else
  {
    digitalWrite(led, LOW);
  }
}

void attendence()
{
  mqtt.publish(MQTTATT, 1, true, "attendence");
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
   tickerTouch.attach_ms(10, touched);
   tickerLeftButtonPressed.attach_ms(10, leftButtonPressed);
   tickerRightButtonPressed.attach_ms(10, rightButtonPressed);
   tickerNoButtonPressed.attach_ms(5, NobuttonPressed);
//init LCD display
   lcd.begin();
// Clear the screen
   lcd.clear();
//check button pressed
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
   wifiManager.setTryConnectDuringConfigPortal(false);
   wifiManager.setAPCallback(inSetupMode);
   if(digitalRead(leftButton))
   {
     if(!wifiManager.startConfigPortal("Table Clock"))
     {
      ESP.restart();
     }
   }
   else
   {
    WiFi.begin();
    while (WiFi.status() != WL_CONNECTED) {
          delay(500);
          Serial.print(".");
      }
   }
   savedAndConnected();
}

void loop() {
}