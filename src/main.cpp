#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "HydrationSensor.h"

#define DHTPIN 13 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11

//general types
struct Config
{
  //wifi params
  bool useWifi = true;
  const char *ssid = "ALHN-3A82";
  const char *password = "LubiePlacki2018";
  const char *mqtt_server = "192.168.1.68";
  int mqtt_server_port = 1883;

  //mqtt params
  bool useMQTT = useWifi && false;
  const char *publish_topic = "home/livingroom/hydration/avocado";
  const char *sub_topic = "config/home/livingroom/hydration/avocado";
  //general
  unsigned long humAndTempCheckPeriod = 1000; //1s
  unsigned long lcdRefreshPeriod = 1000;      //1s
  unsigned long pumpPeriod = 20000;           //20s

  bool userLCD = true;
  //hydration
  int hydrationLevel = 700;
} config;

//general params
unsigned long sensors_time_now = 0;
unsigned long pump_time_now = 0;
bool pumpUsed = false;

//lcd params
LiquidCrystal_I2C lcd(0x27, 16, 2);
unsigned long lcd_refreshes = 0;

//dht params
DHT dht(DHTPIN, DHTTYPE);
String publishMessage;
float humidity;
float temperature;

//mqtt params
WiFiClient espClient;
PubSubClient mqttClient(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
String mqttClientId;

//soil moisture params
const int MOISTURE_PIN = A0;
int soilMoisture;

//pump
const int PUMP_PIN = D3;

void lcdPrintLine(const String &s, int line = 0)
{
  lcd.setCursor(0, line);
  lcd.print(s);
}
void lcdPrint(const String &line1, const String &line2, bool clear = false)
{
  if (clear)
    lcd.clear();

  lcdPrintLine(line1);
  lcdPrintLine(line2, 1);
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if ((char)payload[0] == '1')
  {
    digitalWrite(LED_BUILTIN, LOW); // Turn the LED on (Note that LOW is the voltage level
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH); // Turn the LED off by making the voltage HIGH
  }
}
void setup_wifi()
{
  // We start by connecting to a WiFi network
  if (config.useWifi)
  {
    lcdPrint("Connecting to ", config.ssid, true);

    WiFi.begin(config.ssid, config.password);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);

      lcd.print(".");
    }
    lcd.clear();
    lcdPrintLine("WiFi connected");
    lcdPrintLine("IP address:");
    lcdPrintLine(WiFi.localIP().toString(), 1);
  }
}

void setup_mqtt()
{
  if (config.useMQTT)
  {
    mqttClient.setServer(config.mqtt_server, config.mqtt_server_port);
    mqttClient.setCallback(mqttCallback);
    mqttClientId = WiFi.macAddress();
  }
}
void setup()
{
  Serial.begin(74880);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  lcd.init();
  lcd.backlight();
  dht.begin();
}

void publish_message(const char *topic, const char *message)
{
  if (!mqttClient.connected() || !config.useMQTT)
  {
    return;
  }

  Serial.println(message);
  mqttClient.publish(topic, message);
}

void printSensorData()
{
  lcd_refreshes++;
  String envString = String(humidity, 1) + "%H " + String(temperature, 1) + "*C" + lcd_refreshes;
  String hydrationString;
  if (soilMoisture > config.hydrationLevel)
  {
    hydrationString = "Dehydrated, " + String(soilMoisture);
    digitalWrite(LED_BUILTIN, LOW); // soil is dry
  }
  else
  {
    hydrationString = "Hydrated, " + String(soilMoisture);
    digitalWrite(LED_BUILTIN, HIGH); // soil is wet
  }
  lcdPrint(envString, hydrationString, true);
}

void readSensorData()
{
  unsigned long currentMillis = millis();
  if ((unsigned long)(currentMillis - sensors_time_now) > config.humAndTempCheckPeriod)
  {
    sensors_time_now = currentMillis;
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
    soilMoisture = analogRead(MOISTURE_PIN);
    printSensorData();
  }
}

void publishSensorData()
{
  HydrationSensor sensorData(mqttClientId.c_str(), humidity, temperature, soilMoisture, "avocado soil");
  publish_message(config.publish_topic, sensorData.getJson().c_str());
}
void reconnect()
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    lcdPrintLine("Attempting MQTT connection...");
    // Create a random client ID
    // Attempt to connect
    if (mqttClient.connect(mqttClientId.c_str()))
    {
      Serial.println("connected");
      mqttClient.publish(config.publish_topic, "hello world");
      mqttClient.subscribe(config.sub_topic);
    }
    else
    {
      String errorMessageLine1 = "failed, rc=" + mqttClient.state();
      String errorMessageLine2 = "try again in 5 seconds";
      lcdPrintLine(errorMessageLine1);
      lcdPrintLine(errorMessageLine2, 1);
      Serial.print(errorMessageLine1);
      Serial.println(errorMessageLine2);
      delay(5000); // Wait 5 seconds before retrying
    }
  }
}
bool usePump()
{
  return soilMoisture > config.hydrationLevel && humidity != 0.0 && temperature != 0.0;
}
void pump()
{
  unsigned long currentMillis = millis();
  if ((unsigned long)(currentMillis - pump_time_now) > config.pumpPeriod && usePump())
  {
    pump_time_now = currentMillis;
    digitalWrite(PUMP_PIN, HIGH);
    delay(3000);
    digitalWrite(PUMP_PIN, LOW);
  }
}

void loop()
{
  readSensorData();
  // printSensorData();
  // publishSensorData();
  pump();
  // if (!config.useMQTT || !config.useWifi)
  // {
  //   return;
  // }
  // if (!mqttClient.connected())
  // {
  //   reconnect();
  // }
  // mqttClient.loop();
}