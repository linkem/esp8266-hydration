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
  bool useMQTT = useWifi && true;
  const char *publish_topic = "home/livingroom/hydration/avocado";
  const char *sub_topic = "config/home/livingroom/hydration/avocado";
  //general
  unsigned long humAndTempCheckPeriod = 10000; //1s

  bool userLCD = true;

} config;

//general params
unsigned long time_now = 0;

//lcd params
LiquidCrystal_I2C lcd(0x27, 16, 2);
unsigned long lcd_refreshes = 0;

//dht params
DHT dht(DHTPIN, DHTTYPE);
String publishMessage;

//mqtt params
WiFiClient espClient;
PubSubClient mqttClient(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
String mqttClientId;
//soil moisture params
const int analogInPin = A0;

void printLine(const String &s, int line = 0)
{
  lcd.setCursor(0, line);
  lcd.print(s);
}
void lcdPrint(const String &line1, const String &line2, bool clear = false)
{
  if (clear)
  {
    lcd.clear();
  }
  printLine(line1);
  printLine(line2, 1);
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
    printLine("WiFi connected ");
    printLine("IP address:");
    printLine(WiFi.localIP().toString(), 1);
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
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  dht.begin();
  setup_wifi();
  setup_mqtt();
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
void writeSensorData()
{
  unsigned long currentMillis = millis();
  if ((unsigned long)(currentMillis - time_now) > config.humAndTempCheckPeriod)
  {
    time_now = currentMillis;
    lcd.clear();
    lcd_refreshes++;
    float humidity = dht.readHumidity();
    float temp = dht.readTemperature();
    
    String envString = String(humidity, 1) + "%H " + String(temp, 1) + "*C " + lcd_refreshes;
    printLine(envString);

    int soilMoisture = analogRead(analogInPin);
    String hydrationString;
    if (soilMoisture > 700)
    {
      hydrationString = "Dehydrated, " + String(soilMoisture);
      digitalWrite(LED_BUILTIN, LOW); // soil is dry
      printLine(hydrationString, 1);
    }
    else
    {
      hydrationString = "Hydrated, " + String(soilMoisture);
      digitalWrite(LED_BUILTIN, HIGH); // soil is wet
      printLine(hydrationString, 1);
    }

    HydrationSensor sensorData(mqttClientId.c_str(), humidity, temp, soilMoisture, "avocadoi soil");
    publish_message(config.publish_topic, sensorData.getJson().c_str());
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    printLine1("Attempting MQTT connection...");
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
      printLine(errorMessageLine1);
      printLine(errorMessageLine2, 1);
      Serial.print(errorMessageLine1);
      Serial.println(errorMessageLine2);
      ;
      delay(5000); // Wait 5 seconds before retrying
    }
  }
}
void loop()
{
  writeSensorData();
  if (!config.useMQTT || !config.useWifi)
  {
    return;
  }
  if (!mqttClient.connected())
  {
    reconnect();
  }
  mqttClient.loop();
}