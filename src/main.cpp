#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "HydrationSensor.h"
#include "Helpers.h"

#define DHTPIN D6 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11
#define MOISTURE_PIN A0
// #define PUMP_PIN `D3

//general types
struct Config
{
  //wifi params
  bool useWifi = true;
  const char *ssid = "ALHN-3A82";
  const char *password = "LubiePlacki2018";
  const char *mqtt_server = "192.168.1.100";
  int mqtt_server_port = 1883;

  //mqtt params
  bool useMQTT = useWifi && true;
  const char *publish_topic = "home/livingroom/hydration/avocado";
  const char *sub_topic = "config/home/livingroom/hydration/avocado";
  //general
  unsigned long generalPeriod = 1000;                //1s
  unsigned long humAndTempCheckPeriod = 1000;        //1s
  unsigned long lcdRefreshPeriod = 1000;             //1s
  unsigned long pumpPeriod = 20000;                  //20s
  unsigned long mqttPubllishPeriod = 1 * 30 * 500;   //0.5min
  unsigned long sleepPeriod = /*60 * */ 60 * 1000 * 1000; //60s
                                      //30    000    000
  bool userLCD = true;
  //hydration
  int hydrationLevel = 700;
} config;

//general params
unsigned long general_time_now = 0;
unsigned long sensors_time_now = 0;
unsigned long pump_time_now = 0;
unsigned long mqtt_publish_time_now = 0;
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
int soilMoisture;

#pragma region lcd_helpers
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
#pragma endregion lcd_helpers
#pragma region setup
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

void setup_lcd()
{
  lcd.init();
  lcd.backlight();
}

void setup_dht()
{
  dht.begin();
}
void setup_pins()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // pinMode(PUMP_PIN, OUTPUT);
}
#pragma endregion setup
#pragma region mqtt
void reconnect()
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    lcdPrintLine("Attempting MQTT connection...", 1);
    // Create a random client ID
    // Attempt to connect
    if (mqttClient.connect(mqttClientId.c_str()))
    {
      Serial.println("connected");
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
void publish_message(const char *topic, const char *message)
{
  if (!config.useMQTT)
    return;

  if (!mqttClient.connected())
    reconnect();

  Serial.println("Publish - topic: ");
  Serial.println(topic);
  Serial.println(message);
  mqttClient.publish(topic, message);
}

#pragma endregion mqtt
void printSensorData()
{
  lcd_refreshes++;
  String envString = String(humidity, 1) + "%H " + String(temperature, 1) + "*C" + lcd_refreshes;
  String hydrationString;

  hydrationString = soilMoisture > config.hydrationLevel ? "Dehydrated, " : "Hydrated, ";
  hydrationString += String(soilMoisture);
  lcdPrint(envString, hydrationString, true);
}
void publishSensorData()
{
  digitalWrite(LED_BUILTIN, LOW);
  delay(125);
  HydrationSensor sensorData(mqttClientId.c_str(), humidity, temperature, soilMoisture, "avocado soil");
  publish_message(config.publish_topic, sensorData.getJson().c_str());
  delay(125);
  digitalWrite(LED_BUILTIN, HIGH);
}
void readSensorData()
{
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  soilMoisture = analogRead(MOISTURE_PIN);
}

#pragma region pump
// bool usePump()
// {
//   return soilMoisture > config.hydrationLevel && !isnan(humidity) && !isnan(temperature);
// }
// void pump()
// {
//   unsigned long currentMillis = millis();
//   if ((unsigned long)(currentMillis - pump_time_now) > config.pumpPeriod && usePump())
//   {
//     pump_time_now = currentMillis;
//     digitalWrite(PUMP_PIN, HIGH);
//     delay(3000);
//     digitalWrite(PUMP_PIN, LOW);
//   }
// }
#pragma endregion pump

void setup()
{
  Serial.begin(74880);
  while (!Serial)
  {
  }
  Serial.println("AWAKE");
  setup_pins();
  setup_dht();
  setup_lcd();
  setup_wifi();
  setup_mqtt();

  delay(2000);
  readSensorData();
  printSensorData();
  publishSensorData();
  printSensorData();

  Serial.println("ESP go sleep");
  lcd.setCursor(11,1);
  lcd.print("Sleep");
  delay(100);
  ESP.deepSleep(config.sleepPeriod);

}
void loop()
{
  // unsigned long currentMillis = millis();
  // if ((unsigned long)(currentMillis - general_time_now) > config.generalPeriod || general_time_now == 0)
  // {
  //   general_time_now = currentMillis;

  // }

  // printSensorData();
  // publishSensorData();
  // pump();

  // mqttClient.loop();
}