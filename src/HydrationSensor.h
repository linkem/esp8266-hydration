#include <ArduinoJson.h>

class HydrationSensor
{
public:
    /* data */
    const char* sensorId;
    const char* sensorName;
    float humidity;
    float temperature;
    int soilMoisture;

    String getJson();
    HydrationSensor(const char* sensID, float hum, float temp, int soil, const char* sensName);
    ~HydrationSensor();
};

HydrationSensor::HydrationSensor(const char* sensID, float hum, float temp, int soil, const char* sensName = "")
{
    sensorId = sensID;
    sensorName = sensName;
    humidity = hum;
    temperature = temp;
    soilMoisture = soil;
}

HydrationSensor::~HydrationSensor()
{
}
String HydrationSensor::getJson(void){
    const size_t capacity = JSON_OBJECT_SIZE(5);
    DynamicJsonDocument doc(capacity);

    doc["sensorId"] = sensorId;
    doc["sensorName"] = sensorName;
    doc["temp"] = temperature;
    doc["hum"] = humidity;
    doc["soil"] = soilMoisture;

    String json;
    serializeJson(doc, json);
    return json;
}