#include "ShockManager.h"
#include "Arduino.h"

#define SHOCK_SENSOR_PIN 26

ShockManager::ShockManager(uint8_t sensorPin)
  : pin(sensorPin),
    lastWindowTime(0),
    bufferIndex(0),
    countInWindow(0),
    lastSensorState(LOW),
    latest_average_shock(0.0)
{
  for (int i = 0; i < 5; i++) {
    buffer[i] = 0;
  }
}

void ShockManager::begin() 
{
  pinMode(pin, INPUT);
  lastWindowTime = millis();
}

void ShockManager::update() 
{
  bool currentSensorState = digitalRead(pin);

  // 상승 에지 감지 (LOW → HIGH)
  if (currentSensorState == HIGH && lastSensorState == LOW) 
  {
    countInWindow++;
  }
  lastSensorState = currentSensorState;

  unsigned long currentTime = millis();
  if (currentTime - lastWindowTime >= windowDuration) 
  {
    buffer[bufferIndex] = countInWindow;
    bufferIndex = (bufferIndex + 1) % 5;

    if (bufferIndex == 0) 
    {
      int sum = 0;
      for (int i = 0; i < 5; i++) 
      {
        sum += buffer[i];
      }
      latest_average_shock = sum / 5.0;
    }

    countInWindow = 0;
    lastWindowTime = currentTime;
  }
}

float ShockManager::getLatestAverageShock() 
{
  return latest_average_shock;
}
