#include "AlcoholManager.h"

AlcoholManager::AlcoholManager(uint8_t alcoholPin, uint8_t switchPin)
  : alcoholSensorPin(alcoholPin), switchPin(switchPin) {}

void AlcoholManager::begin()
{
  pinMode(alcoholSensorPin, INPUT);
  pinMode(switchPin, INPUT_PULLUP);
}

bool AlcoholManager::isSwitchPressed()
{
  static bool lastSwitch = HIGH;
  bool current = digitalRead(switchPin);
  bool pressed = (lastSwitch == HIGH && current == LOW);
  lastSwitch = current;
  return pressed;
}

void AlcoholManager::startMeasuring()
{
  measuring = true;
  startTime = millis();
  lastSampleTime = 0;
  sum = 0;
  count = 0;
}

bool AlcoholManager::isMeasuring()
{
  return measuring;
}

bool AlcoholManager::update()
{
  if (!measuring) return false;

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_INTERVAL)
  {
    lastSampleTime = now;
    int val = analogRead(alcoholSensorPin);
    sum += val;
    count++;
  }

  if (now - startTime >= MEASURE_DURATION)
  {
    measuring = false;
    float avg = (float)sum / count;
    Serial.print("Average: ");
    Serial.println(avg);

    return avg < THRESHOLD;
  }

  return false;  // 아직 측정 중
}
