#include "AlcoholManager.h"

void AlcoholManager::begin()
{
  pinMode(A2, INPUT);         // ALCOHOL_SENSOR_PIN
  pinMode(8, INPUT_PULLUP);   // SWITCH_PIN
}

bool AlcoholManager::isSwitchPressed()
{
  static bool lastSwitch = HIGH;
  bool current = digitalRead(8); // SWITCH_PIN
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
    int val = analogRead(A2);  // ALCOHOL_SENSOR_PIN
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
