#include "Arduino.h"
#include "Myclass.h"

#define ALCOHOL_SENSOR_PIN A1
#define SWITCH_PIN 36

void AlcoholManager::begin() {
    pinMode(ALCOHOL_SENSOR_PIN, INPUT);
    pinMode(SWITCH_PIN, INPUT_PULLUP);
}

bool AlcoholManager::isSwitchPressed()
{
    static bool lastSwitch = HIGH;
    bool current = digitalRead(SWITCH_PIN);
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
    if (now - lastSampleTime >= SAMPLEINTERVAL)
    {
    lastSampleTime = now;
    int val = analogRead(ALCOHOL_SENSOR_PIN);
    sum += val;
    count++;
    }

    if (now - startTime >= MEASUREDURATION)
    {
    measuring = false;
    float avg = (float)sum / count;
    Serial.print("Average: ");
    Serial.println(avg);
    if (avg < THRESHOLD)
    {
        return true;
    }
    else
    {
        return false;
    }

    }

    return false; // 아직 측정 중
}