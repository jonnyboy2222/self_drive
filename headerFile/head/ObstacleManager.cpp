// === ObstacleManager.cpp ===
#include "ObstacleManager.h"
#include <Arduino.h>

#define ULTRASONIC_TRIG 28
#define ULTRASONIC_ECHO 29
#define BUZZER_PIN 11

void ObstacleManager::begin()
{
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
}

void ObstacleManager::setReversing(bool state)
{
  isbacking = state;
}

void ObstacleManager::avgDistance()
{
  unsigned long nowtime = millis();
  if (nowtime - pre_measuretime >= checktime_interval)
  {
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);

    unsigned long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 20000);
    float distance = duration * 0.034 / 2;

    if (distance > 0 && distance < 340)
    {
      total_distance -= samples[sampleindex];
      samples[sampleindex] = distance;
      total_distance += samples[sampleindex];
      sampleindex = (sampleindex + 1) % SAMPLESIZE;
    }
    pre_measuretime = nowtime;

    if (sampleindex == 0)
    {
      avg_distance = total_distance / SAMPLESIZE;
      Serial.print("AVG_DISTANCE: ");
      Serial.println(avg_distance);
    }
  }
}

void ObstacleManager::setBeepfreqByDistance()
{
  if (avg_distance <= 10) 
  {
    beepfreq = 2000; 
    beepinterval = 30;
  } 
  else if (avg_distance <= 20) 
  {
    beepfreq = 1500; 
    beepinterval = 50;
  } 
  else if (avg_distance <= 40) 
  {
    beepfreq = 1000; 
    beepinterval = 100;
  } 
  else if (avg_distance <= 70) 
  {
    beepfreq = 700;  
    beepinterval = 200;
  } 
  else if (avg_distance <= 100) 
  {
    beepfreq = 400;  
    beepinterval = 250;
  } 
  else 
  {
    beepfreq = 0;    
    beepinterval = 0;
    noTone(BUZZER_PIN);
  }
}

void ObstacleManager::controlBuzzer()
{
  unsigned long nowtime = millis();
  if (beepfreq > 0 && nowtime - prebeeptime >= beepinterval)
  {
    prebeeptime = nowtime;
    if (buzzerstate)
    {
      noTone(BUZZER_PIN);
      buzzerstate = false;
    }
    else
    {
      tone(BUZZER_PIN, beepfreq);
      buzzerstate = true;
    }
  }
}

void ObstacleManager::update()
{
  if (isbacking)
  {
    avgDistance();
    setBeepfreqByDistance();
    controlBuzzer();
  }
  else 
  {
    noTone(BUZZER_PIN);
  }
}

