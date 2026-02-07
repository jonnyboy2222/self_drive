#include "AmbientLightManager.h"

const unsigned long AmbientLightManager::light_measuretime_interval = 50;

AmbientLightManager::AmbientLightManager(uint8_t lightPin, uint8_t led)
  : lightSensorPin(lightPin), ledPin(led) {}

void AmbientLightManager::begin() {
  pinMode(ledPin, OUTPUT);
  pinMode(lightSensorPin, INPUT);
  digitalWrite(ledPin, LOW);

  light_sample_index = 0;
  total_lights = 0;
  avg_light = 0;
  now_light_measuretime = 0;
  pre_light_measuretime = 0;

  for (int i = 0; i < LIGHTSAMPLESIZE; i++) {
    light_samples[i] = 0;
  }
}

void AmbientLightManager::update() {
  now_light_measuretime = millis();
  if (now_light_measuretime - pre_light_measuretime >= light_measuretime_interval) {
    int light = analogRead(lightSensorPin);
    int map_light = map(constrain(light, 50, 1020), 50, 1020, 255, 0);

    total_lights -= light_samples[light_sample_index];
    light_samples[light_sample_index] = map_light;
    total_lights += map_light;

    light_sample_index = (light_sample_index + 1) % LIGHTSAMPLESIZE;

    if (light_sample_index == 0) {
      avg_light = total_lights / LIGHTSAMPLESIZE;
    }

    if (avg_light > LIGHT_THRESHOLD) {
      digitalWrite(ledPin, HIGH);
      lightOn = true;
    } else {
      digitalWrite(ledPin, LOW);
      lightOn = false;
    }

    pre_light_measuretime = now_light_measuretime;
  }
}

bool AmbientLightManager::getLightState()
{
  return lightOn;
}
