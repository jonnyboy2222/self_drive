#ifndef AMBIENT_LIGHT_MANAGER_H
#define AMBIENT_LIGHT_MANAGER_H

#include <Arduino.h>

class AmbientLightManager {
  public:
    AmbientLightManager(uint8_t lightPin, uint8_t ledPin);
    void begin();
    void update();
    bool getLightState();
  private:
    uint8_t lightSensorPin;
    uint8_t ledPin;

    static const int LIGHTSAMPLESIZE = 10;
    static const int LIGHT_THRESHOLD = 140;
    static const unsigned long light_measuretime_interval;

    int light_samples[LIGHTSAMPLESIZE];
    int light_sample_index;
    int total_lights;
    float avg_light;

    unsigned long now_light_measuretime;
    unsigned long pre_light_measuretime;

    bool lightOn = false;
};

#endif

