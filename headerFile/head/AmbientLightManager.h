#ifndef AMBIENT_LIGHT_MANAGER_H
#define AMBIENT_LIGHT_MANAGER_H

class AmbientLightManager {
  public:
    void begin();
    void update();

  private:
    static const int LIGHTSAMPLESIZE = 10;
    static const int LIGHT_THRESHOLD = 140;

    int light_samples[LIGHTSAMPLESIZE];
    int light_sample_index;
    int total_lights;
    float avg_light;

    unsigned long now_light_measuretime;
    unsigned long pre_light_measuretime;
    static const unsigned long light_measuretime_interval;
};

#endif
