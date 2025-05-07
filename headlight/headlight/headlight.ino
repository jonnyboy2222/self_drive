const int PHOTOPIN = A0;    // 포토 레지스터 연결 핀
const int LED_L = 12 ;       
const int LED_R = 6 ;

const int LIGHTSAMPLESIZE = 10;
int light_samples[LIGHTSAMPLESIZE] = {0};

int light_sample_index = 0;
int total_lights = 0;
float avg_light = 0;
unsigned long now_light_measuretime = 0;
unsigned long pre_light_measuretime = 0;
unsigned long light_measuretime_interval = 50;

int light_threshold = 21;

void setup() 
{
  Serial.begin(9600);
  pinMode(LED_L, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(PHOTOPIN, INPUT);

  digitalWrite(LED_L, LOW);
  digitalWrite(LED_R, LOW);

}

///////////////////////////////////////////////////////////////////////result_avg_Light///////////////////////////////////////////////////////////////////
void result_avg_Light()
{  
  int light = analogRead(PHOTOPIN);
  int map_light = map(constrain(light, 50, 900), 50, 900, 255, 0);

  total_lights -= light_samples[light_sample_index];
  light_samples[light_sample_index] = map_light;
  total_lights += light_samples[light_sample_index];
  light_sample_index = (light_sample_index+1) % LIGHTSAMPLESIZE;

  pre_light_measuretime = now_light_measuretime;
  
  if (light_sample_index == 0)
  {
    avg_light = total_lights / LIGHTSAMPLESIZE;
  }
}

///////////////////////////////////////////////////////////////////임계치에 따른 led 조절///////////////////////////////////////////////////////////////
void led_byThreshold()
{
  if (avg_light > light_threshold)
  {
    digitalWrite(LED_L, HIGH);
    digitalWrite(LED_R, HIGH);
  }
  else 
  {
    digitalWrite(LED_L, LOW);
    digitalWrite(LED_R, LOW);
  }
}

//////////////////////////////////////////////////////////////////메인함수///////////////////
void loop() 
{
  now_light_measuretime = millis(); 
  if (now_light_measuretime - pre_light_measuretime >= light_measuretime_interval)
  {
    result_avg_Light();
    led_byThreshold();

    Serial.println(avg_light);
  }
}