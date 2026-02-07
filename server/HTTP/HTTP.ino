#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// WiFi info
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";

WebServer server(81);  // web server instance at port 81 according to pc web server

// for serial programming
// #define ARDUINO_SERIAL Serial

// ==== 1. camera init ====
void startCameraServer() {
  // server config : registering handlers to respond to different requests

  // [] : setting access to variables from the surrounding scope (outside of itself)
  // () : parameters for the lambda function code blocks

  server.on("/stream", HTTP_GET, []() {
    WiFiClient client = server.client(); // returns a WiFiClient object
    String response = "HTTP/1.1 200 OK\r\n";
    // tells to replace prev data and set boundary
    response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    server.sendContent(response);

    while (client.connected()) {
      camera_fb_t * fb = esp_camera_fb_get(); // structure pointer, image encoded into JPEG format
      // instead of copying large img, simply points to the memory location
      if (!fb) {
        Serial.println("Camera capture failed");
        continue;
      }
      // HTTP standard requires to start boundary string with '--'
      // \r\n : standard line ending in HTTP
      server.sendContent("--frame\r\n");
      server.sendContent("Content-Type: image/jpeg\r\n\r\n");
      server.sendContent((const char *)fb->buf, fb->len); // start(address), end
      // buf(uint8_t) is a pointer too pointing to the start of the img pixel, len is an integer
      // sendContent expects to change pointer to bytes to pointer to characters
      server.sendContent("\r\n");

      esp_camera_fb_return(fb);
      delay(20); // a bit of delay between frames
    }
  });
}

// ==== 2. data handler ====
void setupControlRoute() {
  server.on("/control", HTTP_GET, []() {
    if (server.hasArg("cmd")) 
    { 
      // ex) http://192.168.2.2:81/control?cmd=P
      String cmd = server.arg("cmd");
      Serial.println("Received command: " + cmd);
      // ARDUINO_SERIAL.println(cmd);  // sending data to arduino
    }
    server.send(200, "text/plain", "Command received"); 
    // text/plain == Content-Type header of the response
  });
}

// ==== 3. init ====
void setup() {
  Serial.begin(115200);
  // ARDUINO_SERIAL.begin(9600); // arduino connection

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());

  // camera config
  camera_config_t config;
  // depends on models

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  // Select frame size - PSRAM needed for higher resolutions
  // config.frame_size = FRAMESIZE_UXGA; // (1600 x 1200)
  // config.frame_size = FRAMESIZE_SXGA; // (1280 x 1024)
  // config.frame_size = FRAMESIZE_XGA;  // (1024 x 768)
  config.frame_size = FRAMESIZE_SVGA; // (800 x 600)
  // config.frame_size = FRAMESIZE_VGA;  // (640 x 480)
  // config.frame_size = FRAMESIZE_CIF;  // (400 x 296)
  config.jpeg_quality = 12; // 0-63 lower number means higher quality
  config.fb_count = 1;      // Use 1 frame buffer when streaming

  // camera init
  // check if ESP_OK received
  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) 
  {
    Serial.printf("Camera init failed with error 0x%x", err); // hexadecimal: makes it easy to look up
    ESP.restart(); // Restart if camera init fails
    return;
  }
  Serial.println("Camera initialized successfully.");

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }

  ARDUINO_SERIAL.begin(9600);

  startCameraServer();    // video stream
  setupControlRoute();    // control
  server.begin();         // start web server
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();  // checks requests
}
