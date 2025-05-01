#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// --- WiFi Configuration ---
const char* ssid = "Your_SSID"; // *** REPLACE WITH YOUR WIFI SSID ***
const char* password = "Your_PASSWORD"; // *** REPLACE WITH YOUR WIFI PASSWORD ***

// --- Flask Server Configuration ---
// IMPORTANT: Replace <FLASK_SERVER_IP> and <PORT> with your actual PC's IP and Flask port
const char* flaskServerIp = "<FLASK_SERVER_IP>";
const int flaskServerPort = 5000; // Default Flask port is 5000
String sensorPostUrl = String("http://") + flaskServerIp + ":" + String(flaskServerPort) + "/sensor";

// --- ESP32 WebServer Configuration ---
WebServer server(81); // Run ESP32 server on port 81

// --- UART Pin Configuration (for Arduino communication) ---
// *** REPLACE with the actual GPIO pins connected to your Arduino's Serial ***
#define RXD2 14 // GPIO pin connected to Arduino TX
#define TXD2 15 // GPIO pin connected to Arduino RX

// --- RTOS Configuration ---
TaskHandle_t TaskUARTReceiveHandle;
TaskHandle_t TaskNetworkSendHandle;
QueueHandle_t sensorDataQueue;
#define SENSOR_DATA_QUEUE_LENGTH 10 // How many sensor readings can be queued
#define SENSOR_DATA_MAX_LEN 128     // Max expected length of a sensor data string + null terminator

// --- Camera Configuration (AI-Thinker Model Example) ---
// *** IMPORTANT: Verify these pins match your specific ESP32-CAM model ***
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1 // -1 if not used
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// --- Function Declarations ---
void setupCamera();
void startCameraServer();
void setupControlRoute();
void uartReceiveTask(void *parameter);
void networkSendTask(void *parameter);


// =========================================================================
// CAMERA INITIALIZATION
// =========================================================================
void setupCamera() {
    camera_config_t config;
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
    config.pixel_format = PIXFORMAT_JPEG; // Use JPEG for streaming
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM; // Use PSRAM for frame buffer

    // Frame size - SVGA recommended for balance, adjust if needed
    config.frame_size = FRAMESIZE_SVGA; // (800 x 600)
    // config.frame_size = FRAMESIZE_VGA;  // (640 x 480)
    config.jpeg_quality = 12; // 0-63 (lower means higher quality)
    config.fb_count = 1;      // Use 1 frame buffer for streaming (saves memory)
                              // Use 2 if you face issues like image tearing

    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("FATAL ERROR: Camera init failed with error 0x%x\n", err);
        ESP.restart(); // Restart if camera fails
        return;
    }
    Serial.println("Camera initialized successfully.");

    // Optional: Adjust sensor settings if needed (e.g., for OV3660)
    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);       // Flip vertical
        s->set_brightness(s, 1);  // Adjust brightness
        s->set_saturation(s, -2); // Adjust saturation
    }
}

// =========================================================================
// WEBSERVER: /stream Handler (Video Streaming)
// =========================================================================
void startCameraServer() {
    server.on("/stream", HTTP_GET, []() {
        WiFiClient client = server.client();
        if (!client) {
            Serial.println("Stream: Client connection failed.");
            return;
        }
        Serial.println("Stream: Client connected.");

        String response = "HTTP/1.1 200 OK\r\n";
        response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
        server.sendContent(response); // Send the header

        while (client.connected()) {
            camera_fb_t * fb = esp_camera_fb_get(); // Get frame buffer
            if (!fb) {
                Serial.println("Stream Error: Camera capture failed");
                // Small delay before retrying
                vTaskDelay(100 / portTICK_PERIOD_MS);
                continue; // Skip this frame
            }

            // Send the frame buffer over HTTP
            client.write("--frame\r\n"); // Boundary marker
            client.write("Content-Type: image/jpeg\r\n");
            client.write("Content-Length: ");
            client.print(fb->len);
            client.write("\r\n\r\n"); // End of headers
            client.write(fb->buf, fb->len); // Write the JPEG data
            client.write("\r\n"); // End of frame part

            esp_camera_fb_return(fb); // Return frame buffer to be reused

            // Add a small delay to control frame rate and yield CPU
            // Adjust delay based on desired FPS (e.g., 50ms -> ~20 FPS)
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        Serial.println("Stream: Client disconnected.");
    });
}

// =========================================================================
// WEBSERVER: /control Handler (Receive commands from Flask)
// =========================================================================
void setupControlRoute() {
    server.on("/control", HTTP_GET, []() {
        if (server.hasArg("cmd")) {
            String cmd = server.arg("cmd");
            cmd.trim(); // Clean up command string

            Serial.println("Control: Received command from Flask: '" + cmd + "'");

            // Immediately forward the command to Arduino via Serial1
            Serial1.println(cmd);
            Serial.println("Control: Forwarded command to Arduino: '" + cmd + "'");

            // Send confirmation response back to Flask
            server.send(200, "text/plain", "Command received and forwarded: " + cmd);
        } else {
            Serial.println("Control Error: Received request without 'cmd' parameter.");
            server.send(400, "text/plain", "Bad Request: Missing 'cmd' parameter");
        }
    });
}

// =========================================================================
// RTOS TASK: UART Receive (Read from Arduino)
// =========================================================================
void uartReceiveTask(void *parameter) {
    Serial.println("UART Receive Task started on Core " + String(xPortGetCoreID()));
    char receivedData[SENSOR_DATA_MAX_LEN]; // Static buffer for incoming data

    while (true) {
        if (Serial1.available()) {
            // Read the incoming data line (until newline or buffer full)
            size_t len = Serial1.readBytesUntil('\n', receivedData, SENSOR_DATA_MAX_LEN - 1);

            if (len > 0) {
                receivedData[len] = '\0'; // Null-terminate the string
                String sensorDataStr(receivedData); // Create String object
                sensorDataStr.trim(); // Remove potential whitespace

                if (sensorDataStr.length() > 0) {
                    Serial.println("UART Received: " + sensorDataStr);

                    // --- Queue the data for the Network Task ---
                    // 1. Allocate memory on the heap for this specific data string
                    //    (The pointer to this memory will be sent via the queue)
                    char* dataToSend = (char*)malloc(sensorDataStr.length() + 1);

                    if (dataToSend != NULL) {
                        // 2. Copy the received data into the allocated memory
                        strcpy(dataToSend, sensorDataStr.c_str());

                        // 3. Send the *pointer* to the allocated memory to the queue.
                        //    Wait max 10 ticks if the queue is full.
                        if (xQueueSend(sensorDataQueue, &dataToSend, (TickType_t)10) == pdPASS) {
                            // Serial.println("Sensor data queued for network sending."); // Debug
                        } else {
                            Serial.println("UART Warn: Failed to queue sensor data (queue full?). Discarding.");
                            free(dataToSend); // IMPORTANT: Free memory if sending failed!
                        }
                    } else {
                        Serial.println("UART Error: Failed to allocate memory for sensor data! Discarding.");
                        // Memory allocation failed, cannot queue data
                    }
                }
            } else {
                 // Read 0 bytes, likely just a newline or empty line, ignore.
            }
        }

        // Wait a bit before checking Serial1 again to yield CPU time
        vTaskDelay(50 / portTICK_PERIOD_MS); // Check UART every 50ms
    }
}

// =========================================================================
// RTOS TASK: Network Send (POST sensor data to Flask)
// =========================================================================
void networkSendTask(void *parameter) {
    Serial.println("Network Send Task started on Core " + String(xPortGetCoreID()));
    char* dataToPost; // Pointer to receive data from the queue

    while (true) {
        // Wait indefinitely until an item (a char* pointer) is available in the queue
        if (xQueueReceive(sensorDataQueue, &dataToPost, portMAX_DELAY) == pdPASS) {

            // Check if we actually received a valid pointer
            if (dataToPost != NULL) {
                Serial.println("Network Task: Dequeued data for sending: " + String(dataToPost));

                // Check WiFi connection before attempting POST
                if (WiFi.status() == WL_CONNECTED) {
                    HTTPClient http;
                    http.begin(sensorPostUrl); // Target the Flask /sensor endpoint
                    // Set content type header - Assuming Flask expects JSON
                    // Adjust if your Arduino sends data in a different format
                    http.addHeader("Content-Type", "application/json");

                    // Perform the blocking HTTP POST request.
                    // This block only affects this task, not the web server or UART task.
                    Serial.println("Network Task: Sending POST to " + sensorPostUrl);
                    int httpResponseCode = http.POST(String(dataToPost)); // Convert char* back to String for POST

                    // Check the response code
                    if (httpResponseCode > 0) {
                        // String response = http.getString(); // Optional: Get response body
                        Serial.printf("Network Task: POST successful, Response Code: %d\n", httpResponseCode);
                        // Serial.println("Flask Response: " + response);
                    } else {
                        Serial.printf("Network Task: POST failed, Error Code: %d\n", httpResponseCode);
                        // Consider adding retry logic here if needed for robustness
                    }
                    http.end(); // Close the connection
                } else {
                    Serial.println("Network Task Warn: WiFi disconnected. Cannot send sensor data.");
                    // Data is lost if WiFi is down when dequeued.
                    // Could implement re-queuing or saving to SPIFFS/SD if needed.
                }

                // --- CRITICAL: Free the memory ---
                // Free the memory that was allocated in uartReceiveTask
                // This prevents memory leaks.
                free(dataToPost);
                dataToPost = NULL; // Avoid dangling pointer issues
                // Serial.println("Network Task: Freed memory for sent data."); // Debug

            } else {
                 Serial.println("Network Task Error: Received NULL pointer from queue.");
            }
        }
        // No vTaskDelay needed here because xQueueReceive blocks the task
        // until data is available, effectively yielding the CPU.
    }
}

// =========================================================================
// SETUP FUNCTION
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("Booting ESP32-CAM...");

    // Initialize Serial1 for Arduino communication
    // Baud rate should match Arduino's Serial.begin() rate (e.g., 9600)
    Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
    Serial.println("Serial1 initialized for Arduino communication (RXD2=" + String(RXD2) + ", TXD2=" + String(TXD2) + ")");

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.println("Flask Sensor URL: " + sensorPostUrl);


    // Initialize Camera
    setupCamera();

    // Create the queue for passing sensor data between tasks
    // Stores pointers to dynamically allocated char arrays
    sensorDataQueue = xQueueCreate(SENSOR_DATA_QUEUE_LENGTH, sizeof(char*));
    if (sensorDataQueue == NULL) {
        Serial.println("FATAL ERROR: Could not create sensor data queue!");
        ESP.restart(); // Cannot proceed without the queue
    } else {
        Serial.println("Sensor data queue created successfully.");
    }

    // Create RTOS Tasks
    // Core 0: Often better for non-WiFi tasks like UART
    // Core 1: Often preferred for WiFi/Network tasks
    Serial.println("Creating RTOS tasks...");
    xTaskCreatePinnedToCore(
        uartReceiveTask,      // Task function
        "UART Receive Task",  // Name for debugging
        4096,                 // Stack size (adjust if needed)
        NULL,                 // Task input parameter
        1,                    // Priority (lower numbers are lower priority)
        &TaskUARTReceiveHandle, // Task handle
        0);                   // Core ID (0)

    xTaskCreatePinnedToCore(
        networkSendTask,      // Task function
        "Network Send Task",  // Name for debugging
        8192,                 // Stack size (HTTPClient needs more stack)
        NULL,                 // Task input parameter
        1,                    // Priority
        &TaskNetworkSendHandle, // Task handle
        1);                   // Core ID (1)

    // Setup WebServer routes
    startCameraServer(); // For /stream endpoint
    setupControlRoute(); // For /control endpoint
    server.begin();      // Start the HTTP server
    Serial.println("HTTP server started on port " + String(81));

    Serial.println("Setup complete. System running.");
}

// =========================================================================
// MAIN LOOP - Keep it minimal and responsive
// =========================================================================
void loop() {
    // Handle incoming web client requests (for /stream and /control)
    // This is non-blocking when used correctly with the WebServer library
    server.handleClient();

    // Briefly yield CPU time to allow other tasks (like UART/Network) to run
    // This prevents the loop from hogging the CPU if handleClient is very fast
    vTaskDelay(10 / portTICK_PERIOD_MS); // 10ms delay
}


