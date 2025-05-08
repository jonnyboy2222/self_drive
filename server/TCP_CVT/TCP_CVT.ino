#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <ArduinoJson.h>
#include <WiFiClient.h>

// --- WiFi Configuration ---
const char *ssid = "addinedu_class_1(2.4G)";
const char *password = "addinedu1";

IPAddress staticIP(192, 168, 2, 245); // Should be outside the DHCP range (pick 20 <= num or 200 <= num)
IPAddress gateway(192, 168, 2, 1);    // Router's IP (Gateway)
IPAddress subnet(255, 255, 255, 0);   // Standard subnet mask

// --- Python TCP Server Configuration (from tcp.py) ---
const char* python_server_ip = "192.168.2.234";
const uint16_t python_server_port = 12345;


// --- UART Pin Configuration (for Arduino communication) ---
#define RXD2 3 // connected to Arduino TX
#define TXD2 1 // connected to Arduino RX

// --- RTOS Configuration ---
// pointer to a task to pause, resume, kill the task
TaskHandle_t TaskUARTReceiveHandle;
TaskHandle_t TaskNetworkSendHandle;
QueueHandle_t sensorDataQueue;

#define SENSOR_DATA_QUEUE_LENGTH 10 // How many sensor readings can be queued
#define SENSOR_DATA_MAX_LEN 128     // Max expected length of a sensor data string + null terminator
#define SENSOR_DATA_QUEUE_ITEM_SIZE SENSOR_DATA_MAX_LEN // Size of each item stored in the queue

// --- Camera Configuration (AI-Thinker Model) ---
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
void uartReceiveTask(void *parameter);
void sendRfidDataViaTCP(String rfidTagJson);
void networkSendTask(void *parameter);


// =========================================================================
// TCP CLIENT: Send RFID Data and Receive Verification
// =========================================================================

// To send RFID data
void sendRfidDataViaTCP(String rfidTagJson) {
    if (WiFi.status() == WL_CONNECTED) {
        WiFiClient client;
        // Serial.print("TCP: Attempting to connect to Python server for RFID check: ");
        // Serial.print(python_server_ip);
        // Serial.print(":");
        // Serial.println(python_server_port);

        if (!client.connect(python_server_ip, python_server_port)) {
            // Serial.println("TCP: RFID check connection failed!");
            return;
        }
        // Serial.println("TCP: Connected to Python server for RFID check.");

        // Create JSON payload
        String jsonPayload = rfidTagJson;

        // Serial.print("Sending RFID check: ");
        // Serial.println(jsonPayload);

        client.println(jsonPayload); // Send the JSON string, println adds \r\n

        // Wait for the response from the Python server ("PASS\n" or "FAIL\n")
        unsigned long startTime = millis();
        while (client.available() == 0) {
            if (millis() - startTime > 5000) { // 5 second timeout
                // Serial.println("TCP: Timeout waiting for RFID verification response!");
                client.stop();
                return;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        char buffer[16]; // Make sure buffer is large enough for "FAIL\0" or "PASS\0"
        int bytesRead = client.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
        String verificationResponse = ""; // Initialize

        if (bytesRead > 0) {
            buffer[bytesRead] = '\0'; // Null-terminate the C-string
            verificationResponse = String(buffer); // Convert C-string to Arduino String
        }
        verificationResponse.trim();


        // Serial.print("TCP: RFID Verification Response from Python: '");
        // Serial.print(verificationResponse);
        // Serial.println("'");

        // Forward the PASS/FAIL signal to Arduino via Serial1
        if (verificationResponse.length() > 0) {
            Serial1.println(verificationResponse);
            // Serial.println("TCP: Forwarded verification response to Arduino: '" + verificationResponse + "'");
        } 
        else {
        }
        client.stop();
    } 
    else {
        // Serial.println("WiFi disconnected, cannot send RFID check.");
    }
}


// =========================================================================
// RTOS TASK: UART Receive (Read from Arduino) - MODIFIED
// =========================================================================
void uartReceiveTask(void *parameter) {
    // Serial.println("UART Receive Task started on Core " + String(xPortGetCoreID()));
    // Buffer to hold data read from Serial1
    char receivedData[SENSOR_DATA_MAX_LEN]; // 128
    // Buffer to hold the data item being sent to the queue
    char dataToSend[SENSOR_DATA_QUEUE_ITEM_SIZE]; // 128

    while (true) {
        if (Serial1.available()) {
            // Read the incoming data line (until newline or buffer full)
            size_t len = Serial1.readBytesUntil('\n', receivedData, SENSOR_DATA_MAX_LEN - 1);

            if (len > 0) {
                receivedData[len] = '\0'; // Null-terminate the string
                String sensorDataStr(receivedData); // Create String object
                sensorDataStr.trim(); // Remove potential whitespace

                if (sensorDataStr.length() > 0) {
                    // Attempt to parse as JSON to check for RFID tag
                    StaticJsonDocument<SENSOR_DATA_MAX_LEN> doc; // Use SENSOR_DATA_MAX_LEN for buffer
                    DeserializationError error = deserializeJson(doc, sensorDataStr);

                    if (!error) {
                        if (doc.containsKey("rfid_tag")) {
                            // Serial.println("UART Received RFID data: " + sensorDataStr);
                            sendRfidDataViaTCP(sensorDataStr); // Pass the original JSON string    
                        }
                        else {
                            if (sensorDataStr.length() < SENSOR_DATA_QUEUE_ITEM_SIZE) {
                                // Serial.println("UART Received other sensor data: " + sensorDataStr);
                                // --- Queue the data for the Network Task ---
                                strncpy(dataToSend, sensorDataStr.c_str(), SENSOR_DATA_QUEUE_ITEM_SIZE - 1);
                                dataToSend[SENSOR_DATA_QUEUE_ITEM_SIZE - 1] = '\0'; // Ensure null termination

                                if (xQueueSend(sensorDataQueue, dataToSend, (TickType_t)10) == pdPASS) {
                                    // Serial.println("Other sensor data queued for network sending.");
                                } else {
                                    // Serial.println("UART Warn: Failed to queue other sensor data (queue full?). Discarding.");
                                }
                            } else {
                                // Serial.println("UART Error: Received other sensor data exceeds queue item size! Discarding.");
                            }
                        }
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
// RTOS TASK: Network Send (Send sensor data to Python TCP Server)
// =========================================================================
void networkSendTask(void *parameter) {
    // Serial.println("Network Send Task started on Core " + String(xPortGetCoreID()));
    // Fixed-size buffer to receive data from the queue
    char dataToPost[SENSOR_DATA_QUEUE_ITEM_SIZE];
    const int MAX_TCP_SEND_RETRIES = 3; // Renamed for clarity
    const int TCP_RETRY_DELAY_MS = 2000;   // Renamed for clarity

    while (true) {
        // Wait indefinitely until an item is available in the queue. (portMAX_DELAY)
        // The queue item (data) will be copied into the dataToPost buffer.
        if (xQueueReceive(sensorDataQueue, dataToPost, portMAX_DELAY) == pdPASS) {

            // Serial.println("Network Task: Dequeued data for sending: " + String(dataToPost));

            // Check WiFi connection before attempting POST
            if (WiFi.status() == WL_CONNECTED) {
                WiFiClient client;
                bool sendSuccess = false;

                for (int attempt = 0; attempt < MAX_TCP_SEND_RETRIES; attempt++) {
                    // Serial.print("Network Task: Attempting TCP send (try ");
                    // Serial.print(attempt + 1);
                    

                    // Serial.print(" of " + String(MAX_HTTP_RETRIES) + ") to ");
                    // Serial.print(python_server_ip); Serial.print(":"); Serial.println(python_server_port);

                    if (!client.connect(python_server_ip, python_server_port)) {
                        // Serial.println("Network Task: TCP connection failed on attempt " + String(attempt + 1));
                        if (attempt < MAX_TCP_SEND_RETRIES - 1) {
                            vTaskDelay(TCP_RETRY_DELAY_MS / portTICK_PERIOD_MS);
                        }
                        continue; // Try to connect again
                    }

                    // Serial.println("Network Task: Connected. Sending data: " + String(dataToPost));
                    if (client.println(String(dataToPost))) { // println adds \r\n
                        // Serial.print("Network Task: TCP send successful on attempt ");
                        // Serial.print(attempt + 1);
                        // Serial.println(".");
                        sendSuccess = true;
                        client.stop(); // Close connection
                        break;      // Exit retry loop on success
                    } else {
                        // Serial.print("Network Task: TCP send failed on attempt ");
                        // Serial.print(attempt + 1);
                        // Serial.println(".");
                        client.stop(); // Close connection before retrying
                        if (attempt < MAX_TCP_SEND_RETRIES - 1) { // no delay after the last attempt
                            // Serial.println("Network Task: Waiting before retry...");
                            vTaskDelay(TCP_RETRY_DELAY_MS / portTICK_PERIOD_MS);
                        }
                    }
                } // End of retry loop

                if (!sendSuccess) {
                    // Serial.println("Network Task: All POST retries failed. Discarding data: " + String(dataToPost));
                }
            } else {
                // Serial.println("Network Task Warn: WiFi disconnected. Cannot send sensor data.");
                // Data is lost if WiFi is down when dequeued.
                // Could implement re-queuing or saving to SPIFFS/SD if needed.
                // Add a delay if WiFi is down to prevent rapid queue processing and log spam
                vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait 5 seconds before checking queue again
            }
             // --- No need to free memory here! ---
             // The dataToPost buffer is on the stack and managed automatically.

        }
        // No vTaskDelay needed here because xQueueReceive blocks the task
        // until data is available, effectively yielding the CPU.
    }
}

// =========================================================================
// SETUP FUNCTION
// =========================================================================
void setup() {
    // Serial.begin(115200);
    // Serial.println("Booting ESP32-CAM...");

    // Initialize Serial1 for Arduino communication
    // Baud rate should match Arduino's Serial.begin() rate (e.g., 9600)
    Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
    // Serial.println("Serial1 initialized for Arduino communication (RXD2=" + String(RXD2) + ", TXD2=" + String(TXD2) + ")");

    // Connect to Wi-Fi
    WiFi.config(staticIP, gateway, subnet);
    WiFi.begin(ssid, password);
    // Serial.print("Connecting to WiFi ");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        // Serial.print(".");
    }
    // Serial.println("\nWiFi connected");
    // Serial.print("IP Address: ");
    // Serial.println(WiFi.localIP());
    
    // Serial.print("Python TCP Server: ");
    // Serial.print(python_server_ip); Serial.print(":"); Serial.println(python_server_port);


    // Create the queue for passing sensor data between tasks
    // Stores fixed-size character arrays (copies of the data) // 10, 128
    sensorDataQueue = xQueueCreate(SENSOR_DATA_QUEUE_LENGTH, SENSOR_DATA_QUEUE_ITEM_SIZE);
    if (sensorDataQueue == NULL) {
        // Serial.println("FATAL ERROR: Could not create sensor data queue!");
        ESP.restart(); // Cannot proceed without the queue
    } else {
        // Serial.println("Sensor data queue created successfully.");
    }

    // Create RTOS Tasks
    // Core 0: Often better for non-WiFi tasks like UART
    // Core 1: Often preferred for WiFi/Network tasks
    // Serial.println("Creating RTOS tasks...");
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

    

    // Serial.println("Setup complete. System running.");
}

// =========================================================================
// MAIN LOOP - Keep it minimal and responsive
// =========================================================================
void loop() {
    // Briefly yield CPU time to allow other tasks (like UART/Network) to run
    // This prevents the loop from hogging the CPU if handleClient is very fast
    vTaskDelay(10 / portTICK_PERIOD_MS); // 10ms delay
}
