// Define the GPIO pins to use for Serial1 (UART1) communication
// RXD1 is the pin where the ESP32 receives data (connect to Arduino TX via level shifter/divider)
// TXD1 is the pin where the ESP32 transmits data (connect to Arduino RX via level shifter, if needed)
// *** Using pins other than GPIO 1 and 3 is strongly recommended ***
#define RXD1_PIN 3 // Example: Using GPIO 16 for Serial1 RX
#define TXD1_PIN 1 // Example: Using GPIO 17 for Serial1 TX

void setup() {
  // Initialize the primary Serial port (UART0) for debugging output
  // Set your Serial Monitor baud rate to this value (115200)
  Serial.begin(115200);
  while (!Serial); // Optional: wait for serial monitor connection
  Serial.println("ESP32 Serial1 Receiver Initialized.");
  Serial.print("Monitoring on Serial at 115200 baud.\n");
  Serial.print("Listening for Arduino data on Serial1 (RX Pin: ");
  Serial.print(RXD1_PIN);
  Serial.println(") at 9600 baud.");

  // Initialize Serial1 (UART1) with specific pins and baud rate
  // 9600: Baud rate (must match the sending Arduino)
  // SERIAL_8N1: Standard data format (8 data bits, No parity, 1 stop bit)
  // RXD1_PIN, TXD1_PIN: The GPIO pins assigned to Serial1 RX and TX
  Serial1.begin(9600, SERIAL_8N1, RXD1_PIN, TXD1_PIN);
}

void loop() {
  // Check if any data has arrived on Serial1
  if (Serial1.available() > 0) {
    // Read the incoming data until a newline character ('\n') is received
    String receivedData = Serial1.readStringUntil('\n');

    Serial1.print(receivedData);
    Serial1.println("        Received");
  }

  // You can add a small delay here if needed, but it's often not necessary
  // delay(1);
}
