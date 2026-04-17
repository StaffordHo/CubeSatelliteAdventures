#include <SPI.h>
#include <LoRa.h>

#define RFM95_INT 2
#define RFM95_RST 9
#define RFM95_CS 10
#define LED_B1 4
#define LED_B2 6
#define LED_R 8

String secretKey = "STAFFORDPWROX";
String inputmsg;

uint8_t GSconnected = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(LED_B1, OUTPUT);
  pinMode(LED_B2, OUTPUT);
  pinMode(LED_R, OUTPUT);
  
  // Fancy startup LED sequence
  digitalWrite(LED_B1, HIGH); delay(100); digitalWrite(LED_B1, LOW);
  digitalWrite(LED_B2, HIGH); delay(100); digitalWrite(LED_B2, LOW);
  digitalWrite(LED_R, HIGH);  delay(100); digitalWrite(LED_R, LOW);
  
  setupLora();
  
  Serial.print("Ground ");
  Serial.print(secretKey);
  Serial.println(" Initialized...");
}

void loop() {
  // Listen for you typing commands in the PC Serial Monitor
  if (Serial.available()) {
    inputmsg = secretKey;
    inputmsg += Serial.readStringUntil('\n');
    inputmsg.trim();
    inputmsg.toUpperCase(); // Forces commands to uppercase (ALL, IMU, etc)
    transmitLora();
    inputmsg = "";
  }

  // Always be listening to space
  receiveLora();
}

// ---------------------------------------------------------
// OTHER FUNCTIONS
// ---------------------------------------------------------

void setupLora() {
  if (!LoRa.begin(434E6)) { // Ensure this matches your OBC frequency! 
    Serial.println("Starting LoRa failed!");
  } else {
    Serial.println("LoRa radio init OK!");
  }
}

void transmitLora() {
  if (inputmsg != "<STATION70>CONN_REQUEST") {
    Serial.print("Sending packet: ");
    Serial.println(inputmsg);
  }

  // Broadcast packet into the void
  LoRa.beginPacket();
  LoRa.print(inputmsg);
  LoRa.endPacket();
}

void receiveLora() {
  int packetSize = LoRa.parsePacket();
  
  if (packetSize) {
    String LoRaData = "";

    // Read packet from the radio buffer
    while (LoRa.available()) {
      LoRaData = LoRaData + (char)LoRa.read();
    }

    // CHECK 1: Does it contain our Secret Password? (Telemetry data)
    if (LoRaData.substring(0, secretKey.length()) == secretKey) {
      
      // Strip the password so it looks pretty for the user
      LoRaData.remove(0, secretKey.length());
      
      Serial.print("Received packet '");
      Serial.print(LoRaData);
      Serial.print("' with RSSI ");
      Serial.println(LoRa.packetRssi());
      
    } 
    // CHECK 2: Does it lack a password? (This means it is raw Camera Data!)
    else {
      // Just print the raw bytes out to the Serial monitor directly
      // This protects your photos from being destroyed by the password checker
      Serial.print(LoRaData);
    }
  }
}

void msgConnected() {
  Serial.println("Receiver <STATION70> connected");
  GSconnected = 1; // Fixed the == typo that existed in your old code!
}
