//A6 Pro Serial GPRS GSM Module Baud Rate Configuration
//Default: 115200 bps -> Set to: 9600 bps

#include <SoftwareSerial.h>

// Connect RX to SIM800 U_TXD
// Connect TX to SIM800 U_RXD
SoftwareSerial gsmSerial(7, 8); 

void setup() {
  Serial.begin(9600);
  delay(100);
  
  Serial.println(F("GSM Baud Rate Setup"));
  Serial.println(F("Default: 115200 -> Target: 9600"));
  Serial.println();
  
  // Try communicating at default baud rate
  gsmSerial.begin(115200);
  delay(200);
  
  // Send AT command to verify connection
  gsmSerial.println("AT");
  delay(300);
  
  // Clear buffer
  while(gsmSerial.available()) { 
    Serial.write(gsmSerial.read());
  }
  
  Serial.println(F("\nSetting baud to 9600..."));
  delay(100);
  
  // Set baud rate to 9600
  gsmSerial.println("AT+IPR=9600");
  delay(500);
  
  // Save configuration
  gsmSerial.println("AT&W");
  delay(500);
  
  // Clear buffer
  while(gsmSerial.available()) { 
    gsmSerial.read();
  }
  
  // Switch to 9600 baud
  gsmSerial.begin(9600);
  delay(200);
  
  Serial.println(F("Testing at 9600..."));
  gsmSerial.println("AT");
  delay(500);
}

void loop() {
  // Echo from module to serial monitor
  if (gsmSerial.available()) {
    Serial.write(gsmSerial.read());
  }
  
  // Echo from serial monitor to module
  if (Serial.available()) {
    gsmSerial.write(Serial.read());
  }
}