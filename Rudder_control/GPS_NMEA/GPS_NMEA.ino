#include <SoftwareSerial.h>

SoftwareSerial gpsSerial(10, 11);  // GPS TX → D10, GPS RX → D11

void setup() {
  Serial.begin(9600);      // USB Serial Monitor
  gpsSerial.begin(9600);   // GPS Module Baud Rate (try 4800 if needed)

  Serial.println("GPS NMEA Output Started...");
}

void loop() {
  while (gpsSerial.available()) {  // Check if GPS is sending data
    char c = gpsSerial.read();
    Serial.write(c);  // Print NMEA data to Serial Monitor (COM6)
  }
}