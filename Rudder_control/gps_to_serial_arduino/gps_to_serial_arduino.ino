#include <SoftwareSerial.h>

SoftwareSerial gpsSerial(10, 11);    // GPS: RX=D2, TX=D3
SoftwareSerial rs232Serial(6, 7);  // RS232: TX=D6, RX=D7

void setup() {
    Serial.begin(9600);        // Arduino Serial Monitor (COM6)
    gpsSerial.begin(9600);     // GPS Baud Rate
    rs232Serial.begin(9600);   // RS232 Baud Rate

    Serial.println("GPS NMEA Output Started...");
}

void loop() {
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        Serial.write(c);       // Display in Arduino Serial Monitor (COM6)
        rs232Serial.write(c);  // Send to RS232 (Tera Term via COM8)
    }
}