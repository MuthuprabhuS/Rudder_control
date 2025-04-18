#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

SoftwareSerial gpsSerial(10, 11);  // GPS TX → D10, GPS RX → D11
TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  Serial.println("GPS Data Processing Started...");
  Serial.println("Enter Destination Latitude and Longitude in format: lat lon");

}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    double currentLat = gps.location.lat();
    double currentLon = gps.location.lng();

    Serial.print("Current Position: ");
    Serial.print(currentLat, 6);
    Serial.print(", ");
    Serial.println(currentLon, 6);

    if (destinationLat != 0 && destinationLon != 0) { // If destination is set
      double bearing = calculateBearing(currentLat, currentLon, destinationLat, destinationLon);
      double distance = calculateDistance(currentLat, currentLon, destinationLat, destinationLon);

      Serial.print("Bearing Angle to Destination: ");
      Serial.println(bearing, 2);
      
      Serial.print("Distance to Destination (m): ");
      Serial.println(distance, 2);
    }
    Serial.print("Speed (km/h): ");
    Serial.println(gps.speed.kmph());

    Serial.print("Altitude (m): ");
    Serial.println(gps.altitude.meters());

    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());

    Serial.println("--------------------------");
    delay(5000); 
  }
    if (Serial.available()) {
    destinationLat = Serial.parseFloat();
    destinationLon = Serial.parseFloat();
    Serial.print("Destination Set: ");
    Serial.print(destinationLat, 6);
    Serial.print(", ");
    Serial.println(destinationLon, 6);
  }
}
// Function to calculate the bearing angle
double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  double dLon = lon2 - lon1;
  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  double bearing = atan2(y, x);

  return degrees(bearing);
}

// Function to calculate the distance using Haversine formula
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000; // Earth's radius in meters

  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1) * cos(lat2) * 
             sin(dLon / 2) * sin(dLon / 2);

  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return R * c;  // Distance in meters
}