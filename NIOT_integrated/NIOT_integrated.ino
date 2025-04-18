#define HALL_A 2
#define HALL_B 3
#define IN1 7
#define IN2 8
#define ENA 9

volatile int position = 0;
const float pulsesPerRevolution = 14.0;
const float degreesPerPulse = 360.0 / pulsesPerRevolution;

unsigned long lastPulseTime = 0;
float estimatedAngle = 0;
float lastAngle = 0;
float hallAlpha = 0.3;

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
SoftwareSerial gpsSerial(10, 11);
TinyGPSPlus gps;

double destinationLat = 0.0;
double destinationLon = 0.0;
double currentLat = 0.0;
double currentLon = 0.0;

#include <Wire.h>
#include <MPU6050_tockn.h>
MPU6050 mpu6050(Wire);

const float alpha = 0.98;
float pitch = 0, roll = 0, yaw = 0;
unsigned long prevTime = 0;

const float L = 1.5;
const float V = 0.35;
const float Cr = 0.3;
const float Iz = 0.3279;
const float maxRudderAngle = 60.0;

void setup() {
    pinMode(HALL_A, INPUT);
    pinMode(HALL_B, INPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);

    Serial.begin(115200);
    gpsSerial.begin(9600);

    attachInterrupt(digitalPinToInterrupt(HALL_A), hallSensorISR, CHANGE);
    position = 0;
    estimatedAngle = 0;
    lastAngle = 0;
    lastPulseTime = micros();

    Serial.println("System Initialized. Starting at 0 degrees.");

    Wire.begin();
    mpu6050.begin();
    Serial.println("Calibrating gyroscope... Keep the sensor still.");
    mpu6050.calcGyroOffsets(true);
    Serial.println("Calibration complete.");

    prevTime = millis();

    Serial.println("GPS Data Processing Started...");
    Serial.println("Enter Destination Latitude and Longitude in format: lat lon");
}

void loop() {
    while (gpsSerial.available()) gps.encode(gpsSerial.read());

    if (Serial.available()) {
        float lat = Serial.parseFloat();
        float lon = Serial.parseFloat();
        if (lat != 0.0 && lon != 0.0) {
            destinationLat = lat;
            destinationLon = lon;
            Serial.print("Destination Set: ");
            Serial.print(destinationLat, 6);
            Serial.print(", ");
            Serial.println(destinationLon, 6);
        } else {
            Serial.println("Invalid destination coordinates.");
        }

        while (Serial.available()) Serial.read();
    }

    if (gps.location.isUpdated()) {
        currentLat = gps.location.lat();
        currentLon = gps.location.lng();

        Serial.print("Current Position: ");
        Serial.print(currentLat, 6); Serial.print(", ");
        Serial.println(currentLon, 6);

        if (destinationLat != 0.0 && destinationLon != 0.0) {
            double bearing = calculateBearing(currentLat, currentLon, destinationLat, destinationLon);
            double distance = calculateDistance(currentLat, currentLon, destinationLat, destinationLon);

            Serial.print("Bearing Angle to Destination: ");
            Serial.println(bearing, 2);

            Serial.print("Distance to Destination (m): ");
            Serial.println(distance, 2);
        }

        Serial.println("--------------------------");
    }

    if (destinationLat != 0.0 && destinationLon != 0.0 && currentLat != 0.0 && currentLon != 0.0) {
        mpu6050.update();

        unsigned long currentTime = millis();
        float deltaTime = (currentTime - prevTime) / 1000.0;
        prevTime = currentTime;

        float accX = mpu6050.getAccX();
        float accY = mpu6050.getAccY();
        float accZ = mpu6050.getAccZ();
        float gyroX = mpu6050.getGyroX();
        float gyroY = mpu6050.getGyroY();
        float gyroZ = mpu6050.getGyroZ();

        float accPitch = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180 / PI;
        float accRoll  = atan2(-accX, accZ) * 180 / PI;

        pitch = alpha * (pitch + gyroX * deltaTime) + (1 - alpha) * accPitch;
        roll  = alpha * (roll  + gyroY * deltaTime) + (1 - alpha) * accRoll;
        yaw  += gyroZ * deltaTime;

        if (yaw < 0) yaw += 360;
        if (yaw >= 360) yaw -= 360;

        Serial.print(" Yaw: "); Serial.print(yaw, 2);

        unsigned long timeSinceLastPulse = micros() - lastPulseTime;
        float interpolationFactor = timeSinceLastPulse / 50000.0;
        if (interpolationFactor > 1.0) interpolationFactor = 1.0;

        float targetAngle = position * degreesPerPulse;

        if (position == 0 && lastAngle == 0) {
            estimatedAngle = 0;
        } else {
            estimatedAngle = (1 - hallAlpha) * lastAngle + hallAlpha * (targetAngle + (degreesPerPulse * interpolationFactor));
        }

        if (abs(estimatedAngle) < 0.01) estimatedAngle = 0.00;
        if (estimatedAngle > 360.0) estimatedAngle -= 360.0;
        if (estimatedAngle < -360.0) estimatedAngle += 360.0;

        double bearing = calculateBearing(currentLat, currentLon, destinationLat, destinationLon);
        float yawDiff = bearing - yaw;
        if (yawDiff > 180) yawDiff -= 360;
        if (yawDiff < -180) yawDiff += 360;

        float Kp = 1.2;
        float rudderAngleDeg = constrain(Kp * yawDiff, -maxRudderAngle, maxRudderAngle);

        float yawTolerance = 5.0;
        float rudderTolerance = 1.5;

        bool yawAligned = abs(yawDiff) <= yawTolerance;
        bool rudderReached = abs(estimatedAngle - rudderAngleDeg) <= rudderTolerance;
        bool angleLimitReached = abs(estimatedAngle) >= maxRudderAngle;

        if (yawAligned || rudderReached || angleLimitReached) {
            analogWrite(ENA, 0);
        } else {
            analogWrite(ENA, map(abs(rudderAngleDeg), 0, maxRudderAngle, 0, 255));

            if (rudderAngleDeg > estimatedAngle) {
                digitalWrite(IN1, HIGH);
                digitalWrite(IN2, LOW);
            } else {
                digitalWrite(IN1, LOW);
                digitalWrite(IN2, HIGH);
            }
        }

        Serial.print(" | Rudder Cmd: "); Serial.print(rudderAngleDeg, 2);
        Serial.print(" deg | YawDifference: "); Serial.print(yawDiff);
        Serial.print(" | Shaft Angle: "); Serial.print(estimatedAngle, 2);
        Serial.print(" | RudderError: "); Serial.print(estimatedAngle - rudderAngleDeg, 2);
        Serial.println(" deg");

        lastAngle = estimatedAngle;
    }
}

void hallSensorISR() {
    bool ha = digitalRead(HALL_A);
    bool hb = digitalRead(HALL_B);

    if (ha == hb) {
        position++;
    } else {
        position--;
    }

    lastPulseTime = micros();
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    lat1 = radians(lat1);
    lon1 = radians(lon1);
    lat2 = radians(lat2);
    lon2 = radians(lon2);

    double dLon = lon2 - lon1;
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    double bearing = atan2(y, x);
    bearing = degrees(bearing);
    return (bearing < 0) ? bearing + 360.0 : bearing;
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000;
    lat1 = radians(lat1);
    lon1 = radians(lon1);
    lat2 = radians(lat2);
    lon2 = radians(lon2);

    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;

    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);

    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}
