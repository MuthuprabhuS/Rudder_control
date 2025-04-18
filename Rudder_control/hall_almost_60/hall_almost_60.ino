#define HALL_A 2  // HA connected to pin 2 (interrupt)
#define HALL_B 3  // HB connected to pin 3
#define LED 4     // LED on pin 4 (turn OFF if angle > ±60°)

volatile int position = 0;  
const float pulsesPerRevolution = 14.0;  // 7 pairs of N-S poles → 14 pulses per full rotation
const float degreesPerPulse = 360.0 / pulsesPerRevolution; // ~25.71° per pulse

unsigned long lastPulseTime = 0;  
float estimatedAngle = 0;   
float lastAngle = 0;    

// Low-Pass Filter Constants (For Stability)
float alpha = 0.3; // Smoothing factor (0 = no smoothing, 1 = full change)

void setup() {
    pinMode(HALL_A, INPUT);
    pinMode(HALL_B, INPUT);
    pinMode(LED, OUTPUT);
    Serial.begin(115200);

    attachInterrupt(digitalPinToInterrupt(HALL_A), hallSensorISR, CHANGE);

    // Ensure initial position & angle are set to 0
    position = 0;  
    estimatedAngle = 0;  
    lastAngle = 0;  
    lastPulseTime = micros();  // Reset time tracking

    Serial.println("System Initialized. Starting at 0 degrees.");
}

void loop() {
    // Time since last pulse in microseconds
    unsigned long timeSinceLastPulse = micros() - lastPulseTime;

    // Dynamic interpolation factor (based on speed)
    float interpolationFactor = timeSinceLastPulse / 50000.0;  

    // Limit interpolation to prevent overshooting
    if (interpolationFactor > 1.0) interpolationFactor = 1.0;

    // Expected angle based on position
    float targetAngle = position * degreesPerPulse;

    // Ensure correct interpolation for negative positions
    if (position == 0 && lastAngle == 0) {
        estimatedAngle = 0;  // Force the first value to be exactly 0°
    } else {
        // Apply interpolation with improved handling for negative values
        estimatedAngle = (1 - alpha) * lastAngle + alpha * (targetAngle + (degreesPerPulse * interpolationFactor));
    }

    // Fix small rounding errors near zero
    if (abs(estimatedAngle) < 0.01) {
        estimatedAngle = 0.00;
    }

    // Keep angle within -360° to +360°
    if (estimatedAngle > 360.0) estimatedAngle -= 360.0;  
    if (estimatedAngle < -360.0) estimatedAngle += 360.0;  

    // LED Control for ±60° limit
    digitalWrite(LED, (abs(estimatedAngle) > 60.0) ? LOW : HIGH);

    // Print interpolated values
    Serial.print("Position: ");
    Serial.print(position);
    Serial.print(" | Interpolated Angle: ");
    Serial.print(estimatedAngle, 2);  // 2 decimal places
    Serial.println(" degrees");

    lastAngle = estimatedAngle; // Store last angle for filtering
    delay(10);
}

// ISR for Hall sensor (Handles rotation direction)
void hallSensorISR() {
    bool ha = digitalRead(HALL_A);
    bool hb = digitalRead(HALL_B);

    if (ha == hb) {
        position++;  // Clockwise (angle increases)
    } else {
        position--;  // Counterclockwise (angle decreases)
    }

    lastPulseTime = micros();  // Reset last pulse timestamp
}
