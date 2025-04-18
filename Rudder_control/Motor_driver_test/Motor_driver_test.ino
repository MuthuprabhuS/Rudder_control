// Motor pins
#define IN1 7   // Connect to IN1 on L298N
#define IN2 8   // Connect to IN2 on L298N
#define ENA 9   // Connect to ENA (PWM) on L298N

void setup() {
  // Set motor control pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  Serial.begin(9600);
  Serial.println("Motor Test Starting...");
}

void loop() {
  Serial.println("Rotating FORWARD...");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200); // Speed (0-255)
  delay(2000);           // Run for 2 seconds

  Serial.println("Stopping...");
  analogWrite(ENA, 0);   // Stop motor
  delay(1000);

  Serial.println("Rotating BACKWARD...");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 200); // Speed (0-255)
  delay(2000);           // Run for 2 seconds

  Serial.println("Stopping...");
  analogWrite(ENA, 0);   // Stop motor
  delay(2000);
}
