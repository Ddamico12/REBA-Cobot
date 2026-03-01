// ultrasonic_dual_sensor.ino
// Reads two HC-SR04-style ultrasonic sensors and prints:
//   start_cm,middle_cm\n
// at a fixed rate over Serial.

static const uint8_t TRIG_START  = 8;
static const uint8_t ECHO_START  = 9;
static const uint8_t TRIG_MIDDLE = 10;
static const uint8_t ECHO_MIDDLE = 11;

static const unsigned long SERIAL_BAUD = 9600;
static const unsigned long SAMPLE_PERIOD_MS = 100;

// Use a timeout so pulseIn can't hang forever if sensor glitches.
static const unsigned long PULSE_TIMEOUT_US = 25000; // ~4m max range (safe cap)

float readDistanceCm(uint8_t trigPin, uint8_t echoPin) {
  // Trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure echo
  unsigned long duration = pulseIn(echoPin, HIGH, PULSE_TIMEOUT_US);

  // If timeout returns 0, treat as "no reading"
  if (duration == 0) return -1.0;

  // Speed of sound ~343 m/s => 0.0343 cm/us
  // Divide by 2 for round trip
  return (duration * 0.0343f) / 2.0f;
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(TRIG_START, OUTPUT);
  pinMode(ECHO_START, INPUT);

  pinMode(TRIG_MIDDLE, OUTPUT);
  pinMode(ECHO_MIDDLE, INPUT);

  digitalWrite(TRIG_START, LOW);
  digitalWrite(TRIG_MIDDLE, LOW);
}

void loop() {
  const float startCm  = readDistanceCm(TRIG_START, ECHO_START);
  const float middleCm = readDistanceCm(TRIG_MIDDLE, ECHO_MIDDLE);

  Serial.print(startCm, 2);
  Serial.print(",");
  Serial.println(middleCm, 2);

  delay(SAMPLE_PERIOD_MS);
}
