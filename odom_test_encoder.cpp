volatile unsigned long encoderCount = 0;
const int encoderPin = 13;

const int pulsesPerRevolution = 30;       // You said 30 teeth
const unsigned long stopTimeout = 500;    // ms to detect when wheel stops
const unsigned long debounceDelay = 2;    // Ignore signals within 2ms = debounce

unsigned long lastPulseTime = 0;
unsigned long lastDebounceTime = 0;

void IRAM_ATTR countPulse() {
  unsigned long now = millis();

  // Debounce: allow only if 2 ms have passed
  if (now - lastDebounceTime > debounceDelay) {
    encoderCount++;
    lastPulseTime = now;
    lastDebounceTime = now;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);
}

void loop() {
  static unsigned long lastCount = 0;

  if (encoderCount != lastCount) {
    float rotations = (float)encoderCount / (pulsesPerRevolution*2);

    Serial.print("Pulse Count: ");
    Serial.print(encoderCount);
    Serial.print(" | Rotations: ");
    Serial.println(rotations, 2);  // show 2 decimal places

    lastCount = encoderCount;
  }

  // Reset when stopped
  if (millis() - lastPulseTime > stopTimeout && encoderCount > 0) {
    Serial.println("Wheel stopped. Resetting count.");
    encoderCount = 0;
    lastCount = 0;
  }

  delay(10);
}
