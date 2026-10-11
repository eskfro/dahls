
void setup() {
  /*
  LED connected to pins 3-8 blinking sequentially
  */
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 3; i < 9; i++) {
    pinMode(i, OUTPUT);
  }
  Serial.begin(9600);

}

int should_light(int light) {
  if (Serial.available() == 0) return light;

  const char c = Serial.read();

  if (c == '1' && !light) {
    Serial.println("Start signal, turning on light");
    return 1;
  }

  if (c == '0' && light) {
    Serial.println("Stop signal, turning off light");
    return 0;
  }

  return light;
  
}

void loop() {
  const int kPeriodMs = 400;
  static int light = 1;

  light = should_light(light);

  for (int i = 3; i < 9; i++) {
    if (light) {
      digitalWrite(i, HIGH);
      Serial.println(i);
    }
    delay(kPeriodMs);
    digitalWrite(i, LOW);
  }
}

