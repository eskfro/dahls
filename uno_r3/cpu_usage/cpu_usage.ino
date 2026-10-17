/*
CPU usage

Shows the PC cpu usage as a bar graph on do 4-13.
The PC sends the usage in percent (0-100) as a text
line every second, see scripts/cpu_usage_to_serial.sh.
*/
const int do_start = 4;
const int n_pins = 10;

void show_usage(int percent) {
    int n_on = (percent * n_pins + 50) / 100;
    for (int i = 0; i < n_pins; i++) {
        digitalWrite(do_start + i, i < n_on ? HIGH : LOW);
    }
}

// === Setup ===

void setup() {
    for (int i = 0; i < n_pins; i++) {
        pinMode(do_start + i, OUTPUT);
    }

    Serial.begin(9600);
}

// === Main loop ===

void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        show_usage(constrain(line.toInt(), 0, 100));
    }
}
