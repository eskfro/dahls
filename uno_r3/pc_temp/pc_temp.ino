/*
PC temperature

Shows the PC cpu temperature as a bar graph on do 4-13
and the fan speeds as blinking leds on do 2 and 3, faster
blinking means faster fan. The PC sends "temp fan1 fan2"
as a text line every second, temp in C and fans in percent,
see scripts/pc_temp_to_serial.sh.
*/
const int do_temp_start = 4;
const int n_temp_pins = 10;
const int temp_min = 30;
const int temp_max = 80;

const int n_fans = 2;
const int do_fans[n_fans] = {2, 3};

int fan_percent[n_fans] = {0, 0};
unsigned long fan_last_toggle[n_fans] = {0, 0};

void show_temp(int temp) {
    int n_on = (temp - temp_min) * n_temp_pins / (temp_max - temp_min);
    n_on = constrain(n_on, 0, n_temp_pins);
    for (int i = 0; i < n_temp_pins; i++) {
        digitalWrite(do_temp_start + i, i < n_on ? HIGH : LOW);
    }
}

void update_fans() {
    for (int i = 0; i < n_fans; i++) {
        if (fan_percent[i] <= 0) {
            digitalWrite(do_fans[i], LOW);
            continue;
        }

        unsigned long half_period_ms = map(fan_percent[i], 1, 100, 500, 50);
        if (millis() - fan_last_toggle[i] >= half_period_ms) {
            digitalWrite(do_fans[i], !digitalRead(do_fans[i]));
            fan_last_toggle[i] = millis();
        }
    }
}

// === Setup ===

void setup() {
    for (int i = 0; i < n_temp_pins; i++) {
        pinMode(do_temp_start + i, OUTPUT);
    }
    for (int i = 0; i < n_fans; i++) {
        pinMode(do_fans[i], OUTPUT);
    }

    Serial.begin(9600);
}

// === Main loop ===

void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        int temp = 0;
        sscanf(line.c_str(), "%d %d %d", &temp, &fan_percent[0], &fan_percent[1]);
        show_temp(temp);
    }

    update_fans();
}
