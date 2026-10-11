/*
    uno      | stepper controller
    do 8    -> di 1
    do 9    -> di 2    
    do 10   -> di 3
    do 11   -> di 4
*/
const int start = 8;
const int num_pins = 4;
const int steps_per_rev = 2048;

const byte sequence[4][4] {
    {1, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 1},
    {1, 0, 0, 1}
};

void set_coils(int s) {
    for (int i = 0; i < num_pins; i++) {
        digitalWrite(start + i, sequence[s][i]);
    }
}

int curr_step = 0;
int btn_right = 0;
int btn_left = 0;

void step_motor(int steps, int delay_ms) {
    int dir = (steps > 0) ? 1 : -1;
    for (int i = 0; i < abs(steps); i++) {
        curr_step = (curr_step + dir + 4) % 4;
        set_coils(curr_step);
        delay(delay_ms);
    }
}

void step_motor_cw(int delay_ms) {
    const int dir = 1;
    curr_step = (curr_step + dir + 4) % 4;
    set_coils(curr_step);
    delay(delay_ms);
}

void step_motor_cc(int delay_ms) {
    const int dir = -1;
    curr_step = (curr_step + dir + 4) % 4;
    set_coils(curr_step);
    delay(delay_ms);
}

void release_motor() {
    for (int i = 0; i < num_pins; i++) {
        digitalWrite(start + i, LOW);
    }
}

void update_btn_inputs() {
    btn_right =  digitalRead(3);
    btn_left = digitalRead(4);
}

int potensiometer_voltage = 0;
int step_delay_ms = 3;

int inrange(int val, int lo, int hi) {
    int res = (val >= lo && val <= hi) ? 1 : 0;
    return res;
}

int get_step_delay_ms() {
    
    potensiometer_voltage = analogRead(1);

    if (inrange(potensiometer_voltage, 0, 256)) {
        return 6;
    } else if (inrange(potensiometer_voltage, 256, 512)) {
        return 5;
    } else if (inrange(potensiometer_voltage, 512, 768)) {
        return 4;
    } else {
        return 3;
    }

}

// === Setup === 

void setup() {
    for (int i = 0; i < num_pins; i++) {
        pinMode(start + i, OUTPUT);
    }
    pinMode(3, INPUT);
    pinMode(4, INPUT);
}

// === Main loop ===

void loop() {

    update_btn_inputs();

    step_delay_ms = get_step_delay_ms();

    if (btn_right) {
        step_motor_cw(step_delay_ms);
    } else if (btn_left) {
        step_motor_cc(step_delay_ms);
    } else {
        release_motor();
    }

    
}