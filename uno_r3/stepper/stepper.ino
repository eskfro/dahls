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

int curr_step = 0;

void set_coils(int s) {
    for (int i = 0; i < num_pins; i++) {
        digitalWrite(start + i, sequence[s][i]);
    }
}

void step_motor(int steps, int delay_ms) {
    int dir = (steps > 0) ? 1 : -1;
    for (int i = 0; i < abs(steps); i++) {
        curr_step = (curr_step + dir + 4) % 4;
        set_coils(curr_step);
        delay(delay_ms);
    }
}

void release_motor() {
    for (int i = 0; i < num_pins; i++) {
        digitalWrite(start + i, LOW);
    }
}

void setup() {
    for (int i = 0; i < num_pins; i++) {
        pinMode(start + i, OUTPUT);
    }
}

void loop() {
    step_motor(steps_per_rev, 3);
    release_motor();
    delay(1000);

    step_motor(-steps_per_rev, 3);
    release_motor();
    delay(1000);

}