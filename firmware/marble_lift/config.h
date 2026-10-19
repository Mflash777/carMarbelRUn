// Marble lift controller - pin map and tuning
// Board: Seeed Studio XIAO ESP32-C3   Driver: DRV8833   Motor: N20 6V ~100 RPM
#pragma once

// ---- Pins (XIAO ESP32-C3 silkscreen names) ----
#define PIN_POT      A0   // D0 / GPIO2  - 10k pot wiper (speed). Leave unconnected to use DEFAULT_SPEED.
#define PIN_BUTTON   D1   // GPIO3       - push button to GND (internal pull-up)
#define PIN_AIN1     D2   // GPIO4       - DRV8833 AIN1
#define PIN_AIN2     D3   // GPIO5       - DRV8833 AIN2
#define PIN_NSLEEP   D4   // GPIO6       - DRV8833 nSLEEP/EEP (HIGH = driver on)

// ---- Behaviour ----
#define USE_POT            1      // 0 = ignore the pot, always DEFAULT_SPEED
#define DEFAULT_SPEED      70     // % duty when USE_POT is 0
#define MIN_SPEED          30     // % - below this an N20 tends to stall under load
#define MAX_SPEED          100    // %
#define RAMP_MS_PER_PCT    8      // soft start / stop: ms per 1% duty step
#define PWM_FREQ_HZ        20000  // above hearing range, no motor whine
#define PWM_BITS           10
#define LONG_PRESS_MS      800    // hold the button this long to run the un-jam routine
#define UNJAM_REVERSE_MS   400    // how long to back the screw off when un-jamming
#define AUTO_OFF_MIN       15     // stop after this many minutes of running (0 = never)
#define START_RUNNING      1      // 1 = motor starts as soon as power is applied
#define MOTOR_REVERSED     0      // set to 1 if the screw turns the wrong way (marbles go down)
