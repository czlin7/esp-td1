#include "mbed.h"
#include "sensor.h"
#include "motors.h"
#include "line_follow.h"

// ---- Sensors ----
Sensor sensors[6] = {
    Sensor(A0), Sensor(A1), Sensor(A2),
    Sensor(A3), Sensor(A4), Sensor(A5)
};

// ---- Motors ----
Motor leftMotor(D5, D4, D3);
Motor rightMotor(D6, D7, D8);

// ---- Line follower ----
LineFollower robot(sensors, leftMotor, rightMotor);

Ticker controlTicker;

void loop() {
    robot.update();
}

int main() {
    leftMotor.setMode(true);
    rightMotor.setMode(true);

    // Run at 500 Hz
    controlTicker.attach(&loop, 0.002f);

    while (true) {
        wait(1000);
    }
}
