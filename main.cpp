#include "mbed.h"
#include <cstdio>
#include <cstring>

#include "sensor.h"
#include "motors.h"
#include "encoder.h"
#include "buggy.h"
#include "PID.h"

// UART to BLE module (e.g. HM-10): single-character commands from the phone app.
Serial bt(PA_11, PA_12);

SensorArray sensorPCB(PC_0, PB_0, PC_1, PA_4, PA_0, PA_1);

Motor rightMotor(PA_15, PC_2, PA_14);
Motor leftMotor(PB_7, PC_3, PA_13);

WheelEncoder leftEncoder(PC_10, PC_12, NC, 10.0f, 256);
WheelEncoder rightEncoder(PC_8, PC_6, NC, 10.0f, 256);

Buggy buggy(&leftMotor, &rightMotor, &leftEncoder, &rightEncoder, PC_4);

PID linePID(0.35f, 0.0f, 0.04f, -0.4f, 0.4f);
PID leftSpeedPID(2000.0f, 0.0f, 0.1f, -1000.0f, 1000.0f);
PID rightSpeedPID(2000.0f, 0.0f, 0.1f, -1000.0f, 1000.0f);

float baseSpeed = 0.5f;

const float LINE_DT = 0.01f;
const float SPEED_DT = 0.001f;

float targetLeft = 0.0f;
float targetRight = 0.0f;

/** When true, line follower updates targets and speed PIDs drive the wheels. */
static bool line_follow_active = false;

/** Line PID update phase accumulator (file scope so BLE start command can prime it). */
static float lineTimer = 0.0f;

static void seed_line_targets_from_sensors()
{
    const float position = sensorPCB.getPosition();
    const float correction = linePID.compute(position, LINE_DT);
    targetLeft = baseSpeed + correction;
    targetRight = baseSpeed - correction;
}

static void stop_autonomous_no_spin()
{
    line_follow_active = false;
    buggy.stop();
    linePID.reset();
    leftSpeedPID.reset();
    rightSpeedPID.reset();
    targetLeft = 0.0f;
    targetRight = 0.0f;
}

/** BLE text line buffer for `line,kp,ki,kd` / `left,...` / `right,...` (newline-terminated). */
static char ble_line_buf[96];
static size_t ble_line_len = 0;

static void ble_send_ack(const char *msg)
{
    bt.printf("%s\r\n", msg);
}

static void process_ble_text_line(const char *line)
{
    char cmd[16];
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    if (sscanf(line, " %15[^,],%f,%f,%f", cmd, &kp, &ki, &kd) != 4) {
        ble_send_ack("ERR format: line,kp,ki,kd or left/right,kp,ki,kd");
        return;
    }

    if (strcmp(cmd, "line") == 0) {
        linePID.setGains(kp, ki, kd);
        linePID.reset();
        ble_send_ack("OK line");
    } else if (strcmp(cmd, "left") == 0) {
        leftSpeedPID.setGains(kp, ki, kd);
        leftSpeedPID.reset();
        ble_send_ack("OK left");
    } else if (strcmp(cmd, "right") == 0) {
        rightSpeedPID.setGains(kp, ki, kd);
        rightSpeedPID.reset();
        ble_send_ack("OK right");
    } else {
        ble_send_ack("ERR unknown cmd (use line, left, right)");
    }
}

static void poll_ble_serial()
{
    while (bt.readable()) {
        const char c = static_cast<char>(bt.getc());

        if (ble_line_len == 0 && (c == '1' || c == '2' || c == '3')) {
            if (c == '1') {
                line_follow_active = true;
                buggy.setEnable(1);
                linePID.reset();
                leftSpeedPID.reset();
                rightSpeedPID.reset();
                lineTimer = LINE_DT;
                seed_line_targets_from_sensors();
            } else if (c == '2' || c == '3') {
                stop_autonomous_no_spin();
                if (c == '3') {
                    const float kSpin180DistanceScale = 0.88f;
                    const float kSpin180LeftFraction = 0.92f;
                    buggy.rotateAngleTuned(180.0f, 500.0f, kSpin180DistanceScale, kSpin180LeftFraction);
                    buggy.stop();
                }
            }
            continue;
        }

        if (c == '\r' || c == '\n') {
            if (ble_line_len > 0) {
                ble_line_buf[ble_line_len] = '\0';
                process_ble_text_line(ble_line_buf);
                ble_line_len = 0;
            }
            continue;
        }

        if (ble_line_len + 1 < sizeof(ble_line_buf)) {
            ble_line_buf[ble_line_len++] = c;
        } else {
            ble_line_len = 0;
            ble_send_ack("ERR line too long");
        }
    }
}

int main()
{
    bt.baud(9600);

    buggy.setEnable(0);
    line_follow_active = false;
    targetLeft = 0.0f;
    targetRight = 0.0f;

    linePID.reset();
    leftSpeedPID.reset();
    rightSpeedPID.reset();

    lineTimer = 0.0f;

    while (true) {
        poll_ble_serial();

        wait_us(1000);

        if (!line_follow_active) {
            buggy.stop();
            continue;
        }

        const float dt_speed = SPEED_DT;

        const float actualLeft = leftEncoder.getVelocity();
        const float actualRight = rightEncoder.getVelocity();

        const float errorLeft = targetLeft - actualLeft;
        const float errorRight = targetRight - actualRight;

        const float leftCmd = leftSpeedPID.compute(errorLeft, dt_speed);
        const float rightCmd = rightSpeedPID.compute(errorRight, dt_speed);

        buggy.drive((int)leftCmd, (int)rightCmd);

        lineTimer += dt_speed;

        if (lineTimer >= LINE_DT) {
            lineTimer = 0.0f;

            const float position = sensorPCB.getPosition();
            const float correction = linePID.compute(position, LINE_DT);

            targetLeft = baseSpeed + correction;
            targetRight = baseSpeed - correction;
        }
    }
}
