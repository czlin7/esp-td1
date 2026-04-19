#include "mbed.h"
#include <cstdio>
#include <cstring>

#include "sensor.h"
#include "motors.h"
#include "encoder.h"
#include "buggy.h"
#include "PID.h"

Serial bt(PA_9, PA_10);

SensorArray  sensorPCB(PC_0, PB_0, PC_1, PA_4, PA_0, PA_1);
Motor        rightMotor(PA_15, PC_2, PA_14);
Motor        leftMotor(PB_7,  PC_3, PA_13);
WheelEncoder leftEncoder(PC_10, PC_12, NC, 10.0f, 256);
WheelEncoder rightEncoder(PC_8,  PC_6, NC, 10.0f, 256);
Buggy        buggy(&leftMotor, &rightMotor, &leftEncoder, &rightEncoder, PC_4);


PID linePID(0.4f, 0.0f, 0.04f, -0.8f, 0.8f);
PID leftSpeedPID(7.0f,  0.0f, 0.0f, -2000.0f, 2000.0f);
PID rightSpeedPID(7.0f, 0.0f, 0.0f, -2000.0f, 2000.0f);


const float LINE_DT  = 0.01f;   // 10 ms  — line PID period
const float SPEED_DT = 0.001f;  // 1 ms   — main loop tick


float baseSpeed = 0.2f;

#define START_LINE_ON_BOOT  0
#define LINE_LOST_THRESHOLD 7   // 5 × 10 ms = 50 ms of lost line before stop


static bool  line_follow_active = false;
static float filtered_position  = 0.0f;
static float raw_position_last  = 0.0f;
static float targetLeft         = 0.0f;
static float targetRight        = 0.0f;
static float cachedLeft         = 0.0f;
static float cachedRight        = 0.0f;
static int   line_lost_count    = 0;
static float lineTimer          = 0.0f;


static char   ble_line_buf[96];
static size_t ble_line_len = 0;


static void ble_send_line(const char *msg)
{
    bt.printf("%s\r\n", msg);
}

static void seed_line_targets_from_sensors()
{
    const float position   = sensorPCB.getPosition();
    const float correction = linePID.compute(position, LINE_DT);
    targetLeft  = baseSpeed + correction;
    targetRight = baseSpeed - correction;
}

static void stop_autonomous_no_spin()
{
    line_follow_active = false;
    buggy.stop();
    linePID.reset();
    leftSpeedPID.reset();
    rightSpeedPID.reset();
    targetLeft      = 0.0f;
    targetRight     = 0.0f;
    cachedLeft      = 0.0f;
    cachedRight     = 0.0f;
    line_lost_count = 0;
}

static void process_ble_text_line(const char *line)
{
    char  cmd[16];
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;

    if (sscanf(line, " %15[^,],%f,%f,%f", cmd, &kp, &ki, &kd) != 4) {
        ble_send_line("ERR format: line,kp,ki,kd  or  left/right,kp,ki,kd");
        return;
    }

    if (strcmp(cmd, "line") == 0) {
        linePID.setGains(kp, ki, kd);
        bt.printf("line=%.4f,%.4f,%.4f\r\n", kp, ki, kd);
    } else if (strcmp(cmd, "left") == 0) {
        leftSpeedPID.setGains(kp, ki, kd);
        bt.printf("left=%.4f,%.4f,%.4f\r\n", kp, ki, kd);
    } else if (strcmp(cmd, "right") == 0) {
        rightSpeedPID.setGains(kp, ki, kd);
        bt.printf("right=%.4f,%.4f,%.4f\r\n", kp, ki, kd);
    } else if (strcmp(cmd, "base") == 0) {
        baseSpeed = kp;   // reuse kp slot for the single float
        bt.printf("base=%.4f\r\n", baseSpeed);
    } else {
        ble_send_line("ERR unknown cmd");
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
                line_lost_count = 0;
                lineTimer       = LINE_DT;
                seed_line_targets_from_sensors();
                ble_send_line("OK start");
            } else if (c == '2') {
                stop_autonomous_no_spin();
                ble_send_line("OK stop");
            } else {
                stop_autonomous_no_spin();
                buggy.rotateAngle(160.0f, 125.0f);
                buggy.stop();
                ble_send_line("OK 180");
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
            ble_send_line("ERR line too long");
        }
    }
}


int main()
{
    bt.baud(9600);
    wait_ms(150);

    ble_send_line("READY");

    buggy.setEnable(0);
    line_follow_active = false;
    filtered_position  = 0.0f;
    raw_position_last  = 0.0f;
    targetLeft         = 0.0f;
    targetRight        = 0.0f;
    cachedLeft         = 0.0f;
    cachedRight        = 0.0f;
    line_lost_count    = 0;

    linePID.reset();
    leftSpeedPID.reset();
    rightSpeedPID.reset();
    lineTimer    = 0.0f;
    ble_line_len = 0;

#if START_LINE_ON_BOOT
    line_follow_active = true;
    buggy.setEnable(1);
    linePID.reset();
    leftSpeedPID.reset();
    rightSpeedPID.reset();
    line_lost_count = 0;
    lineTimer       = LINE_DT;
    seed_line_targets_from_sensors();
#endif

    while (true) {
        poll_ble_serial();
        wait_us(500);
        poll_ble_serial();
        wait_us(500);

        if (!line_follow_active) {
            buggy.stop();
            poll_ble_serial();
            continue;
        }

        // Speed PID — every 1 ms using cached velocities
        const float errorLeft  = targetLeft  - cachedLeft;
        const float errorRight = targetRight - cachedRight;

        const int leftCmd  = static_cast<int>(leftSpeedPID.compute(errorLeft,  SPEED_DT));
        const int rightCmd = static_cast<int>(rightSpeedPID.compute(errorRight, SPEED_DT));

        buggy.drive(leftCmd, rightCmd);

        lineTimer += SPEED_DT;

        // Line PID block — every 10 ms
        if (lineTimer >= LINE_DT) {
            lineTimer = 0.0f;

            cachedLeft  = leftEncoder.getVelocity();
            cachedRight = rightEncoder.getVelocity();

            float v[6];
            sensorPCB.readRaw(v);
            float max_val = v[0];
            for (int i = 1; i < 6; i++) {
                if (v[i] > max_val) max_val = v[i];
            }

            if (max_val < 0.40f) {
                line_lost_count++;
                if (line_lost_count >= LINE_LOST_THRESHOLD) {
                    stop_autonomous_no_spin();
                    ble_send_line("STOP line ended");
                }
                // Below threshold: coast on last valid targets
            } else {
                line_lost_count   = 0;
                raw_position_last = sensorPCB.getPosition();
                filtered_position = 0.3f * raw_position_last + 0.7f * filtered_position;

                const float correction = linePID.compute(filtered_position, LINE_DT);
                targetLeft  = baseSpeed + correction;
                targetRight = baseSpeed - correction;
            }

            // Position telemetry — every 250 ms (25 × 10 ms ticks)
            static int pos_tick = 0;
            if ((++pos_tick % 25) == 0) {
                char msg[32];
                snprintf(msg, sizeof(msg), "POS %.4f", filtered_position);
                ble_send_line(msg);
            }

            poll_ble_serial();
        }
    }
}
