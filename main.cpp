#include "mbed.h"
#include <cstdio>
#include <cstring>
#include <cmath>

#include "sensor.h"
#include "motors.h"
#include "encoder.h"
#include "buggy.h"
#include "PID.h"

Serial bt(PA_9, PA_10);
InterruptIn userButton(USER_BUTTON);

SensorArray  sensorPCB(PC_0, PB_0, PC_1, PA_4, PA_0, PA_1);
Motor        rightMotor(PA_15, PC_2, PA_14);
Motor        leftMotor(PB_7,  PC_3, PA_13);
WheelEncoder leftEncoder(PC_10, PC_12, NC, 10.0f, 256);
WheelEncoder rightEncoder(PC_8,  PC_6, NC, 10.0f, 256);
Buggy        buggy(&leftMotor, &rightMotor, &leftEncoder, &rightEncoder, PC_4);


PID linePID(98.0f, 0.0f, 2.7f, -550.0f, 550.0f);
PID leftSpeedPID(50.0f,  5.0f, 0.0f, -2000.0f, 2000.0f);
PID rightSpeedPID(50.0f, 5.0f, 0.0f, -2000.0f, 2000.0f);


const float LINE_DT  = 0.0025f;   // 2ms 10 ms  — line PID period
const float SPEED_DT = 0.001f;  // 1 ms   — main loop tick


float baseSpeed = 50.0f;
float downhillMaxSpeed = 0.9f;
float downhillBrakeGain = 500.5f;
float turnSlowGain = 0.015f;
float turnMinSpeedScale = 100.5f;
float baseCmdClamp = 350.0f;

#define START_LINE_ON_BOOT  0
#define LINE_LOST_THRESHOLD 19   // 5 × 10 ms = 50 ms of lost line before stop


static bool  line_follow_active = false;
static float filtered_position  = 0.0f;
static float raw_position_last  = 0.0f;
static float targetLeft         = 0.0f;
static float targetRight        = 0.0f;
static float cachedLeft         = 0.0f;
static float cachedRight        = 0.0f;
static int   line_lost_count    = 0;
static float lineTimer          = 0.0f;
static bool  telem_raw_enabled  = false;
static bool  telem_ps_enabled   = false;
static bool  telem_vel_enabled  = false;
static bool  telem_target_enabled = false;
static bool  telem_cmd_enabled = false;
static volatile bool button_start_requested = false;
static Timer buttonDebounceTimer;
static int   lastLeftCmd = 0;
static int   lastRightCmd = 0;
static float filteredLeftVel = 0.0f;
static float filteredRightVel = 0.0f;


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

static void start_autonomous_line_follow()
{
    line_follow_active = true;
    buggy.setEnable(1);
    linePID.reset();
    leftSpeedPID.reset();
    rightSpeedPID.reset();
    line_lost_count = 0;
    lineTimer       = LINE_DT;
    seed_line_targets_from_sensors();
}

static void on_user_button_pressed()
{
    button_start_requested = true;
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

    // Parse 3-float control commands first (line/left/right/base)
    if (sscanf(line, " %15[^,],%f,%f,%f", cmd, &kp, &ki, &kd) == 4) {
        if (strcmp(cmd, "line") == 0) {
            linePID.setGains(kp, ki, kd);
            bt.printf("line=%.4f,%.4f,%.4f\r\n", kp, ki, kd);
            return;
        } else if (strcmp(cmd, "left") == 0) {
            leftSpeedPID.setGains(kp, ki, kd);
            bt.printf("left=%.4f,%.4f,%.4f\r\n", kp, ki, kd);
            return;
        } else if (strcmp(cmd, "right") == 0) {
            rightSpeedPID.setGains(kp, ki, kd);
            bt.printf("right=%.4f,%.4f,%.4f\r\n", kp, ki, kd);
            return;
        } else if (strcmp(cmd, "base") == 0) {
            baseSpeed = kp;   // reuse kp slot for the single float
            bt.printf("base=%.4f\r\n", baseSpeed);
            return;
        }
    }

    // Parse telemetry toggles (raw/ps/vel/target/cmd, optional 0/1)
    if (sscanf(line, " %15[^,],%f", cmd, &kp) == 2) {
        if (strcmp(cmd, "raw") == 0) {
            telem_raw_enabled = (kp != 0.0f);
            ble_send_line(telem_raw_enabled ? "OK raw on" : "OK raw off");
            return;
        } else if (strcmp(cmd, "ps") == 0) {
            telem_ps_enabled = (kp != 0.0f);
            ble_send_line(telem_ps_enabled ? "OK ps on" : "OK ps off");
            return;
        } else if (strcmp(cmd, "vel") == 0) {
            telem_vel_enabled = (kp != 0.0f);
            ble_send_line(telem_vel_enabled ? "OK vel on" : "OK vel off");
            return;
        } else if (strcmp(cmd, "target") == 0) {
            telem_target_enabled = (kp != 0.0f);
            ble_send_line(telem_target_enabled ? "OK target on" : "OK target off");
            return;
        } else if (strcmp(cmd, "cmd") == 0) {
            telem_cmd_enabled = (kp != 0.0f);
            ble_send_line(telem_cmd_enabled ? "OK cmd on" : "OK cmd off");
            return;
        } else if (strcmp(cmd, "vmax") == 0) {
            downhillMaxSpeed = kp;
            bt.printf("vmax=%.4f\r\n", downhillMaxSpeed);
            return;
        } else if (strcmp(cmd, "vbrake") == 0) {
            downhillBrakeGain = kp;
            bt.printf("vbrake=%.4f\r\n", downhillBrakeGain);
            return;
        } else if (strcmp(cmd, "turnk") == 0) {
            turnSlowGain = kp;
            bt.printf("turnk=%.4f\r\n", turnSlowGain);
            return;
        } else if (strcmp(cmd, "turnmin") == 0) {
            if (kp < 0.1f) kp = 0.1f;
            if (kp > 1.0f) kp = 1.0f;
            turnMinSpeedScale = kp;
            bt.printf("turnmin=%.4f\r\n", turnMinSpeedScale);
            return;
        } else if (strcmp(cmd, "cmdmax") == 0) {
            if (kp < 0.0f) kp = 0.0f;
            if (kp > 1000.0f) kp = 1000.0f;
            baseCmdClamp = kp;
            bt.printf("cmdmax=%.1f\r\n", baseCmdClamp);
            return;
        }
    }

    if (sscanf(line, " %15s", cmd) == 1) {
        if (strcmp(cmd, "raw") == 0) {
            telem_raw_enabled = true;
            ble_send_line("OK raw on");
            return;
        } else if (strcmp(cmd, "ps") == 0) {
            telem_ps_enabled = true;
            ble_send_line("OK ps on");
            return;
        } else if (strcmp(cmd, "vel") == 0) {
            telem_vel_enabled = true;
            ble_send_line("OK vel on");
            return;
        } else if (strcmp(cmd, "target") == 0) {
            telem_target_enabled = true;
            ble_send_line("OK target on");
            return;
        } else if (strcmp(cmd, "cmd") == 0) {
            telem_cmd_enabled = true;
            ble_send_line("OK cmd on");
            return;
        }
    }

    ble_send_line("ERR format: line/left/right,kp,ki,kd | base,spd | vmax,spd | vbrake,gain | turnk,gain | turnmin,0.1..1.0 | cmdmax,0..1000 | raw|ps|vel|target|cmd[,0|1]");
}

static void poll_ble_serial()
{
    while (bt.readable()) {
        const char c = static_cast<char>(bt.getc());

        if (ble_line_len == 0 && (c == '1' || c == '2' || c == '3')) {
            if (c == '1') {
                start_autonomous_line_follow();
                ble_send_line("OK start");
            } else if (c == '2') {
                stop_autonomous_no_spin();
                ble_send_line("OK stop");
            } else {
                stop_autonomous_no_spin();
                wait(0.05);
                buggy.rotateAngle(200.0f, 165.0f);
                buggy.stop();
                ble_send_line("OK 180");
                start_autonomous_line_follow();
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
    buttonDebounceTimer.start();
    userButton.fall(&on_user_button_pressed);

    ble_send_line("READY");

    buggy.setEnable(0);
    line_follow_active = false;
    filtered_position  = 0.0f;
    raw_position_last  = 0.0f;
    targetLeft         = 0.0f;
    targetRight        = 0.0f;
    cachedLeft         = 0.0f;
    cachedRight        = 0.0f;
    filteredLeftVel    = 0.0f;
    filteredRightVel   = 0.0f;
    line_lost_count    = 0;

    linePID.reset();
    leftSpeedPID.reset();
    rightSpeedPID.reset();
    lineTimer    = 0.0f;
    ble_line_len = 0;
    telem_raw_enabled = false;
    telem_ps_enabled  = false;
    telem_vel_enabled = false;
    telem_target_enabled = false;
    telem_cmd_enabled = false;

#if START_LINE_ON_BOOT
    start_autonomous_line_follow();
#endif

    while (true) {
        if (button_start_requested) {
            button_start_requested = false;

            // Basic debounce to ignore switch bounce.
            if (buttonDebounceTimer.read_ms() >= 200) {
                buttonDebounceTimer.reset();
                if (!line_follow_active) {
                    start_autonomous_line_follow();
                    ble_send_line("OK start button");
                }
            }
        }

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
        float effectiveTargetLeft  = targetLeft;
        float effectiveTargetRight = targetRight;

        // Turn-priority speed limiting:
        // stronger steering demand -> lower allowed speed for faster corner entry.
        const float turnDemand = 0.5f * std::fabs(effectiveTargetLeft - effectiveTargetRight);
        float turnScale = 1.0f - (turnSlowGain * turnDemand);
        if (turnScale < turnMinSpeedScale) turnScale = turnMinSpeedScale;
        if (turnScale > 1.0f) turnScale = 1.0f;
        const float effectiveVmax = downhillMaxSpeed * turnScale;

        // Always-on max-speed clamp with ratio-preserving scaling.
        // This keeps turning authority on tight curves while respecting effectiveVmax.
        const float maxAbsTarget = (std::fabs(effectiveTargetLeft) > std::fabs(effectiveTargetRight))
            ? std::fabs(effectiveTargetLeft)
            : std::fabs(effectiveTargetRight);
        if (maxAbsTarget > effectiveVmax && maxAbsTarget > 0.0f) {
            const float scale = effectiveVmax / maxAbsTarget;
            effectiveTargetLeft  *= scale;
            effectiveTargetRight *= scale;
        }

        float errorLeft  = effectiveTargetLeft  - filteredLeftVel;
        float errorRight = effectiveTargetRight - filteredRightVel;

        // Hard overspeed braking: if measured speed exceeds vmax, bias error toward braking.
        const float overspeedLeft = (filteredLeftVel >= 0.0f) ?
            (filteredLeftVel - effectiveVmax) :
            ((-filteredLeftVel) - effectiveVmax);
        const float overspeedRight = (filteredRightVel >= 0.0f) ?
            (filteredRightVel - effectiveVmax) :
            ((-filteredRightVel) - effectiveVmax);

        if (overspeedLeft > 0.0f) {
            errorLeft -= (downhillBrakeGain * overspeedLeft) * (filteredLeftVel >= 0.0f ? 1.0f : -1.0f);
        }
        if (overspeedRight > 0.0f) {
            errorRight -= (downhillBrakeGain * overspeedRight) * (filteredRightVel >= 0.0f ? 1.0f : -1.0f);
        }

        const float rawLeftCmd  = leftSpeedPID.compute(errorLeft,  SPEED_DT);
        const float rawRightCmd = rightSpeedPID.compute(errorRight, SPEED_DT);

        // Base+turn decomposition:
        // clamp only base (common mode) so straight-line overshoot is limited
        // without killing turning differential authority.
        float baseCmd = 0.5f * (rawLeftCmd + rawRightCmd);
        float turnCmd = 0.5f * (rawLeftCmd - rawRightCmd);

        if (baseCmd > baseCmdClamp) baseCmd = baseCmdClamp;
        if (baseCmd < -baseCmdClamp) baseCmd = -baseCmdClamp;

        float outLeftCmd = baseCmd + turnCmd;
        float outRightCmd = baseCmd - turnCmd;

        // Final motor safety scaling while preserving left/right ratio.
        const float maxAbsOut = (std::fabs(outLeftCmd) > std::fabs(outRightCmd))
            ? std::fabs(outLeftCmd)
            : std::fabs(outRightCmd);
        if (maxAbsOut > 1000.0f && maxAbsOut > 0.0f) {
            const float s = 1000.0f / maxAbsOut;
            outLeftCmd *= s;
            outRightCmd *= s;
        }

        const int leftCmd  = static_cast<int>(outLeftCmd);
        const int rightCmd = static_cast<int>(outRightCmd);
        lastLeftCmd = leftCmd;
        lastRightCmd = rightCmd;

        buggy.drive(leftCmd, rightCmd);

        lineTimer += SPEED_DT;

        // Line PID block — every 10 ms
        if (lineTimer >= LINE_DT) {
            lineTimer = 0.0f;

            cachedLeft  = leftEncoder.getVelocity();
            cachedRight = rightEncoder.getVelocity();
            const float velFilterAlpha = 0.25f;
            filteredLeftVel  = (velFilterAlpha * cachedLeft) + ((1.0f - velFilterAlpha) * filteredLeftVel);
            filteredRightVel = (velFilterAlpha * cachedRight) + ((1.0f - velFilterAlpha) * filteredRightVel);

            float v[6];
            sensorPCB.readRaw(v);
            float max_val = v[0];
            for (int i = 1; i < 6; i++) {
                if (v[i] > max_val) max_val = v[i];
            }

            if (max_val < 0.17f) {
                line_lost_count++;
                if (line_lost_count >= LINE_LOST_THRESHOLD) {
                    stop_autonomous_no_spin();
                    ble_send_line("STOP line ended");
                }
                // Below threshold: coast on last valid targets
            } else {
                line_lost_count   = 0;
                raw_position_last = sensorPCB.getPosition();
                filtered_position = 0.4f * raw_position_last + 0.6f * filtered_position;

                const float correction = linePID.compute(filtered_position, LINE_DT);
                targetLeft  = baseSpeed + correction;
                targetRight = baseSpeed - correction;
            }

            // Raw ADC-normalized sensor values — every line PID tick (10 ms)
            // if (telem_raw_enabled && !bt.readable()) {
            //     char raw_msg[96];
            //     snprintf(raw_msg, sizeof(raw_msg),
            //              "RAW %.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
            //              v[0], v[1], v[2], v[3], v[4], v[5]);
            //     ble_send_line(raw_msg);
            // }

            // Wheel speed telemetry (encoder velocities) — every 100 ms
            // static int vel_tick = 0;
            // if (telem_vel_enabled && ((++vel_tick % 10) == 0) && !bt.readable()) {
            //     char vel_msg[48];
            //     snprintf(vel_msg, sizeof(vel_msg), "VEL L=%.4f,R=%.4f", filteredLeftVel, filteredRightVel);
            //     ble_send_line(vel_msg);
            // } else if (!telem_vel_enabled) {
            //     vel_tick = 0;
            // }

            // // Target speed telemetry — every 100 ms
            // static int target_tick = 0;
            // if (telem_target_enabled && ((++target_tick % 10) == 0) && !bt.readable()) {
            //     char target_msg[56];
            //     snprintf(target_msg, sizeof(target_msg), "TGT L=%.4f,R=%.4f", targetLeft, targetRight);
            //     ble_send_line(target_msg);
            // } else if (!telem_target_enabled) {
            //     target_tick = 0;
            // }

            // Command telemetry — every 100 ms
            // static int cmd_tick = 0;
            // if (telem_cmd_enabled && ((++cmd_tick % 10) == 0) && !bt.readable()) {
            //     char cmd_msg[40];
            //     snprintf(cmd_msg, sizeof(cmd_msg), "CMD L=%d,R=%d", lastLeftCmd, lastRightCmd);
            //     ble_send_line(cmd_msg);
            // } else if (!telem_cmd_enabled) {
            //     cmd_tick = 0;
            // }

            // Position telemetry — every 250 ms (25 × 10 ms ticks)
            // static int pos_tick = 0;
            // if (telem_ps_enabled && ((++pos_tick % 25) == 0) && !bt.readable()) {
            //     char msg[32];
            //     snprintf(msg, sizeof(msg), "POS %.4f", filtered_position);
            //     ble_send_line(msg);
            // } else if (!telem_ps_enabled) {
            //     pos_tick = 0;
            // }

            poll_ble_serial();
        }
    }
}
