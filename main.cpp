/**
 * @file main.cpp
 * @brief Firmware entry point for manual motor control through the on-device UI.
 */

#include "C12832.h"
#include "mbed.h"

#include <cstdint>

#include "buggy.h"
#include "encoder.h"
#include "motors.h"
#include "potentiometer.h"
#include "states.h"
#include "ui.h"
#include "vec2f.h"

/** LCD controller instance. */
C12832 lcd(D11, D13, D12, D7, D10);

/** UI input buttons and RGB status outputs. */
InterruptIn button(D4);
InterruptIn Lbutton(A4);
InterruptIn Rbutton(A5);
DigitalOut R(D5);
DigitalOut G(D9);
DigitalOut B(D8);

/** Motor and encoder hardware instances. */
Motor leftMotor(PB_13, PC_2, PH_1);
Motor rightMotor(PB_14, PC_3, PH_0);
WheelEncoder leftEncoder(PC_10, PC_12, NC, 0.0766f, 15, 256);
WheelEncoder rightEncoder(PC_8, PC_6, NC, 0.0766f, 15, 256);
Buggy buggy(&leftMotor, &rightMotor, &leftEncoder, &rightEncoder);
DigitalOut en(PC_4);

/**
 * @brief Program entry point.
 * @return Always returns 0.
 */
int main() {
    button.mode(PullDown);
    Lbutton.mode(PullDown);
    Rbutton.mode(PullDown);

    SamplingPotentiometer leftPot(A0, 3.3f, 200.0f);
    SamplingPotentiometer rightPot(A1, 3.3f, 200.0f);

    MotorData leftMotorData;
    MotorData rightMotorData;

    UIController ui(&lcd,
                    &leftMotorData,
                    &rightMotorData,
                    &leftPot,
                    &rightPot,
                    &button,
                    &Lbutton,
                    &Rbutton,
                    &R,
                    &G,
                    &B,
                    &en);

    const float ui_period_s = 0.10f;

    while (1) {
        const float leftSpeed = leftMotorData.duty_cycle * (leftMotorData.motor_dir ? 1.0f : -1.0f);
        const float rightSpeed = rightMotorData.duty_cycle * (rightMotorData.motor_dir ? 1.0f : -1.0f);

        en.write(leftMotorData.motor_enable || rightMotorData.motor_enable);

        leftMotor.setMode(leftMotorData.motor_bipolar);
        leftMotor.move(leftSpeed);

        rightMotor.setMode(rightMotorData.motor_bipolar);
        rightMotor.move(rightSpeed);

        ui.processMotorSelection();
        ui.handleNavigation();
        ui.processButton();
        ui.renderDisplay();

        wait(ui_period_s);
    }
}
