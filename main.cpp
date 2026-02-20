#include "C12832.h"
#include "mbed.h"

#include "potentiometer.h"
#include "encoder.h"
#include "motors.h"
#include "vec2f.h"
#include <cstdint>
#include "ui.h"
#include "buggy.h"

#include "states.h"

// LCD instance
C12832 lcd(D11, D13, D12, D7, D10);

// Button and LED instance
InterruptIn button(D4);
InterruptIn Lbutton(A4);
InterruptIn Rbutton(A5);
DigitalOut R(D5);
DigitalOut G(D9);
DigitalOut B(D8);

// Buggy instance
// Motors
Motor leftMotor(PB_13,PC_2,PH_1); //2
Motor rightMotor(PB_14,PC_3,PH_0); //1
// Encoders
WheelEncoder leftEncoder (PC_10, PC_12, NC, 0.0766, 15, 256);
WheelEncoder rightEncoder (PC_8, PC_6, NC, 0.0766, 15, 256);
// Buggy Object
Buggy buggy(&leftMotor, &rightMotor, &leftEncoder, &rightEncoder);
DigitalOut en(PC_4);

// MAIN CODE
int main() {
// SETUP
  // Hardware confuguration before the loop
  SamplingPotentiometer leftPot(A0, 3.3f, 200.0f);    // potentiometer sampling freq chosen: 200 Hz
  SamplingPotentiometer rightPot(A1, 3.3f, 200.0f);
  MotorData leftMotorData;
  MotorData rightMotorData;                          // Motor Signal Struct Initialize
  UIController ui(&lcd, &leftMotorData, &rightMotorData, &leftPot, &rightPot, &button, &Lbutton, &Rbutton, &R, &G, &B); // UI Object Initialize

  const float ui_period_s = 0.10f; // 10 Hz UI refresh

  // Main while loop
  while (1) {
        // Editing Values of Left Motor
        //en.write(leftMotorData.motor_enable);
        leftMotor.setMode(leftMotorData.motor_bipolar);
        leftMotor.setDirection(leftMotorData.motor_dir);
        leftMotor.setDuty(leftMotorData.duty_cycle);

        // Editing Values of Right Motor
        //en.write(rightMotorData.motor_enable);
        rightMotor.setMode(rightMotorData.motor_bipolar);
        rightMotor.setDirection(rightMotorData.motor_dir);
        rightMotor.setDuty(rightMotorData.duty_cycle);

        // UI rendering
        ui.processMotorSelection();
        ui.handleNavigation();
        ui.processButton();
        ui.renderDisplay();
    wait(ui_period_s); // NOTE: can be altered later to adjust sensor sampling period
  }
}
