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
Motor MotorL(PB_13,PC_2,PH_1); //2
Motor MotorR(PB_14,PC_3,PH_0); //1
// Encoders
WheelEncoder leftEncoder (PC_10, PC_12, NC, 0.0766, 15, 256);
WheelEncoder rightEncoder (PC_8, PC_6, NC, 0.0766, 15, 256);
// Buggy Object
Buggy LBuggy(&MotorL, &MotorR, &leftEncoder, &rightEncoder);
DigitalOut En(PC_4);

// MAIN CODE
int main() {
// SETUP
  // Hardware confuguration before the loop
  SamplingPotentiometer potL(A0, 3.3f, 200.0f);    // potentiometer sampling freq chosen: 200 Hz
  SamplingPotentiometer potR(A1, 3.3f, 200.0f);
  MotorData motor_sig_L;
  MotorData motor_sig_R;                          // Motor Signal Struct Initialize
  UIController ui(&lcd, &motor_sig_L, &motor_sig_R, &potL, &potR, &button, &Lbutton, &Rbutton, &R, &G, &B); // UI Object Initialize

  const float ui_period_s = 0.10f; // 10 Hz UI refresh

  // Main while loop
  while (1) {
        // Editing Values of Left Motor
        En.write(motor_sig_L.motor_enable);
        MotorL.BiUni_Setter(motor_sig_L.motor_bipolar);
        MotorL.DirectionModifier(motor_sig_L.motor_dir);
        MotorL.PWMModifier(motor_sig_L.duty_cycle);

        // Editing Values of Right Motor
        En.write(motor_sig_R.motor_enable);
        MotorL.BiUni_Setter(motor_sig_R.motor_bipolar);
        MotorL.DirectionModifier(motor_sig_R.motor_dir);
        MotorL.PWMModifier(motor_sig_R.duty_cycle);

        // UI rendering
        ui.processMotorSelection();
        ui.handleNavigation();
        ui.processButton();
        ui.renderDisplay();
    wait(ui_period_s); // NOTE: can be altered later to adjust sensor sampling period
  }
}
