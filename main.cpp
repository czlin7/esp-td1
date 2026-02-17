#include "C12832.h"
#include "mbed.h"

#include "potentiometer.h"
#include "encoder.h"
#include "motors.h"
#include "vec2f.h"
#include <cstdint>
#include "ui.h"

#include "states.h"


// LCD instance
C12832 lcd(D11, D13, D12, D7, D10);

// PWM LED
PwmOut pwmMotor(PC_8);
InterruptIn button(D4);
DigitalOut R(D5);
DigitalOut G(D9);
DigitalOut B(D8);


// ============================================
// main
// ============================================
int main() {
/// SETUP
  /// Hardware confuguration before the loop
  SamplingPotentiometer potL(A0, 3.3f, 200.0f);    // potentiometer sampling freq chosen: 200 Hz
  SamplingPotentiometer potR(A1, 3.3f, 200.0f);
  MotorData motor_sig;                          //Motor Signal Struct Initialize
  UIController ui(&lcd, &motor_sig, &potL, &potR, &button, &R, &G, &B); //UI Object Initialize
  pwmMotor.period(0.02f); // 20ms period (50Hz)
  // 
  // cycle

  const float ui_period_s = 0.10f; // 10 Hz UI refresh

  // 2. LOOP: Do nothing (or do other tasks) while PWM runs in background
  while (1) {
    // manual control of motor_sig  ||Added above, test code then can delete these lines
    //motor_sig.duty_cycle =
    //(uint8_t)(potR.Potentiometer::getCurrentSampleNorm() * 100.0f);
        pwmMotor.write(motor_sig.duty_cycle); // TODO: for TD1, later change to the motor instance || IN PROGRESS waiting tests

        // Render UI
        
        ui.handleNavigation();
        ui.processButton();
        ui.renderDisplay();
    wait(ui_period_s); // NOTE: can be altered later to adjust sensor sampling
                       // period.
  }
}
