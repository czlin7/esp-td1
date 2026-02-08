#include "C12832.h"
#include "coursework2/C12832/C12832.h"
#include "mbed.h"
#include <PwmOut.h>

#include "potentiometer.h"
#include "vec2f.h" // vector coordinates

// ============================================
// FSM Enums
// ============================================

// FSM states
enum ProgramState {
  SInit,
  SForward,
  SBackward,
  SStandby,
  SDebug,
  SNone
}; // FSM ProgramState

enum UI { UInit, UMotor };

struct HardwareEvents {};


// UI Controller class
class UIController {
    C12832 lcd()
private:
  void renderInit();
  void renderMotorSelect(bool motor) // 0 for left motor, 1 for right motor
  {
        lcd.   
  }
  void renderVoltageAdjust();
  void renderConfirm();

  void renderRealTime();
  void renderFault();

}; // UI Controller class

// PWM Signals
PwmOut pwmRedLED(D5);
PwmOut pwmGreenLED(D9);
PwmOut pwmBlueLED(D8);

// RGB LEDs
// LED redLED(D5);
//  LED greenLED(D9);
// LED blueLED(D8);

// ============================================
// main
// ============================================
int main() {

  SamplingPotentiometer potL(A0, 3.3f, 200.0f);
  SamplingPotentiometer potR(A1, 3.3f, 200.0f);

  // 1. SETUP: Configure the hardware ONCE before the loop
  pwmRedLED.period(0.02f); // 20ms period (50Hz)
  pwmRedLED.write(potR.Potentiometer::getCurrentSampleNorm()); // 50% duty cycle

  // 2. LOOP: Do nothing (or do other tasks) while PWM runs in background
  while (1) {
  }
}
