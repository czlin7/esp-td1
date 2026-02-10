#include "C12832.h"
#include "mbed.h"
#include <PwmOut.h>
#include <stdbool.h>

#include "potentiometer.h"
#include "vec2f.h" // vector coordinates

// ============================================
// Enums
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

typedef enum { UI_NAV, UI_EDIT } UI_State;

// UI states
typedef enum {
  UI_FOCUS_STATUS,
  UI_FOCUS_DUTY,
  UI_FOCUS_SPEED,
  UI_FOCUS_MODE,
  UI_FOCUS_DIRECTION
} UI_Focus;

// UI control states
typedef struct {
  UI_State state;
  UI_Focus focus;
} UI;


// ============================================
// struct MotorSignals
// ============================================
// Control Signals towards Motor Driver board
struct MotorSignals {
  bool motor_enable;   // Motors Disabled by default
  bool motor_unipolar; // H-Bridge Mode, Unipolar by default
  bool motor_dir;      // Default direction is Forwards
  uint8_t duty_cycle;  // Any default value for PWM signal

  float encoder_speed_ms; // encoder speed in m/s // No default value

  MotorSignals()
      : motor_enable(false), motor_unipolar(true), motor_dir(true),
        duty_cycle(50) {}
};


// UI Controller
class UIController {
  UI m_ui;

  C12832 *m_lcd;
  MotorSignals *m_motor_sig;

public:
  UIController(C12832 *lcd, MotorSignals *motor_sig)
      : m_lcd(lcd), m_motor_sig(motor_sig) {}

  void renderDisplay() {
    m_lcd->locate(0, 0);
    m_lcd->printf("Status:%s",
                  m_motor_sig->motor_enable ? "Enabled" : "Disabled");

    m_lcd->locate(0, 10);
    m_lcd->printf("Duty Cycle:%d%%", m_motor_sig->duty_cycle);

    m_lcd->locate(0, 20);
    m_lcd->printf("Speed: %.2f", m_motor_sig->encoder_speed_ms);

    // Right Column
    m_lcd->locate(70, 0);
    m_lcd->printf("Mode:%s", m_motor_sig->motor_unipolar ? "Uni" : "Bi");

    m_lcd->locate(70, 10);
    m_lcd->printf("Direction:%s", m_motor_sig->motor_dir ? "Fw" : "Bw");

    m_lcd->locate(60, 20);
    m_lcd->printf("TD1 GROUP 37");
  };

  void renderInit();
  void renderVoltageAdjust();
  void renderConfirm();

  void renderRealTime();
  void renderFault();

}; // UI Controller struct

// LCD
C12832 lcd(D11, D13, D12, D7, D10);

// ============================================
// main
// ============================================
int main() {
  MotorSignals motor_sig;
  UIController ui_controller(&lcd, &motor_sig);

  SamplingPotentiometer potL(A0, 3.3f, 200.0f);
  SamplingPotentiometer potR(A1, 3.3f, 200.0f);

  // 1. SETUP: Configure the hardware ONCE before the loop
  //pwmRedLED.period(0.02f); // 20ms period (50Hz)
  //pwmRedLED.write(potR.Potentiometer::getCurrentSampleNorm()); // 50% duty cycle

  ui_controller.renderDisplay();
  // 2. LOOP: Do nothing (or do other tasks) while PWM runs in background
  while (1) {
  }
}
