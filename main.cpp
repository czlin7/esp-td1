#include "C12832.h"
#include "mbed.h"

#include "potentiometer.h"
#include "vec2f.h" // vector coordinates
#include <cstdint>

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
// MotorSignals
// ============================================
// Control Signals towards Motor Driver board
struct MotorSignals {
  bool motor_enable;      // disabled by default
  bool motor_unipolar;    // unipolar by default
  bool motor_dir;         // forward by default
  uint8_t duty_cycle;     // 0..100
  float encoder_speed_ms; // m/s

  MotorSignals()
      : motor_enable(false), motor_unipolar(true), motor_dir(true),
        duty_cycle(50), encoder_speed_ms(0.0f) {}
};

// ============================================
// UIController
// ============================================
class UIController {
private:
  UI m_ui;

  C12832 *m_lcd;
  MotorSignals *m_motor_sig;

public:
  UIController(C12832 *lcd, MotorSignals *motor_sig)
      : m_lcd(lcd), m_motor_sig(motor_sig) {
    // default UI states
    m_ui.state = UI_NAV;
    m_ui.focus = UI_FOCUS_STATUS;
  }

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

// LCD instance
C12832 lcd(D11, D13, D12, D7, D10);

// PWM LED
PwmOut pwmRedLED(D5);

// ============================================
// main
// ============================================
int main() {
  MotorSignals motor_sig;
  UIController ui(&lcd, &motor_sig);

  /// SETUP
  /// Hardware confuguration before the loop
  SamplingPotentiometer potL(
      A0, 3.3f, 200.0f); // potentiometer sampling freq chosen: 200 Hz
  SamplingPotentiometer potR(A1, 3.3f, 200.0f);

  pwmRedLED.period(0.02f); // 20ms period (50Hz)
  // 
  // cycle

  const float ui_period_s = 0.10f; // 10 Hz UI refresh

  // 2. LOOP: Do nothing (or do other tasks) while PWM runs in background
  while (1) {
    // manual control of motor_sig
    motor_sig.duty_cycle =
        (uint8_t)(potR.Potentiometer::getCurrentSampleNorm() * 100.0f);
        pwmRedLED.write(motor_sig.duty_cycle); // TODO: for TD1, later change to the motor instance

        // Render UI
        ui.renderDisplay();

    wait(ui_period_s); // NOTE: can be altered later to adjust sensor sampling
                       // period.
  }
}
