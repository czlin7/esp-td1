#include "C12832.h"
#include "mbed.h"

//#include "mbed2/299/TARGET_NUCLEO_F401RE/TARGET_STM/TARGET_STM32F4/TARGET_NUCLEO_F401RE/PinNames.h"
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
  float duty_cycle;     // 0..100
  float encoder_speed_ms; // m/s

  MotorSignals()
      : motor_enable(false), motor_unipolar(true), motor_dir(true),
        duty_cycle(0.5), encoder_speed_ms(0.0f) {}
};

// ============================================
// UIController
// ============================================
class UIController {
private:
  UI m_ui;

  C12832 *m_lcd;
  MotorSignals *m_motor_sig;
  SamplingPotentiometer *m_potL;
  SamplingPotentiometer *m_potR;
  InterruptIn *m_button;
  DigitalOut *m_ledR;
  DigitalOut *m_ledG;
  DigitalOut *m_ledB;
  volatile bool m_toggleRequested; //Set by ISR below in Public
  UI_Focus m_prevFocus;

  //void drawSelector(uint8_t x, uint_t y, uint8_t w, uint8_t h){ //Helper function for selector drawer
  //    m_lcd->rect(x, y, w, h, 1);  
  //}


void setColor(float r, float g, float b){
    *m_ledR = r;
    *m_ledG = g;
    *m_ledB = b;
}

void updateModeLED() {
    if (m_ui.state == UI_NAV)
        setColor(0,0,0); //White if navigating
    else 
        setColor(0,1,1); // RED if editing
}

//void drawFocusIndicator()
//{
//    switch (m_ui.focus) {
//        case UI_FOCUS_STATUS:    m_lcd->rect(0, 0, 70, 11,1); break;
//        case UI_FOCUS_DUTY:      m_lcd->rect(0, 10, 70, 21, 1); break;
//        //case UI_FOCUS_SPEED:     m_lcd->rect(0, 20, 60, 31, 1); break;
 //       case UI_FOCUS_MODE:      m_lcd->rect(70, 0, 58, 11, 1); break;
 //       case UI_FOCUS_DIRECTION: m_lcd->rect(70, 10, 58, 21, 1); break;
 //   }
//}

void updateSelector()
{
    if (m_ui.focus == m_prevFocus) return;

    // erase old rectangle
    switch (m_prevFocus) {
        case UI_FOCUS_STATUS:    m_lcd->fillrect(0, 0, 69, 12, 0); break;
        case UI_FOCUS_DUTY:      m_lcd->fillrect(0, 20, 69, 31, 0); break;
        //case UI_FOCUS_SPEED:     m_lcd->fillrect(0, 20, 60, 10, 0); break;
        case UI_FOCUS_MODE:      m_lcd->fillrect(70, 0, 126, 12, 0); break;
        case UI_FOCUS_DIRECTION: m_lcd->fillrect(70, 20, 126, 31, 0); break;
    }

    // draw new rectangle
    switch (m_ui.focus) {
        case UI_FOCUS_STATUS:    m_lcd->rect(0, 0, 69, 12, 1); break;
        case UI_FOCUS_DUTY:      m_lcd->rect(0, 20, 69, 31, 1); break;
        //case UI_FOCUS_SPEED:     m_lcd->rect(0, 20, 60, 10, 1); break;
        case UI_FOCUS_MODE:      m_lcd->rect(70, 0, 126, 12, 1); break;
        case UI_FOCUS_DIRECTION: m_lcd->rect(70, 20, 126, 31, 1); break;
    }

    m_prevFocus = m_ui.focus;
}



public:
  UIController(C12832 *lcd, MotorSignals *motor_sig, SamplingPotentiometer *potL, SamplingPotentiometer *potR, InterruptIn *button, DigitalOut *r, DigitalOut *g, DigitalOut *b)
      : m_lcd(lcd), m_motor_sig(motor_sig), m_potL(potL), m_potR(potR), m_button(button), m_ledR(r), m_ledG(g), m_ledB(b), m_toggleRequested(false), m_prevFocus(UI_FOCUS_STATUS) {
    // default UI states
    m_ui.state = UI_NAV;
    m_ui.focus = UI_FOCUS_STATUS;
    m_button->rise(callback(this, &UIController::onButtonISR));
    updateModeLED();
  }

  void onButtonISR()
  {m_toggleRequested = true;}

  void processButton()
  {
      if (m_toggleRequested) {
          m_toggleRequested = false;
          m_ui.state = (m_ui.state == UI_NAV) ? UI_EDIT : UI_NAV;
      }
      updateModeLED();
  }

  void renderDisplay() {
    updateSelector();

    m_lcd->locate(1, 1);
    m_lcd->printf("Status:%s",
                  m_motor_sig->motor_enable ? "Enabled" : "Disabled");

    m_lcd->locate(1, 21);
    m_lcd->printf("Duty Cycle:%0.1f", m_motor_sig->duty_cycle);

    //m_lcd->locate(35, 12);
    //m_lcd->printf("TD1 GROUP 37");
    //m_lcd->printf("Speed: %.2f", m_motor_sig->encoder_speed_ms);

    // Right Column
    m_lcd->locate(71, 1);
    m_lcd->printf("Mode:%s", m_motor_sig->motor_unipolar ? "Uni" : "Bi");

    m_lcd->locate(71, 21);
    m_lcd->printf("Direction:%s", m_motor_sig->motor_dir ? "Fw" : "Bw");

    //m_lcd->locate(60, 20);
    //m_lcd->printf("TD1 GROUP 37");
  };

  void renderInit();
  void renderVoltageAdjust();
  void renderConfirm();

  void renderRealTime();
  void renderFault();

  void handleNavigation() {
      float navValue = m_potL->getCurrentSampleNorm(); // Nav Potentiometer
      float editValue = m_potR->getCurrentSampleNorm(); //Editing Potentiometer

      //Navigation
      if (m_ui.state == UI_NAV) {
          if (navValue < 0.2f)      m_ui.focus = UI_FOCUS_STATUS;
          else if (navValue < 0.4f) m_ui.focus = UI_FOCUS_DUTY;
          else if (navValue < 0.6f) m_ui.focus = UI_FOCUS_SPEED;
          else if (navValue < 0.8f) m_ui.focus = UI_FOCUS_MODE;
          else                      m_ui.focus = UI_FOCUS_DIRECTION;
      }
      //Editing
      if (m_ui.state == UI_EDIT) {
          if (m_ui.focus == UI_FOCUS_DUTY) {
              m_motor_sig->duty_cycle = editValue;
          }
          if (m_ui.focus == UI_FOCUS_DIRECTION) {
              m_motor_sig->motor_dir = (editValue >= 0.5);
          }
          if (m_ui.focus == UI_FOCUS_MODE) {
              m_motor_sig->motor_unipolar = (editValue >= 0.5);
          }
          if (m_ui.focus == UI_FOCUS_STATUS) {
              m_motor_sig->motor_enable = (editValue >= 0.5);
          }
  }

}; // UI Controller class
};

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
  MotorSignals motor_sig;                          //Motor Signal Struct Initialize
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
