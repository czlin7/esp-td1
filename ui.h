#pragma once

#include "mbed.h"
#include "C12832.h"
#include "motors.h"
#include "potentiometer.h"
#include "states.h"

// UI control states
typedef struct {
    UiState state;
    UiFocus focus;
} UI;

// ============================================
// UIController
// ============================================
class UIController {
private:
    UI m_ui;

    C12832 *m_lcd;
    MotorData *m_motor_sig;
    SamplingPotentiometer *m_potL;
    SamplingPotentiometer *m_potR;
    InterruptIn *m_button;
    DigitalOut *m_ledR;
    DigitalOut *m_ledG;
    DigitalOut *m_ledB;

    volatile bool m_toggleRequested;
    UiFocus m_prevFocus;

    // ------------------------------------------------
    void setColor(float r, float g, float b) {
        *m_ledR = r;
        *m_ledG = g;
        *m_ledB = b;
    }

    void updateModeLED() {
        if (m_ui.state == Navigation)
            setColor(0,0,0);     // Navigation mode
        else
            setColor(0,1,1);     // Edit mode
    }

    void updateSelector() {

        if (m_ui.focus == m_prevFocus) return;

        // erase old rectangle
        switch (m_prevFocus) {
            case Status: m_lcd->fillrect(0, 0, 69, 12, 0); break;
            case Duty:   m_lcd->fillrect(0, 20, 69, 31, 0); break;
            case Mode:   m_lcd->fillrect(70, 0, 126, 12, 0); break;
            case Dir:    m_lcd->fillrect(70, 20, 126, 31, 0); break;
            case Speed:  break;
        }

        // draw new rectangle
        switch (m_ui.focus) {
            case Status: m_lcd->rect(0, 0, 69, 12, 1); break;
            case Duty:   m_lcd->rect(0, 20, 69, 31, 1); break;
            case Mode:   m_lcd->rect(70, 0, 126, 12, 1); break;
            case Dir:    m_lcd->rect(70, 20, 126, 31, 1); break;
            case Speed:  break;
        }

        m_prevFocus = m_ui.focus;
    }

public:

    UIController(C12832 *lcd,
                 MotorData *motor_sig,
                 SamplingPotentiometer *potL,
                 SamplingPotentiometer *potR,
                 InterruptIn *button,
                 DigitalOut *r,
                 DigitalOut *g,
                 DigitalOut *b)
        : m_lcd(lcd),
          m_motor_sig(motor_sig),
          m_potL(potL),
          m_potR(potR),
          m_button(button),
          m_ledR(r),
          m_ledG(g),
          m_ledB(b),
          m_toggleRequested(false),
          m_prevFocus(Status)
    {
        m_ui.state = Navigation;
        m_ui.focus = Status;

        m_button->rise(callback(this, &UIController::onButtonISR));
        updateModeLED();
    }

    // ------------------------------------------------
    void onButtonISR() {
        m_toggleRequested = true;
    }

    void processButton() {
        if (m_toggleRequested) {
            m_toggleRequested = false;
            m_ui.state = (m_ui.state == Navigation) ? Edit : Navigation;
        }
        updateModeLED();
    }

    // ------------------------------------------------
    void renderDisplay() {

        updateSelector();

        m_lcd->locate(1, 1);
        m_lcd->printf("Status:%s",
                      m_motor_sig->motor_enable ? "Enabled" : "Disabled");

        m_lcd->locate(1, 21);
        m_lcd->printf("Duty Cycle:%0.1f",
                      m_motor_sig->duty_cycle);

        m_lcd->locate(71, 1);
        m_lcd->printf("Mode:%s",
                      m_motor_sig->motor_unipolar ? "Uni" : "Bi");

        m_lcd->locate(71, 21);
        m_lcd->printf("Direction:%s",
                      m_motor_sig->motor_dir ? "Fw" : "Bw");
    }

    // ------------------------------------------------
    void handleNavigation() {

        float navValue  = m_potL->getCurrentSampleNorm();
        float editValue = m_potR->getCurrentSampleNorm();

        // Navigation mode
        if (m_ui.state == Navigation) {

            if      (navValue < 0.2f) m_ui.focus = Status;
            else if (navValue < 0.4f) m_ui.focus = Duty;
            else if (navValue < 0.6f) m_ui.focus = Speed;
            else if (navValue < 0.8f) m_ui.focus = Mode;
            else                     m_ui.focus = Dir;
        }

        // Edit mode
        if (m_ui.state == Edit) {

            if (m_ui.focus == Duty)
                m_motor_sig->duty_cycle = editValue;

            if (m_ui.focus == Dir)
                m_motor_sig->motor_dir = (editValue >= 0.5f);

            if (m_ui.focus == Mode)
                m_motor_sig->motor_unipolar = (editValue >= 0.5f);

            if (m_ui.focus == Status)
                m_motor_sig->motor_enable = (editValue >= 0.5f);
        }
    }
};
