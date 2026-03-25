#pragma once

#include "C12832.h"
#include "mbed.h"

#include "motors.h"
#include "potentiometer.h"
#include "states.h"

/**
 * @file ui.h
 * @brief LCD and button driven user-interface controller.
 */

/**
 * @brief Aggregates current UI mode and focused field.
 */
typedef struct {
    UiState state; /**< Active interaction mode. */
    UiFocus focus; /**< Active editable or selectable field. */
} UI;

/**
 * @brief Handles user input, UI state transitions, and LCD rendering.
 */
class UIController {
private:
    UI m_ui;

    C12832 *m_lcd;
    MotorData *m_motor_sig_L;
    MotorData *m_motor_sig_R;
    SamplingPotentiometer *m_potL;
    SamplingPotentiometer *m_potR;
    InterruptIn *m_button;
    InterruptIn *m_lbutton;
    InterruptIn *m_rbutton;
    DigitalOut *m_ledR;
    DigitalOut *m_ledG;
    DigitalOut *m_ledB;
    DigitalOut *m_en;

    volatile bool m_toggleRequested;
    volatile bool m_LeftRequested;
    volatile bool m_RightRequested;
    UiFocus m_prevFocus;

    /**
     * @brief Sets RGB indicator outputs.
     * @param r Red channel value.
     * @param g Green channel value.
     * @param b Blue channel value.
     */
    void setColor(float r, float g, float b) {
        *m_ledR = r;
        *m_ledG = g;
        *m_ledB = b;
    }

    /**
     * @brief Updates status LED color according to UI mode.
     */
    void updateModeLED() {
        if (m_ui.state == Navigation) {
            setColor(0, 0, 0);
        } else {
            setColor(0, 1, 1);
        }
    }

    /**
     * @brief Updates the focus rectangle on the LCD.
     */
    void updateSelector() {
        if (m_ui.focus == m_prevFocus) {
            return;
        }

        switch (m_prevFocus) {
            case Status: m_lcd->fillrect(0, 0, 69, 12, 0); break;
            case Duty: m_lcd->fillrect(0, 20, 69, 31, 0); break;
            case Mode: m_lcd->fillrect(70, 0, 126, 12, 0); break;
            case Dir: m_lcd->fillrect(70, 20, 126, 31, 0); break;
            case Speed: break;
        }

        switch (m_ui.focus) {
            case Status: m_lcd->rect(0, 0, 69, 12, 1); break;
            case Duty: m_lcd->rect(0, 20, 69, 31, 1); break;
            case Mode: m_lcd->rect(70, 0, 126, 12, 1); break;
            case Dir: m_lcd->rect(70, 20, 126, 31, 1); break;
            case Speed: break;
        }

        m_prevFocus = m_ui.focus;
    }

    /**
     * @brief Identifies the motor currently selected in the UI.
     */
    enum MotorSelect {
        MOTOR_LEFT,  /**< Left motor fields are active. */
        MOTOR_RIGHT  /**< Right motor fields are active. */
    };

    MotorSelect m_selectedMotor;
    MotorData *m_motorL;
    MotorData *m_motorR;
    MotorData *m_activeMotor;

public:
    /**
     * @brief Constructs the UI controller and binds input interrupts.
     * @param lcd LCD driver instance.
     * @param motor_sig_L Mutable command data for left motor.
     * @param motor_sig_R Mutable command data for right motor.
     * @param potL Potentiometer used for focus navigation.
     * @param potR Potentiometer used for field editing.
     * @param button Toggle button for switching UI mode.
     * @param Lbutton Left-selection button.
     * @param Rbutton Right-selection button.
     * @param r Red LED output.
     * @param g Green LED output.
     * @param b Blue LED output.
     * @param en Motor enable output line.
     */
    UIController(C12832 *lcd,
                 MotorData *motor_sig_L,
                 MotorData *motor_sig_R,
                 SamplingPotentiometer *potL,
                 SamplingPotentiometer *potR,
                 InterruptIn *button,
                 InterruptIn *Lbutton,
                 InterruptIn *Rbutton,
                 DigitalOut *r,
                 DigitalOut *g,
                 DigitalOut *b,
                 DigitalOut *en)
        : m_lcd(lcd),
          m_motor_sig_L(motor_sig_L),
          m_motor_sig_R(motor_sig_R),
          m_potL(potL),
          m_potR(potR),
          m_button(button),
          m_lbutton(Lbutton),
          m_rbutton(Rbutton),
          m_ledR(r),
          m_ledG(g),
          m_ledB(b),
          m_en(en),
          m_toggleRequested(false),
          m_LeftRequested(false),
          m_RightRequested(false),
          m_prevFocus(Status),
          m_selectedMotor(MOTOR_LEFT),
          m_motorL(motor_sig_L),
          m_motorR(motor_sig_R),
          m_activeMotor(motor_sig_L) {
        m_ui.state = Navigation;
        m_ui.focus = Status;

        m_button->rise(callback(this, &UIController::onButtonISR));
        m_lbutton->rise(callback(this, &UIController::onLeftISR));
        m_rbutton->rise(callback(this, &UIController::onRightISR));
        updateModeLED();
    }

    /**
     * @brief Interrupt handler for UI mode toggle button.
     */
    void onButtonISR() { m_toggleRequested = true; }

    /**
     * @brief Interrupt handler for selecting left motor.
     */
    void onLeftISR() { m_LeftRequested = true; }

    /**
     * @brief Interrupt handler for selecting right motor.
     */
    void onRightISR() { m_RightRequested = true; }

    /**
     * @brief Applies pending mode-toggle request and updates indicator LED.
     */
    void processButton() {
        if (m_toggleRequested) {
            m_toggleRequested = false;
            m_ui.state = (m_ui.state == Navigation) ? Edit : Navigation;
        }
        updateModeLED();
    }

    /**
     * @brief Applies pending motor-selection requests.
     */
    void processMotorSelection() {
        if (m_LeftRequested) {
            m_LeftRequested = false;
            m_selectedMotor = MOTOR_LEFT;
            m_activeMotor = m_motorL;
        }

        if (m_RightRequested) {
            m_RightRequested = false;
            m_selectedMotor = MOTOR_RIGHT;
            m_activeMotor = m_motorR;
        }
    }

    /**
     * @brief Renders motor state and selection frames on the LCD.
     */
    void renderDisplay() {
        updateSelector();

        m_lcd->locate(1, 11);
        m_lcd->printf("%s", m_selectedMotor == MOTOR_LEFT ? "Left" : "Right");

        m_lcd->locate(1, 1);
        m_lcd->printf("Status:%s", m_activeMotor->motor_enable ? "Enabled" : "Disabled");

        m_lcd->locate(1, 21);
        m_lcd->printf("Duty Cycle:%0.1f", m_activeMotor->duty_cycle);

        m_lcd->locate(71, 1);
        m_lcd->printf("Mode:%s", m_activeMotor->motor_bipolar ? "Bi" : "Uni");

        m_lcd->locate(71, 21);
        m_lcd->printf("Direction:%s", m_activeMotor->motor_dir ? "Fw" : "Bw");
    }

    /**
     * @brief Updates focus in navigation mode and edits selected field in edit mode.
     */
    void handleNavigation() {
        const float navValue = m_potL->getCurrentSampleNorm();
        const float editValue = m_potR->getCurrentSampleNorm();

        if (m_ui.state == Navigation) {
            if (navValue < 0.2f) {
                m_ui.focus = Status;
            } else if (navValue < 0.4f) {
                m_ui.focus = Duty;
            } else if (navValue < 0.6f) {
                m_ui.focus = Speed;
            } else if (navValue < 0.8f) {
                m_ui.focus = Mode;
            } else {
                m_ui.focus = Dir;
            }
        }

        if (m_ui.state == Edit) {
            if (m_ui.focus == Duty) {
                m_activeMotor->duty_cycle = editValue;
            }
            if (m_ui.focus == Dir) {
                m_activeMotor->motor_dir = (editValue >= 0.5f);
            }
            if (m_ui.focus == Mode) {
                m_activeMotor->motor_bipolar = (editValue >= 0.5f);
            }
            if (m_ui.focus == Status) {
                m_activeMotor->motor_enable = (editValue >= 0.5f);
            }
        }
    }
};
