#pragma once

/**
 * @file states.h
 * @brief Enumerations describing firmware runtime and user-interface state.
 */

/**
 * @brief High-level program states used by the firmware state machine.
 */
enum ProgramState {
    Init,     /**< System initialization state. */
    Forward,  /**< Forward motion state. */
    Backward, /**< Reverse motion state. */
    Standby,  /**< Idle state with outputs held. */
    Debug,    /**< Diagnostic or debug state. */
    None      /**< Unspecified state placeholder. */
};

/**
 * @brief User-interface operating modes.
 */
enum UiState {
    Navigation, /**< Focus selection mode. */
    Edit        /**< Field editing mode. */
};

/**
 * @brief Selectable fields in the on-device user interface.
 */
enum UiFocus {
    Status, /**< Motor enable field. */
    Duty,   /**< Duty cycle field. */
    Speed,  /**< Speed field. */
    Mode,   /**< Driver mode field. */
    Dir     /**< Direction field. */
};
