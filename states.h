#pragma once

// FSM program states
enum ProgramState {
    Init, 
    Forward,
    Backward,
    Standby,
    Debug,
    None
}; // enum ProgramState

// UI states
enum UiState {
    Navigation,
    Edit
}; // enum UI states

// UI control states
enum UiFocus {
    Status,
    Duty,
    Speed,
    Mode,
    Dir,
};
