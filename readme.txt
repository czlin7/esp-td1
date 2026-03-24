1. added

button.mode(PullDown);
Lbutton.mode(PullDown);
Rbutton.mode(PullDown);

into the setup code in main.cpp

2. in the line

UIController ui(&lcd, ..., &R, &G, &B, &en);, added the &en that wasnt there originally

3. fixed inside the while loop to

float leftSpeed = leftMotorData.duty_cycle * (leftMotorData.motor_dir ? 1.0f : -1.0f);
float rightSpeed = rightMotorData.duty_cycle * (rightMotorData.motor_dir ? 1.0f : -1.0f);

4. enabled the two en.write

5. changed setDuty to move
from:
leftMotor.setDirection(leftMotorData.motor_dir);
leftMotor.setDuty(leftMotorData.duty_cycle);

into:
leftMotor.move(leftSpeed);

6.Restored edit mode logic in ui.h (handle navigation)
