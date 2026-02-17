#include "mbed.h"
#include "enconder.h"
#include <cmath>

class Motor {
private:
    PwmOut pwmPin;
    DigitalOut direction;
    DigitalOut bipolar;
    bool bipolar_status;

public:
    Motor(PinName pwm, PinName dir, PinName bi) : pwmPin(pwm), direction(dir), bipolar(bi) {
        pwmPin.period(0.001f);
        bipolar_status = 0;
        direction = 0;
    }

    void BiUni_Setter(bool bipolarity) { //sets the mode of the motor to either bipolar or unipolar
        bipolar_status = bipolarity;
        if (bipolar_status == 1) {
            bipolar = 1; //set as bipolar
        } else {
            bipolar = 0; //set as unipolar
        }
    }

    bool PolarityCheck(void){
        return bipolar_status;
    }

    void DirectionModifier(bool dirChange){
        direction = dirChange;
    }

    void PWMModifier (float duty_PWM){
        pwmPin.write(duty_PWM);
    }

    void move(float speed) { //custom speed, ranges from -1 to 1, where negative indicate backwards for both unipolar and bipolar
    if (bipolar_status == 1) {
        float duty = (speed * 0.5f) + 0.5f; //converts the value to the vaule between 0~1 (duty cycle)
        pwmPin.write(duty);
    } else {
        pwmPin.write(fabs(speed)); //converts to absolute value to a value between 0~1 (duty cycle)
        if (speed >= 0) {
            direction = 1; // sets direction to forward when speed value > 0
        }
        else {
            direction = 0; // sets direction to backward when speed value < 0
        }
    }
}
};

class Buggy {
private:
    Motor &leftMotor;
    Motor &rightMotor;
    WheelEncoder &leftEnc;
    WheelEncoder &rightEnc;

public:

    Buggy(Motor &L, Motor &R, WheelEncoder &LE, WheelEncoder &RE) : leftMotor(L), rightMotor(R), leftEnc(LE), rightEnc(RE) {}

    void forward_backward(float moving_speed, float distance_fb){

    leftEnc.reset();
    rightEnc.reset(); //reset encoders

    leftMotor.move(moving_speed);
    rightMotor.move(moving_speed); //start motors to designated duty_cycle

    wait(0.2); //wait to reach desired speed

    float v_left = fabs(leftEnc.getVelocity());
    float v_right = fabs(rightEnc.getVelocity());
    float avg_velocity = (v_left + v_right) / 2.0f; //find average velocity of two wheels (both wheels should be about the same)

    if (avg_velocity > 0) { //needed to prevent division by 0
        float time_needed = distance_fb / avg_velocity; //calculate time needed to achieve desired distance from current velocity
        float time_remaining = time_needed - 0.2f; //subtract the initial time take to reach desired speed
        
        if (time_remaining > 0) { //ensures any wait time (time_needed) less than 0.2 is ignored, and just skipped without waiting
            wait(time_remaining);
        }
    }

    leftMotor.move(0);
    rightMotor.move(0); //stop
    }

    void turn_left_right(float turning_speed, float wheel_distance, float degree){ //degree > 0 turns right, degree < 0 turns left
        leftEnc.reset();
        rightEnc.reset();

        float left_speed, right_speed;

        if (degree > 0) { 
            left_speed = fabs(turning_speed);
            right_speed = -fabs(turning_speed); //turns right
        } else {
            left_speed = -fabs(turning_speed);
            right_speed = fabs(turning_speed); //turns left
        }

        leftMotor.move(left_speed);
        rightMotor.move(right_speed);

        wait(0.2); 

        float v_left = fabs(leftEnc.getVelocity());
        float v_right = fabs(rightEnc.getVelocity());
        float avg_velocity = (v_left + v_right) / 2.0f;

        float absolute_degree = fabs(degree);
        float required_distance = (absolute_degree * (M_PI / 180.0f) * track_width) / 2.0f; //converts degree to radian, then multiply it by radius (distance between wheels/2)

        if (avg_velocity > 0) { //prevent division by 0
            float time_needed = required_distance / avg_velocity;
            float time_remaining = time_needed - 0.2f;

            if (time_remaining > 0) {
                wait(time_remaining);
            }
        }

        leftMotor.move(0);
        rightMotor.move(0);
    }
};