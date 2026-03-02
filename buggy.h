#pragma once

#include "motors.h"
#include "encoder.h"


class Buggy {
private:
    Motor *leftMotor;
    Motor *rightMotor;
    WheelEncoder *leftEncoder;
    WheelEncoder *rightEncoder;
    DigitalOut enable;

public:
    Buggy(Motor *L, Motor *R, WheelEncoder *LE, WheelEncoder *RE, PinName en) : leftMotor(L), rightMotor(R), leftEncoder(LE), rightEncoder(RE), enable(en) {}

    void setEnable(bool enable_signal) {
        enable.write(enable_signal);
    }

    bool getEnable() {
        return enable.read();
    }

    void forward_backward(float moving_speed, float time_fb){
        leftMotor->move(moving_speed);
        rightMotor->move(moving_speed);
        wait(time_fb);
        leftMotor->move(0.0f);
        rightMotor->move(0.0f); //stops the buggy after the wait finishes
    }

    void turn_left_right(float turning_speed, float time_lr){ //ensure that speed value is > 0 to turn right, and < 0 to turn left
        leftMotor->move(turning_speed);
        rightMotor->move(-turning_speed);
        wait(time_lr);
        leftMotor->move(0.0f);
        rightMotor->move(0.0f); //stops the buggy after the wait finishes
    }

    void moveDistance(float targetDistance_m, float moving_speed){
        // Reset encoders to measure counts for desired distance
        leftEncoder->reset();
        rightEncoder->reset();

        //Set bipolarity and direction

        // Set the desired speed to move
        enable.write(1);
        leftMotor->move(moving_speed);
        rightMotor->move(moving_speed);

        // // Moving while loop while measuring count/distance using encoder
        while (true)
        {
            float distL = leftEncoder->getDistance();
            float distR = rightEncoder->getDistance();
            float avgDist = (distR + distL) * 0.5 ;
            if (avgDist > targetDistance_m)
            break;

        wait(0.0004f); //2500Hz Distance loop
        }
        // Stop motor after desired distance already travelled
        enable.write(0);
        leftMotor->move(0);
        rightMotor->move(0);
       
    }

    void rotateAngle(float angle_deg, float speed){
        const float PI = 3.14159265359f;
        float angle_rad = angle_deg * PI/180.0f;
        const float wheelBase = 0.16f; // Measure and enter the value

        // Distance targeted for the desired rotation? or turning?
        float targetDistance = (angle_rad * wheelBase) / 2.0f;

        // Reset encoders to measure counts for turning desired degree
        leftEncoder->reset();
        rightEncoder->reset();

        // Determine direction of turning
        float leftSpeed = (angle_deg > 0) ? speed : -speed;
        float rightSpeed = (angle_deg > 0) ? -speed : speed;

        // Actaute motor to turn at desired speed
        enable.write(1);
        leftMotor->move(leftSpeed);
        rightMotor->move(rightSpeed);

        // Turning while loop while measuring count/degree of turning using encoder
        while(true)
        {
            float distL = fabs(leftEncoder->getDistance());
            float distR = fabs(rightEncoder->getDistance());
            float avgDist = (distL + distR) * 0.5f;
            //printf("L %.3f R %.3f\n", distL, distR);
            if (avgDist >= fabs(targetDistance))
            break;

        wait(0.0004f);
        }

        // Stop motor once desired angle has reached
        enable.write(0);
        leftMotor->move(0);
        rightMotor->move(0);
    }
};
