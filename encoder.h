#pragma once

#include "mbed.h"
#include "QEI.h"
#include <cmath>

class WheelEncoder {
private:
    QEI enc;
    Timer t;
    float functionVal;
    float CPR;
    int previousTick;
    float previousTime;
    float velocity;
    float gear;
    float rpm;
    
public:
    WheelEncoder(PinName A, PinName B, PinName index,float gear_ratio, int cpr) : 
        enc(A, B, index, cpr, QEI::X2_ENCODING), t() {
        CPR = cpr; //effective cpr for x2 encoding mode
        gear = gear_ratio;
        functionVal = (PI * diameter) / (CPR * gear_ratio); //calculates the value for function f(x,y,z), meters per encoder tick

        //reset everything once
        previousTick = 0;
        previousTime = t.read();
        t.start();
        enc.reset();
    }
    const float diameter = 0.079;
    const float PI = 3.14159f;
    float getVelocity(void){
        int currentTick = enc.getPulses(); //update tick before timer activation
        float currentTime = t.read(); //update time elapsed by timer prior to timer activation

        //calculate elaspsed tick and time in comparison to previous execution of getVelocity
        int elapsedTick = currentTick - previousTick;
        float elapsedTime = currentTime - previousTime;

        if(elapsedTime <= 0.0f){
            return 0.0f;
        } //terminate if time did not elapsed to prevent division with 0.

        float tickRate = elapsedTick / elapsedTime; //ticks per second
        velocity = tickRate * functionVal; //meter per second
        rpm = (tickRate / CPR) * 60.0f;

        previousTick = currentTick; //save the final tick value at the termination of this function
        previousTime = currentTime; //save the final time value at the termination of this function

        return velocity;
    }

    int getPulses(void) {
        return enc.getPulses();
    }

    float getRPM(void){
        return rpm;
    }

    float getDistance() {
        float distance = (enc.getPulses() / (CPR*2))*(PI*diameter);
        return (distance);
    }

    void reset() {
        //enc.reset();
        //previousTick = 0;
        //t.reset();
        //previousTime = 0;
        enc.reset();
        t.reset();
        t.start();
        previousTick = 0;
        previousTime = t.read();
    }

};
