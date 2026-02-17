#pragma once

#include "mbed.h"
#include "QEI.h"

static float M_PI = 3.14159265359f;

class WheelEncoder {
    private:
        QEI enc;
        Timer t;
        float functionVal;
        int previousTick;
        float previousTime;
        float velocity;
    public:
        WheelEncoder(PinName A, PinName B, PinName index, float diameter, float gear_ratio, int cpr) : enc(A, B, index, cpr, QEI::X2_ENCODING), t() {
            float finalCPR = cpr * 2.0f; //effective cpr for x2 encoding mode
            functionVal = (M_PI * diameter) / (finalCPR * gear_ratio); //calculates the value for function f(x,y,z)

            //reset everything once
            previousTick = 0;
            previousTime = t.read();
            t.start();
            enc.reset();
        }

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

            previousTick = currentTick; //save the final tick value at the termination of this function
            previousTime = currentTime; //save the final time value at the termination of this function

            return velocity;
        }

        void reset() {
            enc.reset();
            previousTick = 0;
            t.reset();
            previousTime = 0;
    }
};
