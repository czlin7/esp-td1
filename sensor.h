#include "mbed.h"

class Sensor{
    private:
        AnalogIn sensorAnalog;
        float referenceVoltage;
    public:
        Sensor(PinName pin, float voltage = 3.3f) : sensorAnalog(pin), referenceVoltage(voltage){}

        float get_voltage(){
            float temp = sensorAnalog.read() * referenceVoltage;
            return temp;
        }
};