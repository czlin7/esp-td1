#include "mbed.h"
#include "sensor.h"

class sensorArray{
    private:
        Sensor *sensors[6];
        float x_peak;

    public:
        sensorArray(Sensor *one, Sensor *two, Sensor *three, Sensor *four, Sensor *five, Sensor *six) {
            sensors[0] = one;
            sensors[1] = two;
            sensors[2] = three;
            sensors[3] = four;
            sensors[4] = five;
            sensors[5] = six;
        }

        float calculate_position(){
            float voltages[6];
            int max_index = 0;
            float max_val = 0;

            for (int i = 0; i < 6; i++) {
                voltages[i] = sensors[i]->get_voltage();
                if (voltages[i] > max_val) {
                    max_val = voltages[i];
                    max_index = i;
                }
            }

            float v_center = voltages[max_index];
            float v_left = (max_index > 0) ? voltages[max_index - 1] : 0.0f;
            float v_right = (max_index < 5) ? voltages[max_index + 1] : 0.0f;

            float denominator = 2.0f * (v_left - (2.0f * v_center) + v_right);

            if (denominator != 0.0f) {
                x_peak = (v_left - v_right) / denominator;
            }

            return x_peak;
        }
};