#ifndef PRESSURE_PID
#define PRESSURE_PID

#include <stdbool.h>
#include <stdint.h>

extern "C" {

float pressure_input = 0, pressure_output = 0, pressure_setpoint = 0;
struct pid_controller pressure_ctrlData;
struct pid_controller* pressure_pid;

void init_pressure_pid_(){
    // Control loop input,output and setpoint variables
    // Control loop gains
    float kp = 20.0, ki = 10.0, kd = 3.0;

    // Prepare PID controller for operation
    pressure_pid = pid_create(&pressure_ctrlData, &pressure_input, &pressure_output, &pressure_setpoint, kp, ki, kd, 0.1);
    // Allow PID to compute and change output
    pid_auto(pressure_pid);
}

float update_pressure_pid_(float input_) {

    if (pid_need_compute(pressure_pid))
    {
        // ESP_LOGI("PressurePID", "sp: %.2f, ", pressure_setpoint);

        pressure_input = input_;
        pid_compute(pressure_pid);
        return pressure_output;
    }
    return NAN;
}


// end of extern "C"
}
#endif
