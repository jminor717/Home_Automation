/*	Floating point PID control loop for Microcontrollers
        Copyright (C) 2014 Jesus Ruben Santa Anna Zamudio.

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see <http://www.gnu.org/licenses/>.

        Author website: http://www.geekfactory.mx
        Author e-mail: ruben at geekfactory dot mx
 */
#ifndef PID_H
#define PID_H
/*-------------------------------------------------------------*/
/*		Includes and dependencies			*/
/*-------------------------------------------------------------*/
#include "esp_timer.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/*-------------------------------------------------------------*/
/*		Macros and definitions				*/
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
/*		Typedefs enums & structs			*/
/*-------------------------------------------------------------*/

// #ifdef __cplusplus
extern "C" {
// #endif

// all of your legacy C code here



/**
 * Defines if the controler is direct or reverse
 */
enum pid_control_directions {
    E_PID_DIRECT,
    E_PID_REVERSE,
};

uint64_t TICK_SECOND = 1000; // 1000 ms = 1 second

/**
 * Structure that holds PID all the PID controller data, multiple instances are
 * posible using different structures for each controller
 */
struct pid_controller {
    // Input, output and setpoint
    float* input; // Current Process Value
    float* output; // Corrective Output from PID Controller
    float* setpoint; // Controller Setpoint
    // Tuning parameters
    float Kp; // Stores the gain for the Proportional term
    float Ki; // Stores the gain for the Integral term
    float Kd; // Stores the gain for the Derivative term
    // Output minimum and maximum values
    float omin; // Maximum value allowed at the output
    float omax; // Minimum value allowed at the output
    float smallValue; // when error is smaller than this value, the integral term is reset to avoid windup
    // Variables for PID algorithm
    float iterm; // Accumulator for integral term
    float lastin; // Last input value for differential term
    // Time related
    uint64_t lasttime; // Stores the time when the control loop ran last time
    uint64_t sampletime; // Defines the PID sample time
    // Operation mode
    uint8_t automode; // Defines if the PID controller is enabled or disabled
    enum pid_control_directions direction;
};

// typedef struct pid_controller* pid_t;
typedef struct pid_controller pid_controller;

/*-------------------------------------------------------------*/
/*		Function prototypes				*/
/*-------------------------------------------------------------*/
void pid_limits(pid_controller* pid, float min, float max);
void pid_direction(pid_controller* pid, enum pid_control_directions dir);
void pid_tune(pid_controller* pid, float kp, float ki, float kd);


/**
 * @brief Creates a new PID controller
 *
 * Creates a new pid controller and initializes it�s input, output and internal
 * variables. Also we set the tuning parameters
 *
 * @param pid A pointer to a pid_controller structure
 * @param in Pointer to float value for the process input
 * @param out Poiter to put the controller output value
 * @param set Pointer float with the process setpoint value
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Diferential gain
 *
 * @return returns a pid_t controller handle
 */
pid_controller* pid_create(pid_controller* pid, float* in, float* out, float* set, float kp, float ki, float kd, float sm)
{
    pid->input = in;
    pid->output = out;
    pid->setpoint = set;
    pid->automode = false;
    pid->smallValue = sm;

    pid_limits(pid, 0, 255);

    // Set default sample time to 100 ms
    // pid->sampletime = 100 * (TICK_SECOND / 1000);
    pid->sampletime = 100;

    pid_direction(pid, E_PID_DIRECT);
    pid_tune(pid, kp, ki, kd);

    // uint64_t currentTime = esp_timer_get_time();

    pid->lasttime = esp_timer_get_time() - pid->sampletime;

    return pid;
}

/**
 * @brief Check if PID loop needs to run
 *
 * Determines if the PID control algorithm should compute a new output value,
 * if this returs true, the user should read process feedback (sensors) and
 * place the reading in the input variable, then call the pid_compute() function.
 *
 * @return return Return true if PID control algorithm is required to run
 */
bool pid_need_compute(pid_controller* pid)
{
    // Check if the PID period has elapsed
    return (esp_timer_get_time() - pid->lasttime >= pid->sampletime) ? true : false;
}

/**
 * @brief Computes the output of the PID control
 *
 * This function computes the PID output based on the parameters, setpoint and
 * current system input.
 *
 * @param pid The PID controller instance which will be used for computation
 */
bool pid_compute(pid_controller* pid)
{
    // Check if control is enabled
    if (!pid->automode)
        return false;

    float in = *(pid->input);
    float sp = *(pid->setpoint);
    // Compute error
    float error = sp - in;
    // Compute integral
    pid->iterm += (pid->Ki * error);
    if (pid->iterm > pid->omax)
        pid->iterm = pid->omax;
    else if (pid->iterm < pid->omin)
        pid->iterm = pid->omin;
    // Compute differential on input
    float dinput = in - pid->lastin;
    // Compute PID output
    float out = pid->Kp * error + pid->iterm - pid->Kd * dinput;
    // Apply limit to output value
    if (out > pid->omax)
        out = pid->omax;
    else if (out < pid->omin)
        out = pid->omin;
    // Output to pointed variable
    (*pid->output) = out;
    // Keep track of some variables for next execution
    pid->lastin = in;
    pid->lasttime = esp_timer_get_time();
    if ((error < 0 && pid->iterm > 0) || (error > 0 && pid->iterm < 0)) {
        // error > 0 = input < setpoint
        // error < 0 = input > setpoint
        // error < 0 && pid->iterm > 0 = overshoot
        // error > 0 && pid->iterm < 0 = undershoot

        // error < pid->smallValue
        // If error is small, reduce the integral term to avoid windup
        // when we over/undershoot scale the integral term down proportional to the derivitive(slope)
        // helps to reduce oscillation and overshoot caused by integral windup
        pid->iterm = pid->iterm * (1 - (abs(dinput) * 0.1));
    }
    return true;
}

/**
 * @brief Sets new PID tuning parameters
 *
 * Sets the gain for the Proportional (Kp), Integral (Ki) and Derivative (Kd)
 * terms.
 *
 * @param pid The PID controller instance to modify
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void pid_tune(pid_controller* pid, float kp, float ki, float kd)
{
    // Check for validity
    if (kp < 0 || ki < 0 || kd < 0)
        return;

    // Compute sample time in seconds
    float ssec = ((float)pid->sampletime) / ((float)TICK_SECOND);

    pid->Kp = kp;
    pid->Ki = ki * ssec;
    pid->Kd = kd / ssec;

    if (pid->direction == E_PID_REVERSE) {
        pid->Kp = 0 - pid->Kp;
        pid->Ki = 0 - pid->Ki;
        pid->Kd = 0 - pid->Kd;
    }
}

/**
 * @brief Sets the pid algorithm period
 *
 * Changes the between PID control loop computations.
 *
 * @param pid The PID controller instance to modify
 * @param time The time in milliseconds between computations
 */
void pid_sample(pid_controller* pid, uint32_t time)
{
    if (time > 0) {
        float ratio = (float)(time * (TICK_SECOND / 1000)) / (float)pid->sampletime;
        pid->Ki *= ratio;
        pid->Kd /= ratio;
        pid->sampletime = time * (TICK_SECOND / 1000);
    }
}

/**
 * @brief Sets the limits for the PID controller output
 *
 * @param pid The PID controller instance to modify
 * @param min The minimum output value for the PID controller
 * @param max The maximum output value for the PID controller
 */
void pid_limits(pid_controller* pid, float min, float max)
{
    if (min >= max)
        return;
    pid->omin = min;
    pid->omax = max;
    // Adjust output to new limits
    if (pid->automode) {
        if (*(pid->output) > pid->omax)
            *(pid->output) = pid->omax;
        else if (*(pid->output) < pid->omin)
            *(pid->output) = pid->omin;

        if (pid->iterm > pid->omax)
            pid->iterm = pid->omax;
        else if (pid->iterm < pid->omin)
            pid->iterm = pid->omin;
    }
}

/**
 * @brief Enables automatic control using PID
 *
 * Enables the PID control loop. If manual output adjustment is needed you can
 * disable the PID control loop using pid_manual(). This function enables PID
 * automatic control at program start or after calling pid_manual()
 *
 * @param pid The PID controller instance to enable
 */
void pid_auto(pid_controller* pid)
{
    // If going from manual to auto
    if (!pid->automode) {
        pid->iterm = *(pid->output);
        pid->lastin = *(pid->input);
        if (pid->iterm > pid->omax)
            pid->iterm = pid->omax;
        else if (pid->iterm < pid->omin)
            pid->iterm = pid->omin;
        pid->automode = true;
    }
}

/**
 * @brief Disables automatic process control
 *
 * Disables the PID control loop. User can modify the value of the output
 * variable and the controller will not overwrite it.
 *
 * @param pid The PID controller instance to disable
 */
void pid_manual(pid_controller* pid)
{
    pid->automode = false;
}

/**
 * @brief Configures the PID controller direction
 *
 * Sets the direction of the PID controller. The direction is "DIRECT" when a
 * increase of the output will cause a increase on the measured value and
 * "REVERSE" when a increase on the controller output will cause a decrease on
 * the measured value.
 *
 * @param pid The PID controller instance to modify
 * @param direction The new direction of the PID controller
 */
void pid_direction(pid_controller* pid, enum pid_control_directions dir)
{
    if (pid->automode && pid->direction != dir) {
        pid->Kp = (0 - pid->Kp);
        pid->Ki = (0 - pid->Ki);
        pid->Kd = (0 - pid->Kd);
    }
    pid->direction = dir;
}

// #ifdef __cplusplus
}
// #endif

#endif
// End of Header file