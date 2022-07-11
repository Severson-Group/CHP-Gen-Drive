#ifdef APP_GEN_CONTROLLER

#include "usr/gen_control/task_gen_control.h"
#include "sys/scheduler.h"
#include "drv/pwm.h"
#include <math.h>
#include "sys/task_stats.h"
#include "drv/motherboard.h"
#include "drv/analog.h"

// Scheduler TCB which holds task "context"
static task_control_block_t tcb;

double Ts    = 1.0 / (double) TASK_GEN_CONTROLLER_UPDATES_PER_SEC;
double theta = 0.0;    // [rad]
double omega = 377.0;  // [rad/s]
double Do    = 0.8;    // [--]

double LOG_Va = 0.0;
double LOG_Vb = 0.0;
double LOG_Vc = 0.0;

int task_gen_controller_init(void)
{
    if (scheduler_tcb_is_registered(&tcb)) {
        return FAILURE;
    }

    // Fill TCB with parameters
    scheduler_tcb_init(&tcb, task_gen_controller_callback,
                        NULL, "Generator Control", TASK_GEN_CONTROLLER_INTERVAL_USEC);

    // Enable Task Profiling
    task_stats_enable(&tcb.stats);


    // Register task with scheduler
    return scheduler_tcb_register(&tcb);
}

int task_gen_controller_deinit(void)
{
    return scheduler_tcb_unregister(&tcb);
}

void task_gen_controller_callback(void *arg)
{
    // Update theta
    theta += (Ts * omega);

    // Wrap to 2*pi
    theta = fmod(theta, 2.0 * M_PI);

    // Calculate desired duty ratios
    double duty_a = 0.5 + Do/2.0 * cos(theta);
    double duty_b = 0.5 + Do/2.0 * cos(theta - 2.0*M_PI/3.0);
    double duty_c = 0.5 + Do/2.0 * cos(theta - 4.0*M_PI/3.0);

    // Update PWM peripheral in FPGA
    pwm_set_duty(0, duty_a); // Set HB1 duty ratio (INV1, PWM1 and PWM2)
    pwm_set_duty(1, duty_b); // Set HB2 duty ratio (INV1, PWM3 and PWM4)
    pwm_set_duty(2, duty_c); // Set HB3 duty ratio (INV1, PWM5 and PWM6)

    int err = FAILURE;
    int32_t out = 0;

    uint32_t MB_BASE_ADDR = MOTHERBOARD_2_BASE_ADDR;

	err = motherboard_get_data(MB_BASE_ADDR, MB_IN1, &out);
	LOG_Va = 0.00125 * (double) out;

	err = motherboard_get_data(MB_BASE_ADDR, MB_IN2, &out);
    LOG_Vb = 0.00125 * (double) out;

	err = motherboard_get_data(MB_BASE_ADDR, MB_IN3, &out);
    LOG_Vc = 0.00125 * (double) out;

    if(err==FAILURE){
    	LOG_Va = 99.9;
    	LOG_Vb = 99.9;
		LOG_Vc = 99.9;
    }

//    LOG_Va = duty_a;
//    LOG_Vb = duty_b;
//    LOG_Vc = duty_c;
}

int task_gen_controller_set_frequency(double freq)
{
    omega = freq;
    return SUCCESS;
}

int task_gen_controller_set_amplitude(double amplitude)
{
    int err = FAILURE;

    if (amplitude >= 0.0 && amplitude <= 1.0) {
        Do = amplitude;
        err = SUCCESS;
    }

    return err;
}

void task_gen_controller_stats_print(void)
{
    task_stats_print(&tcb.stats);
}

void task_gen_controller_stats_reset(void)
{
    task_stats_reset(&tcb.stats);
}


#endif // APP_GEN_CONTROLLER
