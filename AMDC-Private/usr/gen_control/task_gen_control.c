#ifdef APP_GEN_CONTROLLER

#include "usr/gen_control/task_gen_control.h"
#include "sys/scheduler.h"
#include "drv/pwm.h"
#include <math.h>
#include "sys/task_stats.h"
#include "drv/motherboard.h"
#include "drv/analog.h"
#include "usr/power_stack.h"
#include "usr/measurement_io.h"

// Scheduler TCB which holds task "context"
static task_control_block_t tcb;

double Ts    = 1.0 / (double) TASK_GEN_CONTROLLER_UPDATES_PER_SEC;
double theta = 0.0;    // [rad]
double omega = 377.0;  // [rad/s]
double Do    = 0.8;    // [--]

int curr_channel = 0; 	// Current cal select channel

double LOG_Iu = 0.0;
double LOG_Iv = 0.0;
double LOG_Iw = 0.0;
double LOG_Vdc = 0.0;
double LOG_Idc = 0.0;

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

    // Set HW PWM frequency, deadtime, and initialize duty ratios
    pwm_set_deadtime_ns((double)POWERSTACK_DEADTIME);
    pwm_set_switching_freq((double)POWERSTACK_PWM_FREQ);
    pwm_set_all_duty_midscale();
    pwm_enable_hw(true);
    //pwm_enable();	// This gets called in the command init handler

    // Register task with scheduler
    return scheduler_tcb_register(&tcb);
}

int task_gen_controller_deinit(void)
{
//	pwm_disable();	// Gets calld in command deinit handler
	pwm_enable_hw(false);

    return scheduler_tcb_unregister(&tcb);
}

void task_gen_controller_callback(void *arg)
{
    // Current Calibration Phase Selection
    switch(curr_channel)
    {
    case 0:
        pwm_set_duty(PWM_U, 1.0);
        pwm_set_duty(PWM_V, 0.0);
        pwm_set_duty(PWM_W, 0.0);
    	break;
    case 1:
        pwm_set_duty(PWM_U, 0.0);
        pwm_set_duty(PWM_V, 1.0);
        pwm_set_duty(PWM_W, 0.0);
    	break;
    case 2:
        pwm_set_duty(PWM_U, 0.0);
        pwm_set_duty(PWM_V, 0.0);
        pwm_set_duty(PWM_W, 1.0);
    	break;
    case 3:
        pwm_set_duty(PWM_U, 0.5);
        pwm_set_duty(PWM_V, 0.5);
        pwm_set_duty(PWM_W, 0.5);
    	break;
    case 4:
        pwm_set_duty(PWM_U, 0.0);
        pwm_set_duty(PWM_V, 0.0);
        pwm_set_duty(PWM_W, 0.0);
		break;

    default:
    	// Update theta
		theta += (Ts * omega);

		// Wrap to 2*pi
		theta = fmod(theta, 2.0 * M_PI);

		// Calculate desired duty ratios
		double duty_a = 0.5 + Do/2.0 * cos(theta);
		double duty_b = 0.5 + Do/2.0 * cos(theta - 2.0*M_PI/3.0);
		double duty_c = 0.5 + Do/2.0 * cos(theta - 4.0*M_PI/3.0);

		// Update PWM peripheral in FPGA
		pwm_set_duty(PWM_U, duty_a); // Set HB1 duty ratio (INV1, PWM1 and PWM2)
		pwm_set_duty(PWM_V, duty_b); // Set HB2 duty ratio (INV1, PWM3 and PWM4)
		pwm_set_duty(PWM_W, duty_c); // Set HB3 duty ratio (INV1, PWM5 and PWM6)
    	break;
    }

    // Measurement Pool
    int err = FAILURE;
    int32_t out = 0;

    uint32_t MB_BASE_ADDR = MOTHERBOARD_2_BASE_ADDR;

	err = motherboard_get_data(MB_BASE_ADDR, ADC_I_U, &out);
	LOG_Iu = (double) PHASE_U_CURRENT_GAIN*(out - PHASE_U_CURRENT_OFFSET_COUNT);

	err = motherboard_get_data(MB_BASE_ADDR, ADC_I_V, &out);
    LOG_Iv = (double) PHASE_V_CURRENT_GAIN*(out - PHASE_V_CURRENT_OFFSET_COUNT);

	err = motherboard_get_data(MB_BASE_ADDR, ADC_I_W, &out);
    LOG_Iw = (double) PHASE_W_CURRENT_GAIN*(out - PHASE_W_CURRENT_OFFSET_COUNT);

	err = motherboard_get_data(MB_BASE_ADDR, ADC_I_DC, &out);
	LOG_Idc = (double) Idc_CURRENT_GAIN*(out - IDC_CURRENT_OFFSET_COUNT);

	err = motherboard_get_data(MB_BASE_ADDR, ADC_V_DC, &out);
	LOG_Vdc = (double) Vdc_VOLTAGE_GAIN* out;

    if(err==FAILURE){
    	LOG_Iu = 99.9;
    	LOG_Iv = 99.9;
		LOG_Iw = 99.9;
		LOG_Vdc = 99.9;
		LOG_Idc = 99.9;
    }
}

int task_gen_controller_set_curr_channel(int channel)
{
	curr_channel = channel;
    return SUCCESS;
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
