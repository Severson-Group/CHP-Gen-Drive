#ifdef APP_CVCR

#include <math.h>
#include <string.h>
#include "sys/scheduler.h"
#include "sys/task_stats.h"
#include "drv/pwm.h"
#include "drv/motherboard.h"
#include "drv/analog.h"
#include "drv/encoder.h"
#include "usr/cvcr/task_cvcr.h"
#include "usr/power_stack.h"
#include "usr/measurement_io.h"
#include "usr/machine.h"
#include "CVCR_Controller.h"

// Speed Filter Length
// TODO: Replace all this filter w/ a LPF in simulink and feed in single, raw encoder cnt
#define avgLen (350)

// Scheduler TCB which holds task "context"
static task_control_block_t tcb;

// User vars
uint32_t encoderPos = 0;
double Iqc = 0.0;
double Idc = 0.0;
double Vdc = 0.0;
double mechPos = 0.0;
double elecPos = 0.0;
double pulseToRad = PI/(double)(ENCODER_PULSES_PER_REV);	// Encoder has 2 z pulse per rev, fixed at 4096 pulses per rev
double prevMechPos = 0.0;
double filtWm = 0.0;
double filtWe = 0.0;
double instSpeed = 0.0;
static double speedSamples[avgLen];;

// Flags & Counters
int i = 0;
int encoderInit = 0;
int error = SUCCESS;

// Log Vars
double LOG_Iq, LOG_Id;
double LOG_Iqc, LOG_Idc;
double LOG_Iu, LOG_Iv, LOG_Iw;
double LOG_Vuc, LOG_Vvc, LOG_Vwc;
double LOG_Vdc;
double LOG_enc, LOG_theta, LOG_wm;
// Error Vars
int LOG_error;


int task_cvcr_init(void)
{
    if (scheduler_tcb_is_registered(&tcb)) {
        return FAILURE;
    }

    // Fill TCB with parameters
    scheduler_tcb_init(&tcb, task_cvcr_callback,
                        NULL, "CVCR", TASK_CVCR_INTERVAL_USEC);

    // Enable Task Profiling
    task_stats_enable(&tcb.stats);

    // Set HW PWM frequency, deadtime, and initialize duty ratios
    pwm_set_deadtime_ns((double)POWERSTACK_DEADTIME);
    pwm_set_switching_freq((double)POWERSTACK_PWM_FREQ);
    pwm_set_all_duty_midscale();
    pwm_enable_hw(true);
    //pwm_enable();	// This gets called in the command init handler

    // Get rotor position
	encoder_get_position(&encoderPos);
	LOG_enc = encoderPos;
	mechPos = encoderPos*pulseToRad; 	// Mechanical angle [rad]
	elecPos = mechPos*POLEPAIRS; 	// Electrical angle [erad]

	// Align Q-axis
	while(elecPos > 2*PI)
		elecPos -= 2*PI;
	if(elecPos < ENCODER_TO_D_AXIS_OFFSET)
		elecPos += 2*PI;
	elecPos -= ENCODER_TO_D_AXIS_OFFSET;

	//Update Theta
	CVCR_Controller_U.Theta = elecPos;

	// Clear speed filter
	for(int j = 0; j<avgLen; j++)
		speedSamples[j] = 0.0;

	// Clear all inputs
	CVCR_Controller_U.Id_c = 0.0;
	CVCR_Controller_U.Iq_c = 0.0;
	CVCR_Controller_U.Vdcc = 0.0;

	// Clear Command Inputs
	Iqc = 0.0;
	Idc = 0.0;

    // Initialize CVCR Controller
    CVCR_Controller_initialize();

    // Register task with scheduler
    return scheduler_tcb_register(&tcb);
}

int task_cvcr_deinit(void)
{
//	pwm_disable();	// Gets calld in command deinit handler
	pwm_enable_hw(false);

	// set all commands to zero
	CVCR_Controller_U.Id_c = 0.0;
	CVCR_Controller_U.Iq_c = 0.0;
	CVCR_Controller_U.Vdcc = 0.0;

	// Deinit CVCR controller
	CVCR_Controller_terminate();

	// Clear CVCR control state variables
	memset(((void *) &CVCR_Controller_DW), 0, sizeof(DW_CVCR_Controller_T));

	// TODO: Single use to catch over limit failure
	// Update to expand to other error types
	if(error == FAILURE){
		// Tag error case
		LOG_error = 66;
	}

    return scheduler_tcb_unregister(&tcb);
}

void task_cvcr_callback(void *arg)
{
	double Iu, Iv, Iw;
	double Vuc, Vvc, Vwc;
	double Vdc;
	int err = FAILURE;
	int32_t out = 0;
	uint32_t MB_BASE_ADDR = MOTHERBOARD_2_BASE_ADDR;

	// Not used, set to zero for now
	CVCR_Controller_U.Iload = 0.0;
	CVCR_Controller_U.Ivsi = 0.0;
	CVCR_Controller_U.Vdcc = 0.0;
	CVCR_Controller_U.Wm = 0.0;

	// Update command
	CVCR_Controller_U.Id_c = Idc;
	CVCR_Controller_U.Iq_c = Iqc;

	// Get Currents
	err = motherboard_get_data(MB_BASE_ADDR, ADC_I_U, &out);
	Iu = (double) PHASE_U_CURRENT_GAIN*(out - PHASE_U_CURRENT_OFFSET_COUNT);

	err = motherboard_get_data(MB_BASE_ADDR, ADC_I_V, &out);
	Iv = (double) PHASE_V_CURRENT_GAIN*(out - PHASE_V_CURRENT_OFFSET_COUNT);

	err = motherboard_get_data(MB_BASE_ADDR, ADC_I_W, &out);
	Iw = (double) PHASE_W_CURRENT_GAIN*(out - PHASE_W_CURRENT_OFFSET_COUNT);

	CVCR_Controller_U.Iu = Iu;
	CVCR_Controller_U.Iv = Iv;
	CVCR_Controller_U.Iw = Iw;

	// DC Link Voltage
	err = motherboard_get_data(MB_BASE_ADDR, ADC_V_DC, &out);
	Vdc = (double) Vdc_VOLTAGE_GAIN* out;

	// Update Input
	CVCR_Controller_U.Vdc = Vdc;

	// Limit Check
	if(Iu > OP_LIMIT_PHASE_CURRENT || Iv > OP_LIMIT_PHASE_CURRENT || Iw > OP_LIMIT_PHASE_CURRENT || Vdc > OP_LIMIT_VDC){
		error = FAILURE;
		task_cvcr_deinit();
		return;
	}

	// Get rotor position
	encoder_get_position(&encoderPos);
	mechPos = encoderPos*pulseToRad; 	// Mechanical angle [rad]
	elecPos = mechPos*POLEPAIRS; 	// Electrical angle [erad]

	// Align Q-axis
	while(elecPos > 2*PI)
		elecPos -= 2*PI;
	if(elecPos < ENCODER_TO_D_AXIS_OFFSET)
		elecPos += 2*PI;
	elecPos -= ENCODER_TO_D_AXIS_OFFSET;

	// Update input
	CVCR_Controller_U.Theta = elecPos;

	// Speed filtering
	if(( (mechPos-prevMechPos) > PI) || ( (mechPos - prevMechPos) < -PI) )
	{
		if( filtWm < 0)
		{
			// Forward and sudden large negative sample
			instSpeed = (double)TASK_CVCR_UPDATES_PER_SEC*(2*PI + mechPos - prevMechPos);
			//instSpeed = (double)(2*PI + mechPos - prevMechPos);
		}
		else
		{
			// Reverse and sudden large positive sample
			instSpeed = -(double)TASK_CVCR_UPDATES_PER_SEC*(2*PI - (mechPos - prevMechPos));
			//instSpeed = -(double)(2*PI - (mechPos - prevMechPos));
		}
	}
	else
	{
		instSpeed = (double)TASK_CVCR_UPDATES_PER_SEC*(mechPos - prevMechPos);
		//instSpeed = (double)(mechPos - prevMechPos);
	}
	prevMechPos = mechPos;

	// Correct shaft rotation w/ phase rotation
	instSpeed = -instSpeed;

	speedSamples[i] = instSpeed;
	i++;
	if (i>=avgLen)
		i = 0;

	filtWm = 0.0;
	for (int j = 0; j < avgLen; j++)
		filtWm += speedSamples[j];
	filtWm = (double)(filtWm/avgLen);

	filtWe = filtWm*POLEPAIRS;

	// Update We input
	CVCR_Controller_U.We = filtWe;

	// Vdc regulator not used, set inputs to zero
	CVCR_Controller_U.Vdcc = 0.0;
	CVCR_Controller_U.Iload = 0.0;
	CVCR_Controller_U.Ivsi = 0.0;

	// Do the thing!
	CVCR_Controller_step();

	// Phase Duty Ratio
	Vuc =  CVCR_Controller_Y.H11;
	Vvc =  CVCR_Controller_Y.H21;
	Vwc =  CVCR_Controller_Y.H31;

	// Update PWM
	pwm_set_duty(PWM_U, Vuc);
	pwm_set_duty(PWM_V, Vvc);
	pwm_set_duty(PWM_W, Vwc);

	// Update Log Vars

	LOG_Iq = 0.0;
	LOG_Id = 0.0;
	LOG_Iqc = Iqc;
	LOG_Idc = Idc;
	LOG_Iu = Iu;
	LOG_Iv = Iv;
	LOG_Iw = Iw;
	LOG_Vuc = Vuc;
	LOG_Vvc = Vvc;
	LOG_Vwc = Vwc;
	LOG_Vdc = Vdc;
	LOG_theta = elecPos;
	LOG_wm = filtWm;
	LOG_enc = encoderPos;
}

int task_cvcr_enc(void)
{
	// Init Encoder
	encoder_init();
	encoderInit = 1;

	//curr_channel = channel;
    return SUCCESS;
}


int task_cvcr_set_frequency(double freq)
{
    //omega = freq;
    return SUCCESS;
}

int task_cvcr_set_amplitude(double amplitude)
{

	int err = FAILURE;
    /*
    if (amplitude >= 0.0 && amplitude <= 1.0) {
        Do = amplitude;
        err = SUCCESS;
    }
*/
    return SUCCESS;
}

int task_cvcr_set_iqc(double new_iqc)
{
	Iqc = new_iqc;
	return SUCCESS;
}

int task_cvcr_set_idc(double new_idc)
{
	Idc = new_idc;
	return SUCCESS;
}

void task_cvcr_stats_print(void)
{
    task_stats_print(&tcb.stats);
}

void task_cvcr_stats_reset(void)
{
    task_stats_reset(&tcb.stats);
}


#endif // APP_CVCR
