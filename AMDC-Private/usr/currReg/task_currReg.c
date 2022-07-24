#ifdef APP_CURRREG

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "sys/scheduler.h"
#include "drv/pwm.h"
#include "drv/analog.h"
#include "drv/encoder.h"
#include "usr/user_config.h"
#include "usr/currReg/task_currReg.h"
#include "usr/measurement_io.h"
#include "AMDC_Control_Law.h"

// User Variables
static double WmCommand = 0.0;
static double IqCommand = 0.0;
static double IdCommand = 0.0;
static double Vdc = 20.0;
uint32_t encPos = 0;
double mechPos = 0.0;
double elecPos = 0.0;
double pulseToRad = 2*PI/(double)ENCODER_PULSES_PER_REV;
//double dAxisOffsetToQAxis = 1.41126108 +PI/2.0;
double dAxisOffsetToQAxis = -PI/6-PI/2; //-PI/6 +PI/2.0;
double prevMechPos = 0.0;
int i = 0;
#define avgLen (350)
double filtWm = 0.0;
static double speedSamples[avgLen];;
double instSpeed = 0.0;

// Logging variables
double LOG_WmC = 0.0;
double LOG_Iq = 0.0;
double LOG_IqCom = 0.0;
//double LOG_Ic = 0.0;
double LOG_instSpeed = 0.0;
double LOG_filtWm = 0.0;
double LOG_theta_mech = 0.0;
// Scheduler TCB which holds task "context"
static task_control_block_t tcb;

int task_currReg_init(void)
{
    if (scheduler_tcb_is_registered(&tcb)) {
        return FAILURE;
    }

    // Fill TCB with parameters
    scheduler_tcb_init(&tcb, task_currReg_callback, NULL, "currReg", TASK_CURRREG_INTERVAL_USEC);

    // Set HW PWM frequency, deadtime, and initialize duty ratios
    pwm_set_deadtime_ns((double)SEMIKRON_DEADTIME);
    pwm_set_switching_freq((double)SWITCHING_FREQ);
    pwm_set_all_duty_midscale();
    pwm_enable_hw(true);
	pwm_enable();

	// Get rotor position
	encoder_get_position(&encPos);
	mechPos = encPos*pulseToRad; 	// Mechanical angle [rad]
	elecPos = mechPos*POLEPAIRS; 	// Electrical angle [erad]
	while(elecPos > 2*PI)
		elecPos -= 2*PI;
	if(elecPos < dAxisOffsetToQAxis)
		elecPos += 2*PI;
	elecPos -= dAxisOffsetToQAxis;

	prevMechPos = mechPos;

    // Set control law Variables
	AMDC_Control_Law_U.WmC = WmCommand;
	AMDC_Control_Law_U.Wm = filtWm;
	AMDC_Control_Law_U.Theta_e = elecPos;

	for(int j = 0; j<avgLen; j++)
		speedSamples[j] = 0.0;

    // User task init
    AMDC_Control_Law_initialize();

    // Register task with scheduler
    return scheduler_tcb_register(&tcb);
}

int task_currReg_deinit(void)
{
	// user defined deinit code
	pwm_enable_hw(false);
	pwm_disable();

	/* Terminate model */
	AMDC_Control_Law_terminate();

	WmCommand = 0.0;
	AMDC_Control_Law_U.WmC = WmCommand;
	for(int j = 0; j<avgLen; j++)
		speedSamples[j] = 0.0;
	prevMechPos = 0.0;
	mechPos = 0.0;
	elecPos = 0.0;
	filtWm = 0.0;

	// Clear AMDC control state variables
	memset(((void *) &AMDC_Control_Law_DW), 0, sizeof(DW_AMDC_Control_Law_T));

    // Unregister task with scheduler
    return scheduler_tcb_unregister(&tcb);
}

void task_currReg_callback(void *arg)
{
	float IuMeas,IvMeas,IwMeas;
	double VuDuty, VvDuty, VwDuty;

	// Get rotor position
	encoder_get_position(&encPos);
	mechPos = encPos*pulseToRad; 	// Mechanical angle [rad]
	elecPos = mechPos*POLEPAIRS; 	// Electrical angle [erad]

	// Align Q-axis
	while(elecPos > 2*PI)
		elecPos -= 2*PI;
	if(elecPos < dAxisOffsetToQAxis)
		elecPos += 2*PI;
	elecPos -= dAxisOffsetToQAxis;

	if(( (mechPos-prevMechPos) > PI) || ( (mechPos - prevMechPos) < -PI) )
	{
		if( filtWm < 0)
		{
			// Forward and sudden large negative sample
			instSpeed = (double)TASK_CURRREG_UPDATES_PER_SEC*(2*PI + mechPos - prevMechPos);
			//instSpeed = (double)(2*PI + mechPos - prevMechPos);
		}
		else
		{
			// Reverse and sudden large positive sample
			instSpeed = -(double)TASK_CURRREG_UPDATES_PER_SEC*(2*PI - (mechPos - prevMechPos));
			//instSpeed = -(double)(2*PI - (mechPos - prevMechPos));
		}
	}
	else
	{
		instSpeed = (double)TASK_CURRREG_UPDATES_PER_SEC*(mechPos - prevMechPos);
		//instSpeed = (double)(mechPos - prevMechPos);
	}
	prevMechPos = mechPos;

	// Correct shaft rotation w/ phase rotation
	instSpeed = -instSpeed;

	// Debug
	//instSpeed = (double)(instSpeed/TASK_CURRREG_UPDATES_PER_SEC);

	speedSamples[i] = instSpeed;
	i++;
	if (i>=avgLen)
		i = 0;

	filtWm = 0.0;
	for (int j = 0; j < avgLen; j++)
		filtWm += speedSamples[j];
	filtWm = (double)(filtWm/avgLen);

	AMDC_Control_Law_U.WmC = WmCommand;
	AMDC_Control_Law_U.Wm = filtWm;
	AMDC_Control_Law_U.Theta_e = elecPos;

/*
 // NO AMDS
	// input currents from LEM HASS 50-S sensors
	analog_getf(PHASE_U_CURRENT_ADC_CHANNEL, &IuMeas);
	analog_getf(PHASE_V_CURRENT_ADC_CHANNEL, &IvMeas);
	analog_getf(PHASE_W_CURRENT_ADC_CHANNEL, &IwMeas);

	AMDC_Control_Law_U.Ia = (double)PHASE_U_CURRENT_GAIN*(IuMeas - PHASE_U_CURRENT_OFFSET);
	AMDC_Control_Law_U.Ib = (double)PHASE_V_CURRENT_GAIN*(IvMeas - PHASE_V_CURRENT_OFFSET);
	AMDC_Control_Law_U.Ic = (double)PHASE_W_CURRENT_GAIN*(IwMeas - PHASE_W_CURRENT_OFFSET);
*/

	// AMDS solution
	// Measurement Pool
	int err = FAILURE;
	int32_t out = 0;

	uint32_t MB_BASE_ADDR = MOTHERBOARD_2_BASE_ADDR;

	err = motherboard_get_data(MB_BASE_ADDR, ADC_I_U, &out);
	IuMeas = (double) PHASE_U_CURRENT_GAIN*(out - PHASE_U_CURRENT_OFFSET_COUNT);

	err = motherboard_get_data(MB_BASE_ADDR, ADC_I_V, &out);
	IvMeas = (double) PHASE_V_CURRENT_GAIN*(out - PHASE_V_CURRENT_OFFSET_COUNT);

	err = motherboard_get_data(MB_BASE_ADDR, ADC_I_W, &out);
	IwMeas = (double) PHASE_W_CURRENT_GAIN*(out - PHASE_W_CURRENT_OFFSET_COUNT);

	err = motherboard_get_data(MB_BASE_ADDR, ADC_V_DC, &out);
	Vdc = (double) Vdc_VOLTAGE_GAIN* out;

	AMDC_Control_Law_U.Ia = IuMeas;
	AMDC_Control_Law_U.Ib = IvMeas;
	AMDC_Control_Law_U.Ic = IwMeas;

	AMDC_Control_Law_step();

	// retrieve & parse y struct
	// scale VxC voltages by VxC/Vdc + 0.5
	// set pwm duty cycle w/ scaled value
	VuDuty =  AMDC_Control_Law_Y.VaC/Vdc + 0.5;
	VvDuty =  AMDC_Control_Law_Y.VbC/Vdc + 0.5;
	VwDuty =  AMDC_Control_Law_Y.VcC/Vdc + 0.5;

	pwm_set_duty(PHASE_U_VOLTAGE_PWM_CHANNEL, VuDuty);	// Set HB1 duty ratio (INV3, PWM13 and PWM14)
	pwm_set_duty(PHASE_V_VOLTAGE_PWM_CHANNEL, VvDuty);    // Set HB2 duty ratio (INV3, PWM15 and PWM16)
	pwm_set_duty(PHASE_W_VOLTAGE_PWM_CHANNEL, VwDuty);    // Set HB3 duty ratio (INV3, PWM17 and PWM18)

	/*
	LOG_IdCom = AMDC_Control_Law_Y.Id_Com;
	LOG_IdAct = AMDC_Control_Law_Y.Id_Act;
	LOG_Theta_e = elecPos;
	*/

	LOG_WmC = WmCommand;
	LOG_instSpeed = instSpeed;
	LOG_filtWm = filtWm;
	LOG_theta_mech = mechPos;
	LOG_Iq = AMDC_Control_Law_Y.Iq_Act;
	LOG_IqCom = AMDC_Control_Law_Y.Iq_Com;
}

void task_currReg_stats_print(void)
{
    task_stats_print(&tcb.stats);
}

void task_currReg_stats_reset(void)
{
    task_stats_reset(&tcb.stats);
}

void task_currReg_set_speed(double reqSpeed)
{
	//Convert from RPM reqSpeed to WmCommand rad/s
	WmCommand = reqSpeed*PI/30;
    AMDC_Control_Law_U.WmC = (double)WmCommand;
}

void task_currReg_set_busVoltage(double volts)
{
    Vdc = volts;
}

void task_currReg_set_IqCommand(double IqCom)
{
    IqCommand = IqCom;
 //   AMDC_Control_Law_U.IqC = IqCommand;
}

void task_currReg_set_IdCommand(double IdCom)
{
    IdCommand = IdCom;
   // AMDC_Control_Law_U.IdC = IdCommand;
}

#endif // APP_CURRREG
