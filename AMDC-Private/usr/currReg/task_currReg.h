#ifndef TASK_CURRREG_H
#define TASK_CURRREG_H

#include "sys/scheduler.h"

// Motor Parameters
#define POLEPAIRS (16)

// Current Transformers
#define PHASE_U_CURRENT_GAIN (9.948991219967057)
#define PHASE_V_CURRENT_GAIN (9.976913395045305)
#define PHASE_W_CURRENT_GAIN (10.166327939587392)

#define PHASE_U_CURRENT_OFFSET (-0.0033621852861354298)
#define PHASE_V_CURRENT_OFFSET (-0.0063475261424669124)
#define PHASE_W_CURRENT_OFFSET (0.004509113605904558)

#define PHASE_U_CURRENT_ADC_CHANNEL (ANALOG_IN5)
#define PHASE_V_CURRENT_ADC_CHANNEL (ANALOG_IN6)
#define PHASE_W_CURRENT_ADC_CHANNEL (ANALOG_IN7)

// VSI Setup
#define PHASE_U_VOLTAGE_PWM_CHANNEL (0)
#define PHASE_V_VOLTAGE_PWM_CHANNEL (1)
#define PHASE_W_VOLTAGE_PWM_CHANNEL (2)

// Power Stack
// New CHP stack 2us interlock time, set deadtime to 2100 for known control.
#define SEMIKRON_DEADTIME (2050)

#define SWITCHING_FREQ (20000)
// Frequency that this task is called (in Hz)
//
// Must be less than or equal to scheduler updates per second
// This value is defined in sys/scheduler.h and defaults to 10kHz.
// Note that it can be overridden via usr/user_config.h
#define TASK_CURRREG_UPDATES_PER_SEC (10000)

// Microseconds interval between when task is called
//
// This is what scheduler actually uses to run task,
// but is generated via define above
#define TASK_CURRREG_INTERVAL_USEC (USEC_IN_SEC / TASK_CURRREG_UPDATES_PER_SEC)

// Called in app init function to set up task (or via command)
int task_currReg_init(void); // req'd SJ
int task_currReg_deinit(void);	// recommended SJ

// Callback function which scheduler calls periodically
void task_currReg_callback(void *arg); // req'd SJ

// Print the statistics gathered by the scheduler
// optional task status stats updates
void task_currReg_stats_print(void);

// Reset the statistics gathered by the scheduler
// optional task status stats
void task_currReg_stats_reset(void);

// Set the Fundamental Frequency of the output
void task_currReg_set_speed(double);

// Set the Iq commanded current
void task_currReg_set_IqCommand(double);

// Set the Id commanded current
void task_currReg_set_IdCommand(double);

// Set bus voltage
void task_currReg_set_busVoltage(double);

#endif // TASK_CURRREG_H

