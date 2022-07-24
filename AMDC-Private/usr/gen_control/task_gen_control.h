#ifndef TASK_GEN_CONTROLLER_H
#define TASK_GEN_CONTROLLER_H

#include "sys/scheduler.h"

#define TASK_GEN_CONTROLLER_UPDATES_PER_SEC (5000)
#define TASK_GEN_CONTROLLER_INTERVAL_USEC   (USEC_IN_SEC / TASK_GEN_CONTROLLER_UPDATES_PER_SEC)

int task_gen_controller_init(void);
int task_gen_controller_deinit(void);

void task_gen_controller_callback(void *arg);

int task_gen_controller_set_curr_channel(int channel);
int task_gen_controller_set_frequency(double freq);
int task_gen_controller_set_amplitude(double amplitude);

#endif // TASK_GEN_CONTROLLER_H
