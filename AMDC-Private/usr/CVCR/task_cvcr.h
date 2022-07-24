#ifndef TASK_CVCR_H
#define TASK_CVCR_H

#include "sys/scheduler.h"

#define TASK_CVCR_UPDATES_PER_SEC (10000)
#define TASK_CVCR_INTERVAL_USEC   (USEC_IN_SEC / TASK_CVCR_UPDATES_PER_SEC)

int task_cvcr_init(void);
int task_cvcr_deinit(void);

void task_cvcr_callback(void *arg);

int task_cvcr_enc(void);
int task_cvcr_set_frequency(double freq);
int task_cvcr_set_amplitude(double amplitude);
int task_cvcr_set_iqc(double iqc);
int task_cvcr_set_idc(double idc);

#endif // TASK_CVCR_H
