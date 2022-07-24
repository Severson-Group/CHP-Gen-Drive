#ifndef CMD_CVCR_H
#define CMD_CVCR_H

void cmd_cvcr_register(void);
int cmd_cvcr(int argc, char **argv);
void task_cvcr_stats_print(void);
void task_cvcr_stats_reset(void);

#endif // CMD_CVCR_H
