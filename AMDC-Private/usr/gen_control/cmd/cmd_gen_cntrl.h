#ifndef CMD_GEN_CTRL_H
#define CMD_GEN_CTRL_H

void cmd_gen_ctrl_register(void);
int cmd_gen_ctrl(int argc, char **argv);
void task_gen_controller_stats_print(void);
void task_gen_controller_stats_reset(void);

#endif // CMD_GEN_CTRL_H
