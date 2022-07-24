#ifndef CMD_CURRREG_H
#define CMD_CURRREG_H

// Called in app init function to register command with system
void cmd_currReg_register(void);

// Function called when user types "currReg" command into command prompt
// i.e., this is the controller command handler function
int cmd_currReg(int argc, char **argv);

#endif // CMD_CURRREG_H
