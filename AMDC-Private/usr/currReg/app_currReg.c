#ifdef APP_CURRREG

#include "usr/currReg/app_currReg.h"
#include "usr/currReg/task_currReg.h"
#include "usr/currReg/cmd/cmd_currReg.h"


void app_currReg_init(void)
{
    // Register "currReg" command with system
    cmd_currReg_register();

    // Initialize currReg task with system
    //task_currReg_init();

}

#endif // APP_CURRREG
