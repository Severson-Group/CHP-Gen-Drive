#ifdef APP_GEN_CONTROLLER

#include "usr/gen_control/app_gen_controller.h"
#include "usr/gen_control/cmd/cmd_gen_cntrl.h"
#include "drv/gp3io_mux.h"

void app_gen_controller_init(void)
{
	// Register App Commands
	cmd_gen_ctrl_register();

    // Set up GPIO mux for the AMDS board
	// GP3IO_MUX_1_BASE_ADDR means top AMDC port
	// GP3IO_MUX_DEVICE1 means which mux I/O in the fpga that the AMDS driver is hooked up to.
	// GP3IO_MUX_DEVICE1 is AMDS IP block
	// GP3IO_MUX_DEVICE2 is Eddy Current Sensor IP block
	gp3io_mux_set_device(GP3IO_MUX_2_BASE_ADDR, GP3IO_MUX_DEVICE1);
}

#endif // APP_GEN_CONTROLLER
