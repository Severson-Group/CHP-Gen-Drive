#ifdef APP_CVCR

#include "usr/cvcr/cmd/cmd_cvcr.h"
#include "sys/commands.h"
#include "sys/defines.h"
#include "sys/util.h"
#include "usr/cvcr/task_cvcr.h"
#include "drv/pwm.h"
#include <stdlib.h>
#include <string.h>

// Stores command entry for command system module
static command_entry_t cmd_entry;

// Defines help content displayed for this command
// when user types "help" at command prompt
static command_help_t cmd_help[] = {
    { "init", "Start task" },
    { "deinit", "Stop task" },
    { "freq <freq>", "Set frequency of voltage output (rad/s)" },
    { "amplitude <amp>", "Set amplitude of voltage output (0 to 1)" },
    { "stats print", "Print stats to screen" },
    { "stats reset", "Reset the task timing stats" },
	{ "enc", "Turns encoder on and logs position to LOG_encPos"},
	{ "Iqc", "Set commanded Iq current" },
	{ "Idc", "Set commanded Id current" },
};

void cmd_cvcr_register(void)
{
    commands_cmd_init(&cmd_entry, "cvcr", "CVCR commands",
                        cmd_help, ARRAY_SIZE(cmd_help), cmd_cvcr);
    commands_cmd_register(&cmd_entry);
}

int cmd_cvcr(int argc, char **argv)
{
    if (argc == 2 && STREQ("init", argv[1])) {
        if (task_cvcr_init() != SUCCESS) {
            return CMD_FAILURE;
        }
        if (pwm_enable() != SUCCESS) {
            return CMD_FAILURE;
        }

        return CMD_SUCCESS;
    }

    if (argc == 2 && STREQ("deinit", argv[1])) {
        if (task_cvcr_deinit() != SUCCESS) {
            return CMD_FAILURE;
        }
        if (pwm_disable() != SUCCESS) {
            return CMD_FAILURE;
        }

        return CMD_SUCCESS;
    }

    if (argc == 2 && STREQ("enc", argv[1])) {

        if (task_cvcr_enc() != SUCCESS) {
            return CMD_FAILURE;
        }

        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("Iqc", argv[1])) {

        double new_iqc = strtod(argv[2], NULL);

        if (task_cvcr_set_iqc(new_iqc) != SUCCESS) {
            return CMD_FAILURE;
        }

        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("Idc", argv[1])) {

    	double new_idc = strtod(argv[2], NULL);

        if (task_cvcr_set_idc(new_idc) != SUCCESS) {
            return CMD_FAILURE;
        }

        return CMD_SUCCESS;
    }


    if (argc == 3 && STREQ("freq", argv[1])) {
        double new_freq = strtod(argv[2], NULL);

        if (task_cvcr_set_frequency(new_freq) != SUCCESS) {
            return CMD_FAILURE;
        }

        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("amplitude", argv[1])) {
        double new_amplitude = strtod(argv[2], NULL);

        if (task_cvcr_set_amplitude(new_amplitude) != SUCCESS) {
            return CMD_FAILURE;
        }

        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("stats", argv[1])) {
            if (STREQ("print", argv[2])) {
                task_cvcr_stats_print();
                return CMD_SUCCESS;
            }

            if (STREQ("reset", argv[2])) {
                task_cvcr_stats_reset();
                return CMD_SUCCESS;
            }
        }

    return CMD_INVALID_ARGUMENTS;
}

#endif // APP_CVCR
