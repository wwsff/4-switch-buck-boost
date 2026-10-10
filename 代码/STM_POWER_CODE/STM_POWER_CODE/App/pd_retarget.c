#include "pd_board.h"
#include <rt_sys.h>

/* Standalone firmware must not invoke debugger semihosting services.
 * Diagnostics travel only through framed PowerDesk LOG messages. */
#pragma import(__use_no_semihosting)

void _ttywrch(int ch)
{
    (void)ch;
}

char *_sys_command_string(char *cmd, int len)
{
    if (len > 0)
        cmd[0] = '\0';
    return cmd;
}

void _sys_exit(int returncode)
{
    (void)returncode;
    PD_BoardEmergencyStop();
    __disable_irq();
    for (;;) {}
}
