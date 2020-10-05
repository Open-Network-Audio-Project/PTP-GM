/**
 * @file
 * @brief public declarations for port serial functions
 *
 * @author @htmlonly &copy; @endhtmlonly 2020 James Bennion-Pedley
 *
 * @date 7 June 2020
 */

#ifndef __PORT_SERIAL__
#define __PORT_SERIAL__

/* Configuration includes */
#include <globalConfig.h>
#include <portConfig.h>

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"

/*----------------------------- PUBLIC FUNCTIONS -----------------------------*/

#if DEBUG_LEVEL >= DEBUG_ERRORS
    /**
     * @brief Initialise the Serial interface that is designated for debugging.
    */
    void portSerialInitialise(void);

    /**
     * @brief Sends a single character to the UART in a non-blocking manner.
     * A user-defined ISR is used to unblock the task that calls this function.
     * @param character character to send to UART.
    */
    void portSerialSend(char character);
#endif /* DEBUG_LEVEL >= DEBUG_ERRORS */

#if DEBUG_LEVEL >= DEBUG_FULL
    /**
     * @brief Overrides the <b>newlib</b> "_write" function that is used by
     * <b>printf()</b>.
     * Note that this function is only required for using LWIP's built-in
     * debugging, which is only used at full debugging level.
     * @param fd file descriptor - handled by <b>newlib</b>
     * @param ptr pointer to char array - handled by <b>newlib</b>
     * @param len length of char array - handled by <b>newlib</b>
     * @retval length of char array, or -1 on failure
    */
    int _write(int file, char * ptr, int len);
#endif /* DEBUG_LEVEL >= DEBUG_FULL */

/*----------------------------------------------------------------------------*/

#endif /* __PORT_SERIAL__ */
