/**
 * @file
 * @brief public declarations for port-specific functions
 * 
 * Contains declarations of all the public functions that must be provided by
 * the architecture-specific environment. All functions should start with the
 * word <i>'port'</i> so as to make it clear which parts of the application are
 * architecture-specific (<b>_write</b> is the only exception to this rule)
 *
 * @author @htmlonly &copy; @endhtmlonly 2020 James Bennion-Pedley
 *
 * @date 7 June 2020
 */

#ifndef __PORT__
#define __PORT__

/* Configuration includes */
#include <globalConfig.h>
#include <portConfig.h>

#include "FreeRTOS.h"
#include "task.h"

/*----------------------------- PUBLIC FUNCTIONS -----------------------------*/

/**
 * @brief Initialise the main system clock tree
*/
void portClockInit(void);

/**
 * @brief Initialise the LEDs as outputs
*/
void portLEDInit(void);

/**
 * @brief Toggle the <b>SYSTEM</b> LED
*/
void portSystemLEDToggle(void);

/**
 * @brief Toggle the <b>STATUS</b> LED
*/
void portStatusLEDToggle(void);

/**
 * @brief Toggle the <b>WARNING</b> LED
*/
void portWarningLEDToggle(void);


#if DEBUG_LEVEL >= DEBUG_ERRORS
    /** @brief Task Handle of debug task - required for ISR task notification */
    TaskHandle_t* debug_ptr;

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

/*----------------------------- NETWORK FUNCTIONS ----------------------------*/

// struct ptptime_t {
//   s32_t tv_sec;
//   s32_t tv_nsec;
// };

// void portPTPGetTime(struct ptptime_t * timestamp);

// void portPTPSetTime(struct ptptime_t * timestamp);

// void portPTPUpdateCoarse(struct ptptime_t * timeoffset);

// void portPTPUpdateFine(int32_t Adj);

// YET TO CONFIRM PUBLIC INTERFACE!
void portEthInit(void);

/*----------------------------------------------------------------------------*/

#endif /* __PORT__ */
