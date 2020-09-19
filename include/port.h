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


#ifdef DEBUG
/** 
 * @brief Initialise the Serial interface that is designated for debugging
 * @param baud Sets the baud-rate of the UART
*/
void portSerialInit(int baud);
#endif /* DEBUG */

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

/*----------------------------- NEWLIB OVERRIDES -----------------------------*/

#ifdef DEBUG
/** 
 * @brief Overrides the <b>newlib</b> "_write" function that is used by 
 * <b>printf()</b>
 * @param fd file descriptor - handled by <b>newlib</b>
 * @param ptr pointer to char array - handled by <b>newlib</b>
 * @param len length of char array - handled by <b>newlib</b>
 * @retval length of char array, or -1 on failure
*/
int _write(int fd, char *ptr, int len);
#endif /* DEBUG */

/*----------------------------------------------------------------------------*/

#endif /* __PORT__ */
