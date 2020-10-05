/**
 * @file
 * @brief public declarations for port clock functions.
 *
 * @author @htmlonly &copy; @endhtmlonly 2020 James Bennion-Pedley
 *
 * @date 7 June 2020
 */

#ifndef __PORT_CORE__
#define __PORT_CORE__

/* Configuration includes */
#include <globalConfig.h>
#include <portConfig.h>

/*----------------------------- PUBLIC FUNCTIONS -----------------------------*/

/**
 * @brief Initialise the main system clock tree
*/
void portClockInit(void);

/**
 * @brief Reset entire system
 */
void portSystemReset(void);

/*----------------------------------------------------------------------------*/

#endif /* __PORT_CORE__ */
