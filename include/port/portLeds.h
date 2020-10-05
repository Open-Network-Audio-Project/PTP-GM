/**
 * @file
 * @brief public declarations for port LED functions
 *
 * @author @htmlonly &copy; @endhtmlonly 2020 James Bennion-Pedley
 *
 * @date 7 June 2020
 */

#ifndef __PORT_LEDS__
#define __PORT_LEDS__

/* Configuration includes */
#include <globalConfig.h>
#include <portConfig.h>

/*----------------------------- PUBLIC FUNCTIONS -----------------------------*/

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

/*----------------------------------------------------------------------------*/

#endif /* __PORT_LEDS__ */
