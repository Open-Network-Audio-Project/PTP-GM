/**
 * @file
 * @brief port-specific LED control and initialisation
 *
 * @author @htmlonly &copy; @endhtmlonly 2020 James Bennion-Pedley
 *
 * @date 7 June 2020
 */

/* Port public interface */
#include "port/portLeds.h"

/* Libopencm3 includes */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/*----------------------------- PUBLIC FUNCTIONS -----------------------------*/

/** 
 * @brief Initialise the LEDs as outputs
*/
void portLEDInit(void) {
    /* Enable GPIO clocks */
    rcc_periph_clock_enable(SYSTEM_LED_RCC);
    rcc_periph_clock_enable(STATUS_LED_RCC);
    rcc_periph_clock_enable(WARNING_LED_RCC);
    /* System LED */
    gpio_mode_setup(SYSTEM_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
            SYSTEM_LED_PIN);
    /* Status LED */
    gpio_mode_setup(STATUS_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
            STATUS_LED_PIN);
    /* Warning LED */
    gpio_mode_setup(WARNING_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
            WARNING_LED_PIN);
}

/** 
 * @brief Toggle the <b>SYSTEM</b> LED
*/
void portSystemLEDToggle(void) {
    gpio_toggle(SYSTEM_LED_PORT, SYSTEM_LED_PIN);
}

/** 
 * @brief Toggle the <b>STATUS</b> LED
*/
void portStatusLEDToggle(void) {
    gpio_toggle(STATUS_LED_PORT, STATUS_LED_PIN);
}

/** 
 * @brief Toggle the <b>WARNING</b> LED
*/
void portWarningLEDToggle(void) {
    gpio_toggle(WARNING_LED_PORT, WARNING_LED_PIN);
}
