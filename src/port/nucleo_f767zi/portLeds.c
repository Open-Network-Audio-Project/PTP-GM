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

#include <libopencm3/stm32/adc.h>

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

    /// @todo remove later - used for testing PTP hardware
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO3);

    rcc_periph_clock_enable(RCC_ADC1);
    adc_power_off(ADC1);
    adc_disable_scan_mode(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);

    adc_power_on(ADC1);
}

/// @todo temp!
unsigned int portReadInput(unsigned char pin) {
    uint8_t channel_array[16];
    channel_array[0] = pin;
    adc_set_regular_sequence(ADC1, 1, channel_array);
    adc_start_conversion_regular(ADC1);
    while (!adc_eoc(ADC1));
    uint16_t reg16 = adc_read_regular(ADC1);
    return reg16;
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
