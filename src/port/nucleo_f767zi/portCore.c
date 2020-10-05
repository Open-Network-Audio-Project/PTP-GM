/**
 * @file
 * @brief port-specific clock configuration
 * 
 * The clock configuration is specific to any implementation, but the two most
 * important attributes (HSE_FREQ and SYSCLK_FREQ) are set in the main
 * configuration file. The settings in this file should be decided in accordance
 * with those chosen values.
 *
 * @author @htmlonly &copy; @endhtmlonly 2020 James Bennion-Pedley
 *
 * @date 7 June 2020
 */

/* Port public interface */
#include "port/portCore.h"

/* Libopencm3 includes */
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>

/*---------------------------- PRIVATE FUNCTIONS -----------------------------*/

#if DEBUG_LEVEL >= DEBUG_MINIMAL
/** 
 * @brief map the SYSCLK to a GPIO for inspection with an oscilloscope
*/
static void mco_setup(void)
{
    /* PA8 to AF 0 for MCO */
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO9);
    gpio_set_af(GPIOC, 0, GPIO9);

    /* clock output on pin PC9 (allows checking with scope) */
    RCC_CFGR = (RCC_CFGR & ~(RCC_CFGR_MCO2_MASK << RCC_CFGR_MCO2_SHIFT)) |
            (RCC_CFGR_MCO2_SYSCLK << RCC_CFGR_MCO2_SHIFT);

    /* Set clock prescaler */
    RCC_CFGR = (RCC_CFGR & ~(RCC_CFGR_MCOPRE_MASK << RCC_CFGR_MCO2PRE_SHIFT)) |
            (RCC_CFGR_MCOPRE_DIV_4 << RCC_CFGR_MCO2PRE_SHIFT);
}
#endif /* DEBUG_LEVEL >= DEBUG_MINIMAL */

/*----------------------------- PUBLIC FUNCTIONS -----------------------------*/

/** 
 * @brief Initialise the main system clock tree
*/
void portClockInit(void) {

    struct rcc_clock_scale rcc_config;

    /* Clock Configuration Settings */
    rcc_config.plln = SYSCLK_FREQ/1000000;
    rcc_config.pllp = 0x02; // PLLP divisor of 2
    rcc_config.pllq = 0x04; // PLLQ divisor of 4
    rcc_config.flash_waitstates = FLASH_ACR_LATENCY_3WS;
    rcc_config.hpre = RCC_CFGR_HPRE_DIV_NONE;
    rcc_config.ppre1 = RCC_CFGR_PPRE_DIV_2;
    rcc_config.ppre2 = RCC_CFGR_PPRE_DIV_NONE;
    rcc_config.vos_scale = PWR_SCALE1; // Max power mode
    rcc_config.overdrive = 0; // No overdrive
    rcc_config.ahb_frequency = SYSCLK_FREQ;
    rcc_config.apb1_frequency = SYSCLK_FREQ/2;
    rcc_config.apb2_frequency = SYSCLK_FREQ;

    /* Write Clock Configuration to RCC */
    rcc_clock_setup_hse(&rcc_config, HSE_FREQ/2000000);

    #if DEBUG_LEVEL >= DEBUG_MINIMAL
        /* Map system clock to PC9 (MCO2 output) */
        mco_setup();
    #endif /* DEBUG_LEVEL >= DEBUG_MINIMAL */
}

/**
 * @brief Reset entire system
 */
void portSystemReset(void) {
    reset_handler();
}
