/**
 * @file
 * @brief Port-specific Serial/UART implementation
 * (used for <b>printf()</b> debugging)
 *
 * @author @htmlonly &copy; @endhtmlonly 2020 James Bennion-Pedley
 *
 * @date 7 June 2020
 */


/* Port public interface */
#include "port/portSerial.h"

#if DEBUG_LEVEL >= DEBUG_ERRORS
    #include <stdio.h>
#endif /* DEBUG_LEVEL >= DEBUG_ERRORS */

/* Libopencm3 includes */
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"

/** @brief Task Handle of debug task - required for ISR task notification */
TaskHandle_t* debug_ptr;

/*----------------------------- PUBLIC FUNCTIONS -----------------------------*/

#if DEBUG_LEVEL >= DEBUG_ERRORS
    /**
     * @brief Initialise the Serial interface that is designated for debugging.
    */
    void portSerialInitialise(void)
    {
        /* Enable GPIOD and USART3 clock. */
        rcc_periph_clock_enable(DEBUG_UART_RCC);
        rcc_periph_clock_enable(RCC_USART3);
        /* Setup GPIO pins for USART3 transmit. */
        gpio_mode_setup(DEBUG_UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                DEBUG_UART_TX | DEBUG_UART_RX);
        gpio_set_af(DEBUG_UART_PORT, GPIO_AF7, DEBUG_UART_TX | DEBUG_UART_RX);

        /* Setup USART3 parameters. */
        usart_set_baudrate(USART3, 115200);
        usart_set_databits(USART3, 8);
        usart_set_stopbits(USART3, USART_STOPBITS_1);
        usart_set_mode(USART3, USART_MODE_TX);
        usart_set_parity(USART3, USART_PARITY_NONE);
        usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

        /* Finally enable the USART. */
        usart_enable(USART3);
    }

    /**
     * @brief Sends a single character to the UART in a blocking manner.
     * The UART transmission is blocking to simplify implementation, but this
     * means that the debug task must be the lowest priority.
     * @param character character to send to UART.
    */
    void portSerialSend(char character)
    {
        usart_send_blocking(USART3, (uint16_t)character);
    }
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
    int _write(int file, char * ptr, int len)
    {
        int i;

        if (file == 1) {
            for (i = 0; i < len; i++) {
                if (ptr[i] == '\n')
                    usart_send_blocking(USART3, '\r');
                usart_send_blocking(USART3, ptr[i]);
            }
            return i;
        }
        return -1;
    }
#endif /* DEBUG_LEVEL >= DEBUG_FULL */
