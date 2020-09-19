/**
 * @file
 * @brief Port-specific Serial/UART implementation
 * (used for <b>printf()</b> debugging)
 *
 * @author @htmlonly &copy; @endhtmlonly 2020 James Bennion-Pedley
 *
 * @date 7 June 2020
 */

#ifdef DEBUG
#include <stdio.h>
#endif /* DEBUG */

/* Libopencm3 includes */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

/* Port declarations and config */
#include <port.h>

/*----------------------------- PUBLIC FUNCTIONS -----------------------------*/

#ifdef DEBUG
/** 
 * @brief Initialise the Serial interface that is designated for debugging
 * @param baud Sets the baud-rate of the UART
*/
void portSerialInit(int baud) {
    /* Enable GPIOD and USART3 clock. */
    rcc_periph_clock_enable(DEBUG_UART_RCC);
    rcc_periph_clock_enable(RCC_USART3);
    /* Setup GPIO pins for USART3 transmit. */
    gpio_mode_setup(DEBUG_UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
            DEBUG_UART_TX | DEBUG_UART_RX);
    gpio_set_af(DEBUG_UART_PORT, GPIO_AF7, DEBUG_UART_TX | DEBUG_UART_RX);

    /* Setup USART3 parameters. */
    usart_set_baudrate(USART3, baud);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_mode(USART3, USART_MODE_TX);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART3);
}
#endif /* DEBUG */

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
#endif /* DEBUG */
