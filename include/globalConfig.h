/**
 * @file
 * @brief Global application configuration
 *
 * @author @htmlonly &copy; @endhtmlonly 2020 James Bennion-Pedley
 *
 * @date 7 June 2020
 */

#ifndef __GLOBAL_CONFIG__
#define __GLOBAL_CONFIG__

/*----------------------------- FreeRTOS Config ------------------------------*/

#define FREERTOS_PRIORITIES     10  ///< Number of priorities that FreeRTOS uses

/*------------------------------- lwIP Config --------------------------------*/

//#define MAC_ADDR_MANUAL  ///< If left undefined a PR address will be created

#ifdef MAC_ADDR_MANUAL
    #define MAC_ADDR_0      0x00
    #define MAC_ADDR_1      0x80
    #define MAC_ADDR_2      0xE1
    #define MAC_ADDR_3      0x01
    #define MAC_ADDR_4      0x02
    #define MAC_ADDR_5      0x03
#endif /* MAC_ADDRESS_MANUAL */

#define LWIP_DHCP                   1   ///< Use DHCP and fall back on AutoIP

/* Static IP configuration - only required if LWIP_DHCP==0 */
#if !LWIP_DHCP
    #define LWIP_IP_0   192
    #define LWIP_IP_1   168
    #define LWIP_IP_2   42
    #define LWIP_IP_3   42

    #define LWIP_NM_0   255
    #define LWIP_NM_1   255
    #define LWIP_NM_2   255
    #define LWIP_NM_3   0

    #define LWIP_GW_0   192
    #define LWIP_GW_1   168
    #define LWIP_GW_2   42
    #define LWIP_GW_3   1
#endif /* !LWIP_DHCP */

#define LWIP_HOSTNAME       "lwip"  ///< Hostname of lwIP netif

/** Enable lwIP core locking to allow non thread-safe functions to be called */
#define LWIP_FREERTOS_CHECK_CORE_LOCKING        1

/*----------------------------------------------------------------------------*/

#endif /* __GLOBAL_CONFIG__ */
