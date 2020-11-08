/**
 * @file
 * @brief port-specific ethernet driver for lwIP
 *
 * @author @htmlonly &copy; @endhtmlonly 2020 James Bennion-Pedley
 *
 * @date 7 June 2020
 */

/* Port public interface */
#include "port/portEthernet.h"

/* lwIP Includes */
#include <lwip/init.h>
#include <lwip/netif.h>
#include <lwip/netifapi.h>
#include <lwip/dhcp.h>
#include <lwip/autoip.h>
#include <lwip/stats.h>
#include <lwip/err.h>
#include <lwip/tcpip.h>

#include <netif/etharp.h>
#include <netif/ethernet.h>

/* Libopencm3 Includes */
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/ethernet/mac.h>
#include <libopencm3/ethernet/phy.h>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/// @todo if performance allows, only use RTOS functions that
/// are exposed through the lwIP sys.h header file.

/* Phy Register Includes */
#ifdef PHY_LAN8742A
    #include <libopencm3/ethernet/phy_lan87xx.h>
#endif /* PHY_LAN8742A */
#ifdef PHY_KS8081
    #include <libopencm3/ethernet/phy_ksz80x1.h>
#endif /* PHY_KS8081 */

#if LWIP_PTP
    #define PTP_UPDATE_COARSE   0
    #define PTP_UPDATE_FINE     1

    #include <ptpd-lwip.h>
#endif /* LWIP_PTP */

/**
 * @brief Network interface struct for ethernet port
*/
static struct netif ethernetif;

/**
 * @brief Generic union for conveniently extracting bytes from words
*/
union word_byte {
    u32_t word;
    u8_t byte[4];
};

/*-------------------- Static Global Variables (DMA) -------------------------------*/

/**
 * @brief Generic DMA Descriptor (see STM32Fxx7 Reference Manual)
*/
struct dma_desc {
    volatile uint32_t   Status;
    uint32_t   ControlBufferSize; // in actuality only first 12 bytes are size
    void *     Buffer1Addr;
    void *     Buffer2NextDescAddr;
    uint32_t   ExtendedStatus;
    uint32_t   Reserved1;
    uint32_t   TimeStampLow;
    uint32_t   TimeStampHigh;
    struct pbuf *pbuf;
}; //__attribute__((packed));

/**
 * @brief Transmit descriptor ring - adjust length from <b>port_config</b>
 * to tailor for your application
*/
#ifndef STIF_NUM_TX_DMA_DESC
    #define STIF_NUM_TX_DMA_DESC 10
#endif /* STIF_NUM_TX_DMA_DESC */
static struct dma_desc tx_dma_desc[STIF_NUM_TX_DMA_DESC];
static struct dma_desc *tx_cur_dma_desc;

/**
 * @brief Recieve descriptor ring - adjust length from <b>port_config</b>
 * to tailor for your application
*/
#ifndef STIF_NUM_RX_DMA_DESC
    #define STIF_NUM_RX_DMA_DESC 10
#endif /* STIF_NUM_RX_DMA_DESC */
static struct dma_desc rx_dma_desc[STIF_NUM_RX_DMA_DESC];
static struct dma_desc *rx_cur_dma_desc;

/**
 * @brief Task handle for triggering packet reception with a FreeRTOS
 * direct task notification
*/
static TaskHandle_t eth_task = NULL;

/*------------------------------ PTP FUNCTIONS -------------------------------*/

/* Temp Config Area */
#define ADJ_FREQ_BASE_INCREMENT   43    // 20ns increment
#define ADJ_FREQ_BASE_ADDEND      2^32*50000000/SYSCLK_FREQ // see AN3411

#if LWIP_PTP

// Check the validity of these macros! REWRITE!!!!!
#define PTP_TO_NSEC(SUBSEC)     (u32_t)((uint64_t)(SUBSEC * 1000000000ll) >> 31)
#define PTP_TO_SUBSEC(NSEC)     (u32_t)((uint64_t)(NSEC * 0x80000000ll) \
                                                            / 1000000000)

static err_t ptp_hw_init(s8_t mode) {
    /* Disable timestamp interrupt */
    ETH_MACIMR |= ETH_MACIMR_TSTIM;

    /* Enable timestamps for relevant packets */
    ETH_PTPTSCR |= ETH_PTPTSCR_TSE | ETH_PTPTSCR_TSSIPV4FE |
                        ETH_PTPTSCR_TSSIPV6FE | ETH_PTPTSCR_TSSARFE;
    /// @todo restrict to IPV4/IPV6 later (OR JUST PTP?)

    /* Set smallest clock adjustment increment (20ns) */
    ETH_PTPSSIR = ETH_PTPSSIR_STSSI & ADJ_FREQ_BASE_INCREMENT;

    if(mode == PTP_UPDATE_FINE) {
        /* Set addend based on SYSCLK frequency (see reference manual) */
        ETH_PTPTSAR = ADJ_FREQ_BASE_ADDEND;
        // printf("ADDEND: %lu\n", ETH_PTPTSAR);

        /* Update addend, wait for bit to be cleared */
        ETH_PTPTSCR |= ETH_PTPTSCR_TTSARU;
        while(ETH_PTPTSCR & ETH_PTPTSCR_TTSARU);

        /* Configure for fine update */
        ETH_PTPTSCR |= ETH_PTPTSCR_TSFCU;
    }
    else {
        /* Configure for coarse update */
        ETH_PTPTSCR &= ~ETH_PTPTSCR_TSFCU;
    }

    /// @todo does timestamp need to be set here?
    ETH_PTPTSHUR = 0;
    ETH_PTPTSLUR = 0;

    /* Initialise timestamping and wait for bit to be cleared */
    ETH_PTPTSCR |= ETH_PTPTSCR_TSSTI;
    while(ETH_PTPTSCR & ETH_PTPTSCR_TSSTI);

    return ERR_OK;
}

static void ptp_get_time(timestamp_t *timestamp) {
    timestamp->secondsField.lsb = ETH_PTPTSHR;
    timestamp->nanosecondsField = PTP_TO_NSEC(ETH_PTPTSLR);
}

static void ptp_set_time(const timestamp_t *timestamp) {
    /* Write timestamps to registers */
    ETH_PTPTSHUR = timestamp->secondsField.lsb;
    ETH_PTPTSLUR |= PTP_TO_SUBSEC(timestamp->nanosecondsField) & ETH_PTPTSLUR_TSUSS;

    /* Reinitialise timestamping and wait for bit to be cleared */
    ETH_PTPTSCR |= ETH_PTPTSCR_TSSTI;
    while(ETH_PTPTSCR & ETH_PTPTSCR_TSSTI);
}

static void ptp_update_coarse(const timestamp_t *timestamp, s8_t sign) {
    /* Backup addend (coarse update clears it) */
    u32_t addend = ETH_PTPTSAR;

    /* Wait for timestamp flags to be cleared */
    while(ETH_PTPTSCR & (ETH_PTPTSCR_TSSTI | ETH_PTPTSCR_TSSTU
                                            | ETH_PTPTSCR_TTSARU));

    if(sign) {
        ETH_PTPTSLUR = ETH_PTPTSLUR_TSUPNS; // Set timestamp as negative
    }
    else {
        ETH_PTPTSLUR = 0;   // Set timestamp as positive (default)
    }

    /* Write timestamps to registers */
    ETH_PTPTSHUR = timestamp->secondsField.lsb;
    ETH_PTPTSLUR |= PTP_TO_SUBSEC(timestamp->nanosecondsField) & ETH_PTPTSLUR_TSUSS;

    /* Update timestamps */
    ETH_PTPTSCR |= ETH_PTPTSCR_TSSTU;
    while(ETH_PTPTSCR & ETH_PTPTSCR_TSSTU);

    /* Restore addend */
    ETH_PTPTSAR = addend;
    ETH_PTPTSCR |= ETH_PTPTSCR_TTSARU;
}

static void ptp_update_fine(s32_t adj) {
    /* Limit maximum frequency adjustment */
    if( adj > 5120000) adj = 5120000;
    if( adj < -5120000) adj = -5120000;

    /* Addend estimation (from AN3411) */
    u32_t addend = ((((275LL * adj)>>8) *
                    (ADJ_FREQ_BASE_ADDEND>>24))>>6) + (ADJ_FREQ_BASE_ADDEND);

    /* Update addend */
    ETH_PTPTSAR = addend;
    while(ETH_PTPTSCR & ETH_PTPTSCR_TTSARU);
    ETH_PTPTSCR |= ETH_PTPTSCR_TTSARU;
}

#endif /* LWIP_PTP */

/*------------------------------ DMA Functions -------------------------------*/

/**
 * @brief Initialise the transmit descriptor ring with generic attributes
*/
static void init_tx_dma_desc(void)
{
    for (int i = 0; i < STIF_NUM_TX_DMA_DESC; i++) {
        tx_dma_desc[i].Status = ETH_TDES0_TCH | ETH_TDES0_CIC_IPPLPH;
        tx_dma_desc[i].pbuf = NULL;
        tx_dma_desc[i].Buffer2NextDescAddr = &tx_dma_desc[i+1];
    }
    // Chain buffers in a ring
    tx_dma_desc[STIF_NUM_TX_DMA_DESC-1].Buffer2NextDescAddr = &tx_dma_desc[0];

    ETH_DMATDLAR = (uint32_t) tx_dma_desc; // pointer to start of desc. list
    tx_cur_dma_desc = &tx_dma_desc[0];
}

/**
 * @brief Initialise the recieve descriptor ring with generic attributes
*/
static void init_rx_dma_desc(void)
{
    for (int i = 0; i < STIF_NUM_RX_DMA_DESC; i++) {
        rx_dma_desc[i].Status = ETH_RDES0_OWN;
        rx_dma_desc[i].ControlBufferSize = ETH_RDES1_RCH | PBUF_POOL_BUFSIZE;
        rx_dma_desc[i].pbuf = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);
        rx_dma_desc[i].Buffer1Addr = rx_dma_desc[i].pbuf->payload;
        rx_dma_desc[i].Buffer2NextDescAddr = &rx_dma_desc[i+1];
    }
    // Chain buffers in a ring
    rx_dma_desc[STIF_NUM_RX_DMA_DESC-1].Buffer2NextDescAddr = &rx_dma_desc[0];
    ETH_DMARDLAR = (uint32_t) rx_dma_desc; // pointer to start of desc. list
    rx_cur_dma_desc = &rx_dma_desc[0];
}

/**
 * @brief Process Tx descriptor ready for packet transmission
 * This function is designed to work with pbuf chains, so unlike the
 * Rx descriptors which are strictly one pbuf per packet.
 * @param p The pbuf that is being processed (may be part of a chain)
 * @param first A bool that is true if the pbuf is the first in a chain
 * @param last A bool that is true if the pbuf is the last in a chain
*/
static void process_tx_descr(struct pbuf *p, int first, int last)
{
    // wait until the packet is freed by the DMA
    while (tx_cur_dma_desc->Status & ETH_TDES0_OWN);

    // discard old pbuf pointer (in chain)
    if (tx_cur_dma_desc->pbuf != NULL)
        pbuf_free(tx_cur_dma_desc->pbuf);

    // the pbuf field of the descriptor is not used by the DMA
    pbuf_ref(p);
    tx_cur_dma_desc->pbuf = p;

    // tag first and last frames
    tx_cur_dma_desc->Status &= ~(ETH_TDES0_FS | ETH_TDES0_LS);
    if (first)
        tx_cur_dma_desc->Status |= ETH_TDES0_FS;
    if (last)
        tx_cur_dma_desc->Status |= ETH_TDES0_LS;

    tx_cur_dma_desc->Buffer1Addr = p->payload;
    tx_cur_dma_desc->ControlBufferSize = p->len;

    // Pass ownership back to DMA
    tx_cur_dma_desc->Status |= ETH_TDES0_OWN;

    tx_cur_dma_desc = tx_cur_dma_desc->Buffer2NextDescAddr;
}

/**
 * @brief Process Rx descriptor, pass to lwIP, then allocate a new pbuf to the
 * descriptor
 * @param netif network interface struct
*/
static int process_rx_descr(struct netif *netif)
{
    /* Descriptor 'sanity checks' */
    if(rx_cur_dma_desc->Status & ETH_RDES0_OWN)
        return 1;   // Descriptor is still owned by the DMA controller

    if(rx_cur_dma_desc->pbuf == NULL)
        return 1;   // No pbuf was allocated to this descriptor!

    if(!((ETH_RDES0_LS | ETH_RDES0_FS) ==
                (rx_cur_dma_desc->Status & (ETH_RDES0_LS | ETH_RDES0_FS))))
        return 1;   // In store and forward mode this should never trigger


    int frame_length = (rx_cur_dma_desc->Status & ETH_RDES0_FL)
                                                >> ETH_RDES0_FL_SHIFT;

    rx_cur_dma_desc->pbuf->tot_len = frame_length;
    rx_cur_dma_desc->pbuf->len = frame_length;

    /// @todo check frame validity + frame length +/- CRC?

    /* Copy PTP timestamps to pbuf */
    #if LWIP_PTP
        rx_cur_dma_desc->pbuf->tv_sec = rx_cur_dma_desc->TimeStampHigh;
        rx_cur_dma_desc->pbuf->tv_nsec = rx_cur_dma_desc->TimeStampLow;
    #endif /* LWIP_PTP */

    /* Pass packet to lwIP */
    if (netif->input(rx_cur_dma_desc->pbuf, netif) != ERR_OK) {
        LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
        pbuf_free(rx_cur_dma_desc->pbuf); // Free if lwIP won't take the packet
    }

    /* Reallocate new pbuf to descriptor */
    rx_cur_dma_desc->pbuf = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);
    if (rx_cur_dma_desc->pbuf == NULL)
        return 1;   // Pbuf allocation failed!

    rx_cur_dma_desc->Buffer1Addr = rx_cur_dma_desc->pbuf->payload;
    rx_cur_dma_desc->Status = ETH_RDES0_OWN;   // Pass ownership back to DMA

    if (ETH_DMASR & ETH_DMASR_RBUS) {
        ETH_DMASR = ETH_DMASR_RBUS; // Acknowledge DMA if in suspended state
        ETH_DMARPDR = 0;
    }

    rx_cur_dma_desc = rx_cur_dma_desc->Buffer2NextDescAddr; // Next descriptor

    return 0;
}

/*------------------------------- Phy Functions ------------------------------*/

/**
 * @brief gets phy autonegotiation status on link change - configures the MAC
 * accordingly
*/
static err_t phy_negotiate(void)
{
    int regval = ETH_MACCR;
    regval &= ~(ETH_MACCR_DM | ETH_MACCR_FES); // Clear mode and duplex bits

    eth_smi_write(PHY_ADDRESS, PHY_REG_BCR, PHY_REG_BCR_AN); // Autonegotiate

    #ifdef PHY_LAN8742A
        int status;
        while(!((status=eth_smi_read(PHY_ADDRESS, LAN87XX_SCSR))
                & LAN87XX_SCSR_AUTODONE));  // Wait for autonegotiation to finish
        switch(status & LAN87XX_SCSR_SPEED) {
            case LAN87XX_SCSR_SPEED_10HD:
                break;
            case LAN87XX_SCSR_SPEED_100HD:
                regval |= ETH_MACCR_FES;
                break;
            case LAN87XX_SCSR_SPEED_10FD:
                regval |= ETH_MACCR_DM;
                break;
            case LAN87XX_SCSR_SPEED_100FD:
                regval |= ETH_MACCR_FES;
                regval |= ETH_MACCR_DM;
                break;
        }
    #endif /* PHY_LAN8742A */
    #ifdef PHY_KS8081
        #error "This PHY is not implemented yet!"
    #endif /* PHY_KS8081 */

    LWIP_DEBUGF(NETIF_DEBUG, ("mac_init: autonegotiate status: %d\n", regval));

    ETH_MACCR = regval;     // Write speed and duplex to control register

    return ERR_OK;
}

/*------------------------------- FreeRTOS Tasks -----------------------------*/

/**
 * @brief Task for handling incoming ethernet packets (triggered by ISR)
 * @param argument = <i>netif</i> struct from lwIP
*/
static void ethernetif_input(void* argument)
{
    struct netif *netif = (struct netif *) argument;

    /* Block the task before running for the first time */
    configASSERT(eth_task == NULL);
    eth_task = xTaskGetCurrentTaskHandle();

    for(;;) {
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY); // Block until ISR releases

        /* Reset the task notifier for next ISR */
        configASSERT(eth_task == NULL);
        eth_task = xTaskGetCurrentTaskHandle();

        if(process_rx_descr(netif)) {
            LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: descriptor error!\n"));
        }

        /** @todo if this task and the ISR get out of sync (under heavy load),
         * the DMA controller and the processing loop could get out of sync.
         * Ideally implement support for 'catch-up'/'overflow' handling.
        */
    }
}

/**
 * @brief Polls the phy every 0.5 seconds to check the status
 * @param argument = <i>netif</i> struct from lwIP
 * The phy hardware interrupt is not available if the phy is providing the
 * reference clock, so a simple poll is used.
*/
static void ethernetif_phy(void* argument)
{
    struct netif *netif = (struct netif *) argument;
    bool link_status = netif_is_link_up(netif);

    phy_reset(PHY_ADDRESS);

    for(;;) {
        if(link_status != phy_link_isup(PHY_ADDRESS)) {
            link_status = !link_status;
            if(link_status == true) {
                netifapi_netif_set_link_up(netif);      // Link up
            }
            else {
                netifapi_netif_set_link_down(netif);    // Link down
            }
        }
        vTaskDelay(500);
    }
}

/*--------------------------- Operation Functions ----------------------------*/

static err_t ethernetif_output(struct netif *netif, struct pbuf *p)
{
    (void)(netif); // Unused variable

    /// @todo create transmit IRQ handler?
    struct pbuf *q;

    // Iterate through pbuf chain until next-> == NULL
    for(q = p; q != NULL; q = q->next)
        process_tx_descr(q, q == p, q->next == NULL);

    // check if DMA is waiting for a descriptor it owns
    if (ETH_DMASR & ETH_DMASR_TBUS) {
        ETH_DMASR = ETH_DMASR_TBUS; // acknowledge
        ETH_DMATPDR = 0; // ask DMA to carry on polling
    }
    /// @todo the step above is not required if the DMA hasn't got through the
    /// previous descriptors before the next packet is sent. potentially this
    /// behaviour should be flagged, as currently nothing is stopping descriptor
    /// overflow.

    return ERR_OK;
}

/**
 * @brief Initialises the ethernet peripheral's registers for use with the
 * provided descriptors
 * @retval lwIP-style error status
*/
static err_t mac_init(void)
{
    /* Soft reset ethernet MAC */
    ETH_DMABMR |= ETH_DMABMR_SR;
    while (ETH_DMABMR & ETH_DMABMR_SR);

    /* Initialize PHY Communication */
    ETH_MACMIIAR = ETH_CLK_060_100MHZ; // Change depending on HCLK speed

    /* Configure ethernet MAC */
    ETH_MACCR |= (ETH_MACCR_FES | ETH_MACCR_ROD |
                   ETH_MACCR_IPCO | ETH_MACCR_DM);

    ETH_MACFFR |= ETH_MACFFR_RA; // No Filtering Currently
    ETH_MACFCR = 0;     // Default Flow Control

    ETH_DMABMR = (ETH_DMABMR_AAB | ETH_DMABMR_FB | ETH_DMABMR_PM_2_1 |
                   (32 << ETH_DMABMR_RDP_SHIFT) |  //RX Burst Length
                   (32 << ETH_DMABMR_PBL_SHIFT)  |  //TX Burst Length
                   ETH_DMABMR_USP | ETH_DMABMR_EDFE);

    ETH_DMAOMR = (ETH_DMAOMR_DTCEFD | ETH_DMAOMR_RSF |
                   ETH_DMAOMR_TSF | ETH_DMAOMR_OSF);

    // Configure interrupts on reception only (PTP may change this later)
    ETH_DMAIER = ETH_DMAIER_RIE | ETH_DMAIER_TIE | ETH_DMAIER_NISE;
    //ETH_DMAIER = ETH_DMAIER_NISE;

    return ERR_OK;
}

/**
 * @brief Initialise the <i>netif</i> struct with the relevant operational
 * settings
 * @param netif network interface struct
 * @retval lwIP-style error status
*/
static err_t net_init(struct netif *netif)
{
    /* Check no netif already exists */
    LWIP_ASSERT("netif != NULL", (netif != NULL));

    netif->name[0] = 'e';
    netif->name[1] = 'n';
    netif->output = etharp_output;
    netif->linkoutput = ethernetif_output;

    #if LWIP_NETIF_HOSTNAME
        /* Initialize interface hostname */
        netif->hostname = LWIP_HOSTNAME;
    #endif /* LWIP_NETIF_HOSTNAME */

    /* set MAC hardware address length */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;

    /* Set MAC address, generate one if not provided */
    #ifdef MAC_ADDR_MANUAL
        netif->hwaddr[0] = MAC_ADDR_0;
        netif->hwaddr[1] = MAC_ADDR_1;
        netif->hwaddr[2] = MAC_ADDR_2;
        netif->hwaddr[3] = MAC_ADDR_3;
        netif->hwaddr[4] = MAC_ADDR_4;
        netif->hwaddr[5] = MAC_ADDR_5;
    #else
        uint32_t signature[3];
        desig_get_unique_id(signature); // Pull ID from ST's unique ID registers

        union word_byte id_addr;
        id_addr.word = signature[2];    // Use LSB of the Unique ID

        netif->hwaddr[0] = 0x00;       // ST's MAC Prefix
        netif->hwaddr[1] = 0x80;
        netif->hwaddr[2] = 0xE1;
        netif->hwaddr[3] = id_addr.byte[1];
        netif->hwaddr[4] = id_addr.byte[2];
        netif->hwaddr[5] = id_addr.byte[3];
    #endif /* MAC_ADDRESS_MANUAL */

    /* Set MAC address in ethernet peripheral */
    eth_set_mac(netif->hwaddr);

    /* Netif flags */
    netif->mtu = 1500;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

    #if LWIP_IGMP
        netif->flags |= NETIF_FLAG_IGMP;
    // netif_set_igmp_mac_filter(netif, mac_filter); // LATER
    #endif

    /// @todo this is where hardware Multicast filtering needs to be implemented

    return ERR_OK;
}

/**
 * @brief Callback for initialising the ethernet interface
 * @param netif network interface struct
 * @retval lwIP-style error status
*/
static err_t ethernetif_init(struct netif *netif)
{
    err_t ret;

    /* Initialise ethernet peripheral */
    if ((ret = mac_init()) != ERR_OK)
        return ret;

    /* Initialize the netif struct */
    if ((ret = net_init(netif)) != ERR_OK)
        return ret;

    #if LWIP_PTP
        /* Enable PTP Timestamping */
        if ((ret = ptp_hw_init(PTP_UPDATE_FINE)) != ERR_OK)
        return ret;

        /* Pass PTP device drivers as fptrs to ptpd-lwIP */
        ptpFunctions_t ptp_functions;
        ptp_functions.ptpGetTime = ptp_get_time;
        ptp_functions.ptpSetTime = ptp_set_time;
        ptp_functions.ptpUpdateCoarse = ptp_update_coarse;
        ptp_functions.ptpUpdateFine = ptp_update_fine;

        /* Initialise ptpd-lwip */
        ptpdInit(&ptp_functions, FREERTOS_PRIORITIES - 5);
        /// @todo may need to be moved
    #endif /* LWIP_PTP */


    /* Initialise DMA Descriptor rings */
    init_tx_dma_desc();
    init_rx_dma_desc();

    /* FreeRTOS task initiation */
    xTaskCreate(ethernetif_phy, "ETH_phy", 350, netif, 2, NULL);
    xTaskCreate(ethernetif_input, "ETH_input", 1024, netif,
                                  configMAX_PRIORITIES-1, NULL);

    /* Enable MAC and DMA transmission and reception */
    eth_start();

    return ERR_OK;
}

/**
 * @brief Initialise MCU hardware for use of the ethernet peripheral
*/
static void eth_hw_init(void)
{
    /* Enable relavant clocks */
    rcc_periph_clock_enable(RCC_ETHMAC);
    rcc_periph_clock_enable(RCC_ETHMACRX);
    rcc_periph_clock_enable(RCC_ETHMACTX);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOG);

    /// @todo Have not initialised PPS pin for debug!


    /* Configure ethernet GPIOs */
    /* GPIOA */
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP,
            GPIO_OSPEED_50MHZ, GPIO_ETH_RMII_MDIO);
    gpio_set_af(GPIOA, GPIO_AF11, GPIO_ETH_RMII_MDIO |
            GPIO_ETH_RMII_REF_CLK | GPIO_ETH_RMII_CRS_DV);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_ETH_RMII_MDIO |
            GPIO_ETH_RMII_REF_CLK | GPIO_ETH_RMII_CRS_DV);

    /* GPIOB */
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP,
            GPIO_OSPEED_50MHZ, GPIO_ETH_RMII_TXD1);
    gpio_set_af(GPIOB, GPIO_AF11, GPIO_ETH_RMII_TXD1);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_ETH_RMII_TXD1);
    /// @todo PPS definition to go here when implemented

    /* GPIOC */
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP,
            GPIO_OSPEED_50MHZ, GPIO_ETH_RMII_MDC);
    gpio_set_af(GPIOC, GPIO_AF11, GPIO_ETH_RMII_MDC |
            GPIO_ETH_RMII_RXD0 | GPIO_ETH_RMII_RXD1);
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_ETH_RMII_MDC |
            GPIO_ETH_RMII_RXD0 | GPIO_ETH_RMII_RXD1);

    /* GPIOG */
    gpio_set_output_options(GPIOG, GPIO_OTYPE_PP,
            GPIO_OSPEED_50MHZ, GPIO_ETH_RMII_TX_EN | GPIO_ETH_RMII_TXD0);
    gpio_set_af(GPIOG, GPIO_AF11, GPIO_ETH_RMII_TX_EN | GPIO_ETH_RMII_TXD0);
    gpio_mode_setup(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE,
            GPIO_ETH_RMII_TX_EN | GPIO_ETH_RMII_TXD0);

    /* Configure to use RMII interface */
    rcc_periph_clock_enable(RCC_SYSCFG);

    rcc_periph_reset_hold(RST_ETHMAC);
    SYSCFG_PMC |= (1 << 23);    // MII_RMII_SEL (use RMII)
    rcc_periph_reset_release(RST_ETHMAC);

    /* NVIC Interrupt Configuration */
    nvic_set_priority(NVIC_ETH_IRQ, 5);
    nvic_enable_irq(NVIC_ETH_IRQ);

    #if LWIP_PTP
        /* PTP-Specific Hardware Initialisation */
        ptp_hw_init(PTP_UPDATE_FINE);
    #endif /* LWIP_PTP */
}

/*---------------------------- CALLBACK FUNCTIONS ----------------------------*/

/**
 * @brief Callback on ethernet status changes (set up/down or address change)
 * @param netif network interface struct
*/
static void ethernetif_status_callback(struct netif *netif)
{
    if(netif_is_up(netif)) {} else {}   // Use later if required
}

/**
 * @brief Callback on ethernet link change - restarts network services and phy
 * configuration
 * @param netif network interface struct
*/
static void ethernetif_link_callback(struct netif *netif)
{
    /// @todo Restart services with a semaphore! + any ETH Mac reinitialisation

    if(netif_is_link_up(netif)) {
        phy_negotiate();                // Blocks until negotiation is finished
        netifapi_netif_set_up(netif);
        #if LWIP_DHCP
            netifapi_dhcp_start(netif);
        #endif /* LWIP_DHCP */
        #if LWIP_IGMP
            LOCK_TCPIP_CORE();
            igmp_start(netif);
            UNLOCK_TCPIP_CORE();
        #endif /* LWIP_IGMP */
    }
    else {
        netifapi_netif_set_down(netif);
        #if LWIP_DHCP
            netifapi_dhcp_stop(netif);
        #endif /* LWIP_DHCP */
        #if LWIP_IGMP
            LOCK_TCPIP_CORE();
            igmp_stop(netif);
            UNLOCK_TCPIP_CORE();
        #endif /* LWIP_IGMP */
    }
}

/*--------------------- PUBLIC DEVICE-SPECIFIC FUNCTIONS ---------------------*/

void portEthInit(void)
{
    /* Hardware (MSP) Configuration */
    eth_hw_init();

    tcpip_init(NULL, NULL);

    /* IP Configuration */
    ip_addr_t ip_addr = {0};
    ip_addr_t net_mask = {0};
    ip_addr_t gw_addr = {0};
    #if !LWIP_DHCP
        IP4_ADDR(&ip_addr, LWIP_IP_0, LWIP_IP_1, LWIP_IP_2, LWIP_IP_3);
        IP4_ADDR(&net_mask, LWIP_NM_0, LWIP_NM_1, LWIP_NM_2, LWIP_NM_3);
        IP4_ADDR(&gw_addr, LWIP_GW_0, LWIP_GW_1, LWIP_GW_2, LWIP_GW_3);
    #endif /* !LWIP_DHCP */

    LOCK_TCPIP_CORE();  // Lock lwIP core while configuring

    /* Add ethernet interface (currently only one interface supported) */
    netif_add(&ethernetif, &ip_addr, &net_mask, &gw_addr, NULL,
                       ethernetif_init, tcpip_input);
    netif_set_default(&ethernetif);

    /* Set callbacks for link events (restarting when cable is unplugged) */
    netif_set_status_callback(&ethernetif, ethernetif_status_callback);
    netif_set_link_callback(&ethernetif, ethernetif_link_callback);

    UNLOCK_TCPIP_CORE();

    LWIP_DEBUGF(NETIF_DEBUG, ("Mac Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
                              ethernetif.hwaddr[0], ethernetif.hwaddr[1],
                              ethernetif.hwaddr[2], ethernetif.hwaddr[3],
                              ethernetif.hwaddr[4], ethernetif.hwaddr[5]));
}

/*------------------------------- ETHERNET ISR -------------------------------*/

/**
 * @brief Hardware ethernet interrupt (see reference manual for details)
*/
void eth_isr(void)
{
    BaseType_t task_yield = pdFALSE;
    if((ETH_DMASR & ETH_DMASR_RS) == ETH_DMASR_RS) {    // Packet Recieved
        configASSERT(eth_task != NULL);
        vTaskNotifyGiveFromISR(eth_task, &task_yield);
        eth_task = NULL;
        portYIELD_FROM_ISR(task_yield);

        ETH_DMASR = ETH_DMASR_RS;
    }
    else if((ETH_DMASR & ETH_DMASR_TS) == ETH_DMASR_TS) {
        //printf("hello!");
    }
    else {  // Error Condition
        // gdb_break();    /// @todo implement separate transmission callbacks
    }
    ETH_DMASR = ETH_DMASR_NIS; // Clear Normal Interrupt Summary
}
