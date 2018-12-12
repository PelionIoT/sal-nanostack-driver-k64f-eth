/*
 * Copyright (c) 2014-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "arm_hal_phy.h"
#include "arm_hal_interrupt.h"
#include "ns_types.h"
#include "k64f_eth_nanostack_port.h"
#include "fsl_enet.h"
#include "fsl_phy.h"
#include "eventOS_event_timer.h"
#define HAVE_DEBUG 1
#include "ns_trace.h"
#include "nsdynmemLIB.h"
#ifdef MBED_CONF_RTOS_PRESENT
#include "cmsis_os.h"
#endif


/* Macro Definitions */
#ifndef MEM_ALLOC
#define MEM_ALLOC ns_dyn_mem_alloc
#endif
#ifndef MEM_FREE
#define MEM_FREE ns_dyn_mem_free
#endif
#define TRACE_GROUP  "Eth"
#define ENET_HDR_LEN                  (14)
#define ENET_RX_RING_LEN              (4)
#define ENET_TX_RING_LEN              (4)
#define ENET_ETH_MAX_FLEN             (1518)
#define ENET_PTR_ALIGN(x,align)       ((void *)(((uintptr_t)(x) + ((align)-1)) & (~(uintptr_t)((align)- 1))))
#define ENET_SIZE_ALIGN(x,align)      (((size_t)(x) + ((align)-1)) & (~(size_t)((align)- 1)))
#define ENET_BuffPtrAlign(n)          ENET_PTR_ALIGN(n, ENET_BUFF_ALIGNMENT)
#define ENET_BuffSizeAlign(n)         ENET_SIZE_ALIGN(n, ENET_BUFF_ALIGNMENT)


/* Function Prototypes*/
static int8_t arm_eth_phy_k64f_address_write(phy_address_type_e address_type, uint8_t *address_ptr);
static void k64f_eth_set_address(uint8_t *address_ptr);
static int8_t arm_eth_phy_k64f_interface_state_control(phy_interface_state_e state, uint8_t x);
static int8_t arm_eth_phy_k64f_tx(uint8_t *data_ptr, uint16_t data_len, uint8_t tx_handle,data_protocol_e data_flow);
static void PHY_LinkStatus_Task(void *y);
static void eth_if_lock(void);
static void eth_if_unlock(void);
static void k64f_eth_initialize(uint8_t *mac_addr);
static int8_t k64f_eth_send(const uint8_t *data_ptr, uint16_t data_len);
static void k64f_eth_receive(volatile enet_rx_bd_struct_t *bdPtr);
static void update_read_buffer(void);
static void ethernet_event_callback(ENET_Type *base, enet_handle_t *handle, enet_event_t event, void *param);
static status_t Initialize_Enet_PHY(ENET_Type *base, uint32_t phyAddr, uint32_t srcClock_Hz);
static status_t AutoNegotiation(ENET_Type *base, uint32_t phyAddr);

/* Callback function to notify stack about the readiness of the Eth module */
void (*driver_readiness_status_callback)(uint8_t, int8_t) = 0;

/* External function from hardware_init.c
 * Initializes the Eth module hardware */
extern void initialize_enet_hardware(void);

/* External ENET functions */
extern uint32_t ENET_GetInstance(ENET_Type *base);

/* Global Eth module Handle*/
static enet_handle_t global_enet_handle;

/* Nanostack generic PHY driver structure */
static phy_device_driver_s eth_device_driver;

#ifdef MBED_CONF_RTOS_PRESENT

/* Thread IDs for the threads we will start */
static osThreadId eth_irq_thread_id;

/* Signals for IRQ thread */
#define SIG_TX  1
#define SIG_RX  2

/* This routine starts a 'Thread' which handles IRQs*/
static void Eth_IRQ_Thread_Create(void);
#endif /*MBED_CONF_RTOS_PRESENT*/

/* Main data structure for keeping Eth module data */
typedef struct Eth_Buf_Data_S {
    /* Pointers to start addresses of TX/RX Buffer descriptor*/
    volatile enet_rx_bd_struct_t *rx_buf_desc_start_addr;
    volatile enet_tx_bd_struct_t *tx_buf_desc_start_addr;
    /* Pointer to Buffers themselves */
    uint8_t *rx_buf[ENET_RX_RING_LEN];
    uint8_t *tx_buf[ENET_TX_RING_LEN];
} Eth_Buf_Data_S;

/* Global Variables */
static Eth_Buf_Data_S base_DS;
static uint8_t eth_driver_enabled = 0;
static int8_t eth_interface_id = -1;
static bool link_currently_up = false;
static const clock_ip_name_t s_enetClock[FSL_FEATURE_SOC_ENET_COUNT] = ENET_CLOCKS;


/** \brief  Function to register the ethernet driver to the Nanostack
 *
 *  \param[in] mac_ptr  Pointer to MAC address
 *  \param[in] driver_status_cb  Function pointer to notify the caller about driver status
 */
void arm_eth_phy_device_register(uint8_t *mac_ptr, void (*driver_status_cb)(uint8_t, int8_t))
{
    if (eth_interface_id < 0) {

        eth_device_driver.PHY_MAC = mac_ptr;
        eth_device_driver.address_write = &arm_eth_phy_k64f_address_write;
        eth_device_driver.driver_description = "ETH";
        eth_device_driver.link_type = PHY_LINK_ETHERNET_TYPE;
        eth_device_driver.phy_MTU = 0;
        eth_device_driver.phy_header_length = 0;
        eth_device_driver.phy_tail_length = 0;
        eth_device_driver.state_control = &arm_eth_phy_k64f_interface_state_control;
        eth_device_driver.tx = &arm_eth_phy_k64f_tx;
        eth_device_driver.phy_rx_cb = NULL;
        eth_device_driver.phy_tx_done_cb = NULL;
        eth_interface_id = arm_net_phy_register(&eth_device_driver);
        driver_readiness_status_callback = driver_status_cb;

        if (eth_interface_id < 0){
            tr_error("Ethernet Driver failed to register with Stack. RetCode=%i", eth_driver_enabled);
            driver_readiness_status_callback(0, eth_interface_id);
            return;
        }
    }

    if (!eth_driver_enabled) {
        k64f_eth_initialize(mac_ptr);
        eth_driver_enabled = 1;
        driver_readiness_status_callback(link_currently_up, eth_interface_id);
#ifdef MBED_CONF_RTOS_PRESENT
        Eth_IRQ_Thread_Create();
#endif
        eventOS_timeout_ms(PHY_LinkStatus_Task, 500, NULL);
    }
}


/** \brief  TX routine used by Nanostack to transmit via Ethernet Interface
 *
 *  \param[in] data_ptr   Pointer to data packet
 *  \param[in] drta_len   Length of the data packet
 *  \param[in] tx_handle  Not used in this context. Safe to pass Null.
 *  \param[in] tdata_flow Not used in this context. Safe to pass Null.
 */
static int8_t arm_eth_phy_k64f_tx(uint8_t *data_ptr, uint16_t data_len, uint8_t tx_handle,data_protocol_e data_flow)
{
    int retval = -1;

    if(data_len >= ENET_HDR_LEN){
        retval = k64f_eth_send(data_ptr, data_len);
    }

    (void)data_flow;
    (void)tx_handle;

    return retval;
}

/* TODO State Control Handling.*/
static int8_t arm_eth_phy_k64f_interface_state_control(phy_interface_state_e state, uint8_t not_required)
{
    switch(state){
        case PHY_INTERFACE_DOWN:
            break;
        case PHY_INTERFACE_UP:
            break;
        case PHY_INTERFACE_RESET:
            break;
        case PHY_INTERFACE_SNIFFER_STATE:
            /*TODO Allow promiscuous state here*/
            break;
        case PHY_INTERFACE_RX_ENERGY_STATE:
            /*Just to get rid of compiler warning*/
            break;

    }

    (void)not_required;

    return 0;
}

/** \brief  Function to update the RX buffer descriptors
 *
 *  Moves the pointer to next buffer descriptor. If its the end of the Ring, wraps
 *  it around.
 */
static void update_read_buffer(void)
{
    /* Clears status. */
    global_enet_handle.rxBdCurrent->control &= ENET_BUFFDESCRIPTOR_RX_WRAP_MASK;

    /* Sets the receive buffer descriptor with the empty flag. */
    global_enet_handle.rxBdCurrent->control |= ENET_BUFFDESCRIPTOR_RX_EMPTY_MASK;

    /* Increases the buffer descriptor to the next one. */
    if (global_enet_handle.rxBdCurrent->control & ENET_BUFFDESCRIPTOR_RX_WRAP_MASK) {
        global_enet_handle.rxBdCurrent = global_enet_handle.rxBdBase;
    } else {
        global_enet_handle.rxBdCurrent++;
    }

    /* Actives the receive buffer descriptor. */
    ENET->RDAR = ENET_RDAR_RDAR_MASK;
}


/** \brief  Function to receive data packets
 *
 *  Reads the data from the buffer provided by the buffer descriptor and hands
 *  it over to the Nanostack.
 *
 *  \param[in] bdPtr  Pointer to the receive buffer descriptor structure
 *  \param[in] idx    Index of the current buffer descriptor
 */
static void k64f_eth_receive(volatile enet_rx_bd_struct_t *bdPtr)
{
    if (!(bdPtr->control & ENET_BUFFDESCRIPTOR_RX_ERR_MASK)) {
        /* Hand it over to Nanostack*/
        if (eth_device_driver.phy_rx_cb) {
            eth_device_driver.phy_rx_cb(bdPtr->buffer, bdPtr->length, 0xff,
                                                 0, eth_interface_id);
        }
    }
}

/** \brief  Function to reclaim used TX buffer descriptor
 *
 *  This function is called after a TX interrupt takes place. The interrupt will
 *  kick the IRQ thread which will eventually call this function.
 */
static void tx_queue_reclaim(void)
{
    // Traverse all descriptors, looking for the ones modified by the uDMA
    while (!(global_enet_handle.txBdDirty->control & ENET_BUFFDESCRIPTOR_TX_READY_MASK) && global_enet_handle.txBdDirty->buffer) {
        uint8_t i = global_enet_handle.txBdDirty - global_enet_handle.txBdBase;
        global_enet_handle.txBdDirty->buffer = NULL;
        MEM_FREE(base_DS.tx_buf[i]);
        base_DS.tx_buf[i] = NULL;

        if (global_enet_handle.txBdDirty->control & ENET_BUFFDESCRIPTOR_TX_WRAP_MASK) {
            global_enet_handle.txBdDirty = global_enet_handle.txBdBase;
        } else {
            global_enet_handle.txBdDirty++;
        }
    }
}


/** \brief  Function to send data packets
 *
 * This function is called by arm_eth_phy_tx() which is in turn called through
 * Nanostack.
 *
 *  \param[in] data_ptr  Pointer to the data buffer
 *  \param[in] data_len  Length of the data
 *
 *  \returns 0 if successfull, <0 if unsuccessful
 */
static int8_t k64f_eth_send(const uint8_t *data_ptr, uint16_t data_len)
{
    /*Sanity Check*/
    if (data_len > ENET_ETH_MAX_FLEN) {
        tr_error("Packet size bigger than ENET_TXBuff_SIZE.");
        return -1;
    }

    /* Get the index of the next TX descriptor */
    uint8_t index = global_enet_handle.txBdCurrent - global_enet_handle.txBdBase;

    /* Check if next descriptor is free - reclaim should have freed tx_buf */
    if (base_DS.tx_buf[index]) {
        tr_error("TX buf descriptors full. Can't queue packet.");
        return -1;
    }

    uint8_t *buf_ptr = MEM_ALLOC(data_len + ENET_BUFF_ALIGNMENT);
    if (!buf_ptr) {
        tr_error("Alloc failed");
        return -1;
    }

    // Protect against reclaim routine swallowing tx_buf before we set READY
    eth_if_lock();
    base_DS.tx_buf[index] = buf_ptr;
    uint8_t *aligned_ptr = ENET_BuffPtrAlign(buf_ptr);
    memcpy(aligned_ptr, data_ptr, data_len);

    /* Setup transfers */
    global_enet_handle.txBdCurrent->buffer = aligned_ptr;
    global_enet_handle.txBdCurrent->length = data_len;
    global_enet_handle.txBdCurrent->control |=
            (ENET_BUFFDESCRIPTOR_TX_READY_MASK
                    | ENET_BUFFDESCRIPTOR_TX_LAST_MASK);

    /* Increase the buffer descriptor address. */
    if (global_enet_handle.txBdCurrent->control
            & ENET_BUFFDESCRIPTOR_TX_WRAP_MASK) {
        global_enet_handle.txBdCurrent = global_enet_handle.txBdBase;
    } else {
        global_enet_handle.txBdCurrent++;
    }
    eth_if_unlock();

    /* Active the transmit buffer descriptor. */
    ENET->TDAR = ENET_TDAR_TDAR_MASK;

    return 0;
}


/** \brief  Sets up MAC address
 *
 * This function is calle by arm_eth_phy_k64f_address_write() which is in turn
 * called by nanostack in order to set up MAC address
 *
 *  \param[in] address_ptr   Pointer to MAC address block
 */
static void k64f_eth_set_address(uint8_t *address_ptr)
{
    /* When pointer to the MAC address is given. It could be 48-bit EUI generated
     * from Radio, like atmel RF or manually inserted. Preferred method.*/
    ENET_SetMacAddr(ENET, address_ptr);
}


/** \brief  Stack sets the MAC address using this routine
 *
 * This function sets the MAC address and its type for the Ethernet interface
 * Only type supported is 48 bits.
 *
 *  \param[in] address_type   Type of MAC address, i.e., 48 bit, 64 bit etc.
 *  \param[in] address_ptr    Pointer to MAC address
 *
 *  \return  0 if successful <0 if unsuccessful
 */
static int8_t arm_eth_phy_k64f_address_write(phy_address_type_e address_type, uint8_t *address_ptr)
{
    int8_t retval = 0;

    switch(address_type){
        case PHY_MAC_48BIT:
            k64f_eth_set_address(address_ptr);
            break;
        case PHY_MAC_64BIT:
        case PHY_MAC_16BIT:
        case PHY_MAC_PANID:
            retval=-1;
            break;
    }

    return retval;
}


/** \brief  Task check the status of PHY link
 *
 *      This task PHY link status and tells the stack if the Eth cable is
 *      connected or not. Checks the status after every 500 millisecond.
 *
 *  \param[in] Optional user-given parameter
 */
static void PHY_LinkStatus_Task(void *y)
{
    bool link = false;

    eth_if_lock();
    PHY_GetLinkStatus(ENET, 0, &link);
    if (link != link_currently_up) {
        link_currently_up = link;
        if (link) {
            phy_duplex_t phy_duplex;
            phy_speed_t phy_speed;
            AutoNegotiation(ENET, 0);
            PHY_GetLinkSpeedDuplex(ENET, 0, &phy_speed, &phy_duplex);
            /* Poke the registers*/
            ENET_SetMII(ENET, (enet_mii_speed_t)phy_speed, (enet_mii_duplex_t)phy_duplex);
        }
        eth_if_unlock();
        driver_readiness_status_callback(link, eth_interface_id);
        tr_info ("Ethernet cable %s.", link ? "connected" : "unplugged");
    } else {
        eth_if_unlock();
    }

    eventOS_timeout_ms(PHY_LinkStatus_Task, 500, NULL);
}


/** \brief  Initialization function for Ethernet module.
 *
 *      This function initializes the ethernet module using default
 *      configuration. Link speed and duplex is auto-negotiated. It also sets up
 *      the buffer descriptors. If MAC address is setup later by the stack using
 *      k64f_eth_address_set() routine, that will just overwrite the MAC directly
 *      in Hardware registers.
 *
 *  \param[in] mac_addr Pointer to MAC address.
 *
 */
static void k64f_eth_initialize(uint8_t *mac_addr)
{
    uint32_t sysClock;
    phy_speed_t phy_speed;
    phy_duplex_t phy_duplex;
    uint32_t phyAddr = 0;

    enet_config_t config;


    /* Allocate TX buffer descriptors */
    void *memptr = MEM_ALLOC(
            sizeof(enet_tx_bd_struct_t) * ENET_TX_RING_LEN + ENET_BUFF_ALIGNMENT);
    if (!memptr) {
        tr_error("Could not allocate memory for TX Buf Descriptors.");
        return;
    }
    base_DS.tx_buf_desc_start_addr = ENET_BuffPtrAlign(memptr);
    memset((enet_tx_bd_struct_t *) base_DS.tx_buf_desc_start_addr, 0,
           sizeof(enet_tx_bd_struct_t) * ENET_TX_RING_LEN);

    /* Allocate RX buffer descriptors */
    memptr = MEM_ALLOC( sizeof(enet_rx_bd_struct_t) * ENET_RX_RING_LEN + ENET_BUFF_ALIGNMENT);
    if (!memptr) {
        tr_error("Could not allocate memory for RX Buf Descriptors");
        return;
    }
    base_DS.rx_buf_desc_start_addr = ENET_BuffPtrAlign(memptr);
    memset((enet_rx_bd_struct_t *) base_DS.rx_buf_desc_start_addr, 0,
           sizeof(enet_rx_bd_struct_t) * ENET_RX_RING_LEN);

    /* Allocate RX buffers in one contiguous block */
    memptr = MEM_ALLOC(ENET_BuffSizeAlign(ENET_ETH_MAX_FLEN) * ENET_RX_RING_LEN + ENET_BUFF_ALIGNMENT);
    if (!memptr) {
        tr_error("Could not allocate memory for RX Buffers");
        return;
    }
    base_DS.rx_buf[0] = ENET_BuffPtrAlign(memptr);

    /* Fill in pointers to following buffers */
    for (int i = 1; i < ENET_RX_RING_LEN; i++) {
        base_DS.rx_buf[i] = base_DS.rx_buf[i-1] + ENET_BuffSizeAlign(ENET_ETH_MAX_FLEN);
    }

    /* Preparing the Buffer Configurations */
    enet_buffer_config_t buf_config = {
        .rxBdNumber = ENET_RX_RING_LEN,
        .txBdNumber = ENET_TX_RING_LEN,
        .rxBuffSizeAlign = ENET_BuffSizeAlign(ENET_ETH_MAX_FLEN),
        .txBuffSizeAlign = 0,
        .rxBdStartAddrAlign = base_DS.rx_buf_desc_start_addr,
        .txBdStartAddrAlign = base_DS.tx_buf_desc_start_addr,
        // it appears this is a wrongly-typed pointer to an array of buffer pointers
        .rxBufferAlign = (uint8_t *) base_DS.rx_buf,
        .txBufferAlign = NULL
    };

    /* Initialize the Ethernet Hardware */
    initialize_enet_hardware();

    /* Setup Clock for ENET module */
    sysClock = CLOCK_GetFreq(kCLOCK_CoreSysClk);

    /* Load the Default config (RMII Mode, Full Duplex, 100 Mbps)*/
    ENET_GetDefaultConfig(&config);

    /* Initialize PHY layer */
    if(Initialize_Enet_PHY(ENET, phyAddr, sysClock)!=kStatus_Success) {
        tr_error("Couldn't initialize Ethernet PHY layer.");
        return;
    }

    PHY_GetLinkSpeedDuplex(ENET, phyAddr, &phy_speed, &phy_duplex);
    /* Change the MII speed and duplex for actual link status. */
    config.miiSpeed = (enet_mii_speed_t) phy_speed;
    config.miiDuplex = (enet_mii_duplex_t) phy_duplex;

    config.rxMaxFrameLen = ENET_ETH_MAX_FLEN;
    config.macSpecialConfig = kENET_ControlFlowControlEnable;
    config.rxAccelerConfig = kENET_RxAccelMacCheckEnabled;
    config.interrupt = kENET_RxFrameInterrupt | kENET_TxFrameInterrupt;

    /* Initialize ENET module */
    ENET_Init(ENET, &global_enet_handle, &config, &buf_config, mac_addr,
              sysClock);

    /* Adding Multicast Group
     * Even ksdk 2.0 API is weird.
     * ENET_AddMulticastGroup(ENET_Type *base, uint8_t *address),
     * this routine will need 64 addresses to set a multicast group.
     * So let's hack it. For more details->Kevin*/
    ENET->GAUR = 0xFFFFFFFFu;
    ENET->GALR = 0xFFFFFFFFu;

    /* Setup callback for Interrupt routines */
    ENET_SetCallback(&global_enet_handle, ethernet_event_callback, NULL);

    /* Lastly, activate the module */
    ENET_ActiveRead(ENET);
}


/** \brief  Function to get a lock on the Thread.
 * An abstraction of platform_enter_critical.
 * In RTOS Mode: It claims a mutex and protects the foreground operations from
 *               background stuff.
 * In Non-RTOS Mode: It disables interrupts and hence protects foreground Ops
 *               from background ones.
 */
static void eth_if_lock(void)
{
    platform_enter_critical();
}


/** \brief  Function to release a lock from the Thread.
 * An abstraction of platform_exit_critical.
 * In RTOS Mode: Makes sure that we have released the mutex.
 * In Non-RTOS Mode: Enables interrupts etc.
 */
static void eth_if_unlock(void)
{
    platform_exit_critical();
}


/** \brief  Interrupt Service Routine for RX IRQ
 *      If Mbed RTOS is being used, this routine will be called from the
 *      Eth_IRQ_Thread (thread context) after receiving the signal from
 *      interrupt context (SIG_RX).
 *      Otherwise, it will be called directly from the interrupt context.
 */
static void enet_rx_task(void)
{
    while (!(global_enet_handle.rxBdCurrent->control & ENET_BUFFDESCRIPTOR_RX_EMPTY_MASK)) {
        k64f_eth_receive(global_enet_handle.rxBdCurrent);
        update_read_buffer();
    }
}


/** \brief  Interrupt Service Routine for TX IRQ
 *      If Mbed RTOS is being used, this routine will be called from the
 *      Eth_IRQ_Thread (thread context) after receiving the signal from
 *      interrupt context (SIG_TX).
 *      Otherwise, it will be called directly from the interrupt context.
 */
static void enet_tx_task(void)
{
    tx_queue_reclaim();
}

/** \brief  Callback function to receive Ethernet TX/RX Events
 *
 *  Called from the interrupt context. If we are not using mbed RTOS, we call up
 *  the ISRs, otherwise we send a corresponding signal to the responsible Thread.
 *
 *  \param[in] base  Pointer to base structure of Eth module
 *  \param[in] handle Pointer to global ethernet handle
 *  \param[in] event  Type of interrupt or any other event (what we have
 *             registered in init)
 *  \param[in] param A pointer to user provided data|parameter. [optional]
 */
static void ethernet_event_callback(ENET_Type *base, enet_handle_t *handle, enet_event_t event, void *param)
{
    switch (event) {
        case kENET_RxEvent:
#ifdef MBED_CONF_RTOS_PRESENT
            osSignalSet(eth_irq_thread_id, SIG_RX);
#else
            enet_rx_task();
#endif /*MBED_CONF_RTOS_PRESENT*/
            break;
        case kENET_TxEvent:
#ifdef MBED_CONF_RTOS_PRESENT
            osSignalSet(eth_irq_thread_id, SIG_TX);
#else
            enet_tx_task();
#endif /*MBED_CONF_RTOS_PRESENT*/
            break;
        default:
            break;
    }
}

static status_t Initialize_Enet_PHY(ENET_Type *base, uint32_t phyAddr, uint32_t srcClock_Hz)
{
    status_t result = kStatus_Success;
    uint32_t instance = ENET_GetInstance(base);

    /* Set SMI first. */
    CLOCK_EnableClock(s_enetClock[instance]);
    ENET_SetSMI(base, srcClock_Hz, false);

    /* Reset PHY. */
    result = PHY_Write(base, phyAddr, PHY_BASICCONTROL_REG, PHY_BCTL_RESET_MASK);
    return result;
}

static status_t AutoNegotiation(ENET_Type *base, uint32_t phyAddr)
{
    uint32_t bssReg;
    status_t result;
    uint32_t counter = 0xFFFFFFU;

    /* Set the negotiation. */
    result = PHY_Write(base, phyAddr, PHY_AUTONEG_ADVERTISE_REG,
                      (PHY_100BASETX_FULLDUPLEX_MASK | PHY_100BASETX_HALFDUPLEX_MASK
                      | PHY_10BASETX_FULLDUPLEX_MASK | PHY_10BASETX_HALFDUPLEX_MASK
                      | 0x1U));
    if (result == kStatus_Success) {
        result = PHY_Write(base, phyAddr, PHY_BASICCONTROL_REG,
                (PHY_BCTL_AUTONEG_MASK | PHY_BCTL_RESTART_AUTONEG_MASK));
        if (result == kStatus_Success) {
            /* Check auto negotiation complete. */
            while (counter--) {
                result = PHY_Read(base, phyAddr, PHY_BASICSTATUS_REG, &bssReg);
                if (result == kStatus_Success) {
                    if ((bssReg & PHY_BSTATUS_AUTONEGCOMP_MASK) != 0) {
                        break;
                    }
                }

                if (!counter) {
                    return kStatus_PHY_AutoNegotiateFail;
                }
            }
        }
    }

    return result;
}

#ifdef MBED_CONF_RTOS_PRESENT
/** \brief  Thread started by ETH_IRQ_Thread_Create
 *
 *      Used only in case of  mbed RTOS. This Thread handles the signals coming
 *      from interrupt context.
 *
 *  \param[in] Optional user-given parameter
 */
static void Eth_IRQ_Thread(const void *x)
{
    for (;;) {
        osEvent event =  osSignalWait(0, osWaitForever);
        if (event.status != osEventSignal) {
            continue;
        }

        eth_if_lock();

        if (event.value.signals & SIG_RX) {
            enet_rx_task();
        }

        if (event.value.signals & SIG_TX) {
            enet_tx_task();
        }

        eth_if_unlock();
    }
}


/** \brief  Function creating the IRQ thread
 *
 *      Used only in case of  mbed RTOS. Creates a thread for IRQ task.
 */
static void Eth_IRQ_Thread_Create(void)
{
    static osThreadDef(Eth_IRQ_Thread, osPriorityRealtime, 512);
    eth_irq_thread_id = osThreadCreate(osThread(Eth_IRQ_Thread), NULL);
}
#endif /*MBED_CONF_RTOS_PRESENT*/



