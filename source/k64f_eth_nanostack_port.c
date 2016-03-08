/*
 * Copyright (c) 2016, ARM Limited, All Rights Reserved
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
#include <stdbool.h>
#include "platform/arm_hal_phy.h"
#include "platform/arm_hal_interrupt.h"
#include "ns_types.h"
#include "k64f_eth_nanostack_port.h"
#include "fsl_enet_driver.h"
#include "fsl_enet_hal.h"
#include "fsl_phy_driver.h"
#include "mbed-drivers/mbed_interface.h"
#include "fsl_interrupt_manager.h"
#define HAVE_DEBUG 1
#include "ns_trace.h"
#include "ns_list.h"
#include "nsdynmemLIB.h"


/* Any Pre-processor Macros*/
#define AUTO_INDEX (-1)
#ifndef MEM_ALLOC
#define MEM_ALLOC ns_dyn_mem_alloc
#endif
#ifndef MEM_FREE
#define MEM_FREE ns_dyn_mem_free
#endif
#define TRACE_GROUP  "ethDrv"

/* Function Prototypes*/
static int8_t arm_eth_phy_k64f_address_write(phy_address_type_e address_type, uint8_t *address_ptr);
static void k64f_eth_set_address(uint8_t *address_ptr);
static int8_t arm_eth_phy_k64f_interface_state_control(phy_interface_state_e state, uint8_t x);
static int8_t arm_eth_phy_k64f_tx(uint8_t *data_ptr, uint16_t data_len, uint8_t tx_handle,data_protocol_e data_flow);
void (*driver_readiness_status_callback)(uint8_t, int8_t) = 0;
void eth_enable_interrupts(void);
void eth_disable_interrupts(void);
static int8_t k64f_eth_initialize(void);
static int8_t k64f_eth_send(uint8_t *data_ptr, uint16_t data_len);
extern void k64f_init_eth_hardware(void);

/*Internal Global Variables and declaration of data structures*/
static bool eth_driver_enabled = false;
static int8_t eth_interface_id = -1;
static phy_device_driver_s eth_device_driver;
static enet_dev_if_t ethernet_iface[HW_ENET_INSTANCE_COUNT];
static enet_mac_config_t ethernet_mac_config[HW_ENET_INSTANCE_COUNT];
static enet_phy_config_t enetPhyCfg[HW_ENET_INSTANCE_COUNT] =
{
  {0, false}
};

typedef struct Ethernet_BufferDesc_Ring_t{
    volatile uint32_t rx_free_desc; /* No. of free rx buffer descriptors*/
    volatile uint32_t tx_free_desc; /* No. of free tx buffer descriptors*/
    uint8_t *rx_buf_desc_start_addr; /* Pointer to RX buffer descriptor start address*/
    uint8_t *tx_buf_desc_start_addr; /* Pointer to TX buffer descriptor start address*/
    uint8_t rx_fill_index; /* tells how much RX buffer descriptor ring is filled already*/
    uint8_t tx_buf_busy_index;
    uint8_t tx_buf_free_index;
    uint8_t *rx_data_buf_ptr[ENET_RX_RING_LEN];
    uint8_t *tx_data_buf_ptr[ENET_TX_RING_LEN];
    void *txb_aligned[ENET_TX_RING_LEN]; /**< TX aligned buffers (if needed) */
}Ethernet_BufferDesc_Ring_t;

extern IRQn_Type enet_irq_ids[HW_ENET_INSTANCE_COUNT][FSL_FEATURE_ENET_INTERRUPT_COUNT];
extern uint8_t enetIntMap[kEnetIntNum];
static uint32_t device_id = BOARD_DEBUG_ENET_INSTANCE_ADDR;
static uint8_t isAutoIndex = 0;

ENET_Type *const g_enetBase[] = ENET_BASE_PTRS;

static Ethernet_BufferDesc_Ring_t buffer_descriptor_ring[HW_ENET_INSTANCE_COUNT];

/* This function registers the ethernet driver to the Nanostack */
/* After registration to the stack, it initializez the driver itself*/
void arm_eth_phy_device_register(uint8_t *mac_ptr, void (*driver_status_cb)(uint8_t, int8_t)){

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
        eth_interface_id = arm_net_phy_register(&eth_device_driver);
        driver_readiness_status_callback = driver_status_cb;

        if (eth_interface_id < 0){
            tr_error("Ethernet Driver failed to register with Stack. RetCode=%i", eth_driver_enabled);
            driver_readiness_status_callback(0, eth_interface_id);
            return;
        }
    }

    if (!eth_driver_enabled) {
        int8_t ret = k64f_eth_initialize();
        if (ret==-1) {
            tr_error("Failed to Initialize Ethernet Driver.");
            driver_readiness_status_callback(0, eth_interface_id);
        } else {
            tr_info("Ethernet Driver Initialized.");
            eth_driver_enabled = true;
            driver_readiness_status_callback(1, eth_interface_id);
        }
    }
}

/* This function is called by Nanostack in order to spit out packets through
 * Ethernet Interface */
static int8_t arm_eth_phy_k64f_tx(uint8_t *data_ptr, uint16_t data_len, uint8_t tx_handle,data_protocol_e data_flow){

    int retval = -1;


    if(data_len >= ENET_HDR_LEN){
        //tr_debug("Nanostack wishes to transmit. Data_len=%i",data_len);
        platform_enter_critical();
        retval = k64f_eth_send(data_ptr, data_len);
        platform_exit_critical();
    }

    (void)data_flow;
    (void)tx_handle;


    return retval;
}

/* TODO State Control Handling.*/
static int8_t arm_eth_phy_k64f_interface_state_control(phy_interface_state_e state, uint8_t not_required){

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

/* Initializes the Receive queue for Receive buffer descriptors and also re-queues
 * any used buffers */
static void rx_queue(Ethernet_BufferDesc_Ring_t *buf_desc_ring, uint8_t *data_buf_ptr, int8_t index){

    uint8_t *aligned_ptr = (uint8_t*)ENET_ALIGN((uint32_t)data_buf_ptr, RX_BUF_ALIGNMENT);
    enet_bd_struct_t *start = (enet_bd_struct_t *)buf_desc_ring->rx_buf_desc_start_addr;
    uint8_t idx;
    /* Get next free descriptor index */
    if (index == AUTO_INDEX){
        idx = buf_desc_ring->rx_fill_index;
        isAutoIndex = 1;
    }
    else{
        idx = index;
    }

    /* Setup descriptor and clear statuses */
    enet_hal_init_rxbds(start + idx, aligned_ptr, idx == ENET_RX_RING_LEN - 1);
    buf_desc_ring->rx_data_buf_ptr[idx] = data_buf_ptr;
    /* Wrap at end of descriptor list */
    idx = (idx + 1) % ENET_RX_RING_LEN;
    /* Update index of free descriptors*/
    buf_desc_ring->rx_free_desc -= 1;
    if (isAutoIndex){
        buf_desc_ring->rx_fill_index = idx;
        isAutoIndex = 0;
    }
    /* Activate current buffer descriptor */
    enet_hal_active_rxbd(BOARD_DEBUG_ENET_INSTANCE_ADDR);
   // tr_debug("Free RX buffer descriptors no. is = %i", (uint8_t) buf_desc_ring->rx_free_desc);
}

/* Allocates an alligned bunch of memory for receive queue usage.*/
static int8_t rx_queue_alligned_to_buf_desc(Ethernet_BufferDesc_Ring_t *buf_desc_ring, int8_t index){


    enet_dev_if_t *ethernet_iface_ptr = (enet_dev_if_t *)&ethernet_iface[BOARD_DEBUG_ENET_INSTANCE];


    while(buf_desc_ring->rx_free_desc > 0){
        /* Allocate memory for received data buffer and allign it to the
         * corresponding buffer descriptor*/
        uint8_t *buf_ptr = MEM_ALLOC(ethernet_iface_ptr->macCfgPtr->rxBufferSize + RX_BUF_ALIGNMENT);
        if (!buf_ptr){
            tr_error("Could not allocate memory. rx_queue_alligned_to_buf_desc.");
            return -1;
        }
        //buf_ptr = (uint8_t*)ENET_ALIGN((uint32_t)buf_ptr, RX_BUF_ALIGNMENT);
        rx_queue(buf_desc_ring, buf_ptr, index);
    }

    return 0;
}

/*Setup for Reveive Buffer Desciptors*/
static int8_t k64f_eth_rx_buf_desc_setup(Ethernet_BufferDesc_Ring_t *buf_desc_ring, enet_rxbd_config_t *rxbdCfg){

    enet_dev_if_t *ethernet_iface_ptr = (enet_dev_if_t *)&ethernet_iface[BOARD_DEBUG_ENET_INSTANCE];

    uint8_t *rxBdPtr;
    uint32_t rxBufferSizeAligned;

    /* Allocate memory to RX buffer descriptors - 29x16+8 Bytes*/
    rxBdPtr = MEM_ALLOC(enet_hal_get_bd_size()*ethernet_iface_ptr->macCfgPtr->rxBdNumber+ENET_BD_ALIGNMENT);

    if(!rxBdPtr){
        tr_error("Could not allocate memory for rx buffer descriptors.");
        return -1;
      }

    buf_desc_ring->rx_buf_desc_start_addr = (uint8_t *)ENET_ALIGN((uint32_t)rxBdPtr, ENET_BD_ALIGNMENT);
    buf_desc_ring->rx_free_desc = ethernet_iface_ptr->macCfgPtr->rxBdNumber;
    buf_desc_ring->rx_fill_index = 0;

    rxBufferSizeAligned = ENET_ALIGN(ethernet_iface_ptr->macCfgPtr->rxBufferSize, ENET_RX_BUFFER_ALIGNMENT);
    ethernet_iface_ptr->macContextPtr->rxBufferSizeAligned = rxBufferSizeAligned;
    rxbdCfg->rxBdPtrAlign = buf_desc_ring->rx_buf_desc_start_addr ;
    rxbdCfg->rxBdNum = ethernet_iface_ptr->macCfgPtr->rxBdNumber;
    rxbdCfg->rxBufferNum = ethernet_iface_ptr->macCfgPtr->rxBdNumber;

    memset(rxbdCfg->rxBdPtrAlign, 0, enet_hal_get_bd_size()*ethernet_iface_ptr->macCfgPtr->rxBdNumber);

    rx_queue_alligned_to_buf_desc(buf_desc_ring, AUTO_INDEX);

    return 0;
}

/*Setup for Transmit Buffer Desciptors*/
static int8_t k64f_eth_tx_buf_desc_setup(Ethernet_BufferDesc_Ring_t *buf_desc_ring, enet_txbd_config_t *txbdCfg){

    uint8_t *txBdPtr;
    enet_dev_if_t *ethernet_iface_ptr = (enet_dev_if_t *)&ethernet_iface[BOARD_DEBUG_ENET_INSTANCE];

    /* Allocate memory to TX buffer descriptors */
    txBdPtr = MEM_ALLOC(enet_hal_get_bd_size() * ethernet_iface_ptr->macCfgPtr->txBdNumber + ENET_BD_ALIGNMENT);
    if(!txBdPtr){
        tr_error("Could not allocate memory for tx buffer descriptors.");
        return -1;
    }

    buf_desc_ring->tx_buf_desc_start_addr = (uint8_t *)ENET_ALIGN((uint32_t)txBdPtr, ENET_BD_ALIGNMENT);
    buf_desc_ring->tx_free_desc = ethernet_iface_ptr->macCfgPtr->txBdNumber;
    buf_desc_ring->tx_buf_busy_index = buf_desc_ring->tx_buf_free_index = 0;

    txbdCfg->txBdPtrAlign = buf_desc_ring->tx_buf_desc_start_addr;
    txbdCfg->txBufferNum = ethernet_iface_ptr->macCfgPtr->txBdNumber;
    txbdCfg->txBufferSizeAlign = ENET_ALIGN(ethernet_iface_ptr->maxFrameSize, ENET_TX_BUFFER_ALIGNMENT);

    memset(txbdCfg->txBdPtrAlign, 0, enet_hal_get_bd_size() * ethernet_iface_ptr->macCfgPtr->txBdNumber);

    /* Initialize TX descriptor ring */
    enet_hal_init_txbds(buf_desc_ring->tx_buf_desc_start_addr+ enet_hal_get_bd_size() * (ENET_TX_RING_LEN- 1), 1);

    return 0;
}

/* Sets up the receive buffer ready to be collected by Nanostack */
static uint8_t *Buf_to_Nanostack(Ethernet_BufferDesc_Ring_t *buf_desc_ring, uint8_t idx, uint16_t *data_length){

    enet_bd_struct_t *bdPtr = (enet_bd_struct_t*)buf_desc_ring->rx_buf_desc_start_addr;
    const uint16_t err_mask = kEnetRxBdTrunc | kEnetRxBdCrc | kEnetRxBdNoOctet | kEnetRxBdLengthViolation;
    uint8_t *rx_data_buf_ptr = NULL;

    /* If the recieved packet is in error, discard it and empty the logical
     * descriptor*/
    if ((bdPtr[idx].control & err_mask) != 0) {
        rx_data_buf_ptr = buf_desc_ring->rx_data_buf_ptr[idx];
        buf_desc_ring->rx_data_buf_ptr[idx] = NULL;
        buf_desc_ring->rx_free_desc++;
        /* Re-queue the descriptors and get rid of malformed packet */
        rx_queue(buf_desc_ring, rx_data_buf_ptr, idx);
        rx_data_buf_ptr = NULL;
        tr_warn("Malformed packet at RX.");
        return rx_data_buf_ptr;
      }

    /* Otherwise, copy the pointer to the data and empty the descriptor plus
     * allign it again to our rx desciptor ring */
    *data_length = enet_hal_get_bd_length(bdPtr + idx);
    /* Don't memcpy() here. No need */
    rx_data_buf_ptr = buf_desc_ring->rx_data_buf_ptr[idx];
    /* Clear pointer from descriptor */
    buf_desc_ring->rx_data_buf_ptr[idx] = NULL;
    /* Update index of free descriptors*/
    buf_desc_ring->rx_free_desc++;
    /* Allign the queue to buffer_descriptor again */
    rx_queue_alligned_to_buf_desc(buf_desc_ring, idx);


    return rx_data_buf_ptr;
}

/* This function is called whenever there is data at Ethernet. Prepares and pushes
 * the buffer to the Nanostack */
static void k64f_eth_receive(Ethernet_BufferDesc_Ring_t *buf_desc_ring, uint8_t idx){

    uint8_t *rx_data_buf_ptr = 0;
    int8_t retval = -1;
    uint16_t data_length=0;

    /* Prepare the push to Nanostack, if it ends up in error return
     * immidiately */
    rx_data_buf_ptr = Buf_to_Nanostack(buf_desc_ring, idx, &data_length);
    if(rx_data_buf_ptr==NULL){
        tr_warn("No data in , Buf_to_nanostack.");
        return;
    }

    /*Allign data according to the receive buffer allinment setting*/
    uint8_t *aligned_ptr = (uint8_t*)ENET_ALIGN((uint32_t)rx_data_buf_ptr, RX_BUF_ALIGNMENT);
    /*if (aligned_ptr[12] == 0x86 && aligned_ptr[13] == 0xDD) {
       tr_info("Data RX %u %s", data_length, trace_array(aligned_ptr, 24));
    }*/

    /* When alligned, Hand it over to Nanostack*/
    retval = arm_net_phy_rx(PHY_LAYER_PAYLOAD, aligned_ptr, data_length, 0xff, 0, eth_interface_id);
    /*if(retval>=0){
        tr_info("Data Pushed to Nanostack.");
    }
    else{
        tr_err("Nanostack rejected the push (%d).", retval);
    }*/
    (void) retval;

    MEM_FREE(rx_data_buf_ptr);
}

/* This function is called after the Transmission in order to perform buffer
 * descriptor cleanup */
static void tx_queue_reclaim(Ethernet_BufferDesc_Ring_t *buf_desc_ring)
{
    uint8_t index = 0;
    volatile enet_bd_struct_t *bdPtr = (enet_bd_struct_t*)buf_desc_ring->tx_buf_desc_start_addr;

    /* Traverse all descriptors, looking for the ones modified by the uDMA */
    index = buf_desc_ring->tx_buf_busy_index;
  //  tr_debug("Going to reclaim TX queue as per interrupt. Used BDs = %i", index);

    while (index != buf_desc_ring->tx_buf_free_index && !(bdPtr[index].control & kEnetTxBdReady)) {
        if (buf_desc_ring->txb_aligned[index]) {
            MEM_FREE(buf_desc_ring->txb_aligned[index]);
            buf_desc_ring->txb_aligned[index] = NULL;
        }
        else if (buf_desc_ring->tx_data_buf_ptr[index]) {
            MEM_FREE(buf_desc_ring->tx_data_buf_ptr[index]);
            buf_desc_ring->tx_data_buf_ptr[index] = NULL;
        }
        bdPtr[index].controlExtend2 &= ~TX_DESC_UPDATED_MASK;
        //tr_debug("Reclaimed BufferDescriptor[%i].", index);
        index = (index + 1) % ENET_TX_RING_LEN;
    }

    buf_desc_ring->tx_buf_busy_index = index;
    //tr_debug("buf_desc_ring->tx_buf_busy_index %i.", buf_desc_ring->tx_buf_busy_index);
    //tr_debug("buf_desc_ring->tx_buf_free_index %i.", buf_desc_ring->tx_buf_free_index);
}

/* This function tells how many tx buffer descriptors are free at the moment */
uint8_t k64f_tx_descriptors_ready(Ethernet_BufferDesc_Ring_t *buf_desc_ring)
{
    uint8_t fb;
    uint8_t idx, fidx;

    fidx = buf_desc_ring->tx_buf_free_index;
    idx = buf_desc_ring->tx_buf_busy_index;

    /* Determine number of free buffers */
    if (fidx >= idx)
        fb = (ENET_TX_RING_LEN - 1) - (fidx - idx);
    else
        fb = (ENET_TX_RING_LEN - 1) - ((fidx + ENET_TX_RING_LEN) - idx);

    return fb;
}

/* Update the buffer descriptor, which means spit out what you have in there */
void k64f_update_txbds(Ethernet_BufferDesc_Ring_t *buf_desc_ring, uint8_t idx, uint8_t *buffer, uint16_t length, bool isLast)
{
    volatile enet_bd_struct_t *bdPtr = (enet_bd_struct_t *)(buf_desc_ring->tx_buf_desc_start_addr + idx * enet_hal_get_bd_size());

    bdPtr->length = HTONS(length); /* Set data length*/
    bdPtr->buffer = (uint8_t *)HTONL((uint32_t)buffer); /* Set data buffer*/
    if (isLast)
        bdPtr->control |= kEnetTxBdLast;
    else
        bdPtr->control &= ~kEnetTxBdLast;
    bdPtr->controlExtend1 |= kEnetTxBdTxInterrupt;
    bdPtr->controlExtend2 &= ~TX_DESC_UPDATED_MASK; // descriptor not updated by DMA
    bdPtr->control |= kEnetTxBdTransmitCrc | kEnetTxBdReady;
}

/* This function is called by arm_eth_phy_tx() which is in turn called through
 * Nanostack. As evident from name, this sends out the transmit packets */
static int8_t k64f_eth_send(uint8_t *data_ptr, uint16_t data_len)
{

    uint8_t index = 0;
    uint8_t descriptor_num = 0;

    Ethernet_BufferDesc_Ring_t *buf_desc_ring =
            &buffer_descriptor_ring[BOARD_DEBUG_ENET_INSTANCE];

    /* Get the index of the Free TX descriptor */
    index = buf_desc_ring->tx_buf_free_index;

    /*Sanity Check*/
    if (data_len > ENET_ETH_MAX_FLEN) {
        tr_error("Packet size bigger than ENET_TXBuff_SIZE.");
        return -1;
    }

    /* Check if there are any buffer descriptors free */
   descriptor_num = k64f_tx_descriptors_ready(buf_desc_ring);

    if (descriptor_num < 1) {
        tr_error("TX buf descriptors full. Can't queue packet.");
        return -1;
    }

    uint8_t *buf_ptr = MEM_ALLOC(data_len + TX_BUF_ALIGNMENT);
    if (!buf_ptr) {
        tr_error("Alloc failed");
        return -1;
    }

    uint8_t *aligned_ptr = (uint8_t*) ENET_ALIGN((uint32_t )buf_ptr,
                                                 TX_BUF_ALIGNMENT);

    buf_desc_ring->tx_data_buf_ptr[index] = buf_ptr;

    memcpy(aligned_ptr, data_ptr, data_len);

    //if (aligned_ptr[12] == 0x86 && aligned_ptr[13] == 0xDD) {
    //    tr_debug("Data TX %u %s", data_len, trace_array(aligned_ptr, 24));
    //}

    k64f_update_txbds(buf_desc_ring, index, aligned_ptr, data_len, 1);
    buf_desc_ring->txb_aligned[index] = NULL;

    index = (index + 1) % ENET_TX_RING_LEN;
    buf_desc_ring->tx_buf_free_index = index;
    //tr_debug("Next TX buffer = %i", index);

    enet_hal_active_txbd(BOARD_DEBUG_ENET_INSTANCE_ADDR);

    return 0;
}

/* This function is calle by arm_eth_phy_k64f_address_write() which is in turn
 * called by nanostack in order to set up MAC address */
static void k64f_eth_set_address(uint8_t *address_ptr)
{
    enet_mac_config_t*mac_config =
            &(ethernet_mac_config[BOARD_DEBUG_ENET_INSTANCE]);

    /* When pointer to the MAC address is given. It could be 48-bit EUI generated
     * from Radio, like atmel RF or manually inserted. Preferred method.*/
    memcpy(mac_config->macAddr, address_ptr, kEnetMacAddrLen);
    enet_hal_set_mac_address(device_id, mac_config->macAddr);
}

/* This function sets the MAC address and its type for the Ethernet interface*/
/* Only type supported is 48 bits. If address_ptr is NULL, we will try to set the
 * address using mbed-drivers. */
static int8_t arm_eth_phy_k64f_address_write(phy_address_type_e address_type, uint8_t *address_ptr){

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

/* This function initializes the Ethernet Interface for frdm-k64f*/
static int8_t k64f_eth_initialize(){

    int8_t retval = -1;
    enet_dev_if_t *ethernet_iface_ptr;
    enet_mac_config_t *mac_config_ptr;
    enet_config_rx_accelerator_t rxAcceler;
    enet_config_tx_accelerator_t txAcceler;
    enet_rxbd_config_t rxbdCfg;
    enet_txbd_config_t txbdCfg;
    enet_phy_speed_t phy_speed;
    enet_phy_duplex_t phy_duplex;

    Ethernet_BufferDesc_Ring_t *buf_desc_ring = &buffer_descriptor_ring[BOARD_DEBUG_ENET_INSTANCE];

    ethernet_iface_ptr = &(ethernet_iface[BOARD_DEBUG_ENET_INSTANCE]);
    mac_config_ptr = &(ethernet_mac_config[BOARD_DEBUG_ENET_INSTANCE]);

    /*Initialize Ethernet Hardware on k64f board, so as to setup pins and clock*/
    k64f_init_eth_hardware();

    /* Setup the device ID*/
    ethernet_iface_ptr->deviceNumber = device_id;

    /* Setup MAC related configuration settings*/
    mac_config_ptr->rxBufferSize = ENET_ETH_MAX_FLEN;
    mac_config_ptr->txBufferSize = ENET_ETH_MAX_FLEN;
    mac_config_ptr->rxLargeBufferNumber = 0;
    mac_config_ptr->rxBdNumber = ENET_RX_RING_LEN;
    mac_config_ptr->txBdNumber = ENET_TX_RING_LEN;
    /* ! MAC address was alread set by the function k64f_eth_set_address ! */
    mac_config_ptr->rmiiCfgMode = kEnetCfgRmii;
    mac_config_ptr->speed = kEnetSpeed100M;
    mac_config_ptr->duplex = kEnetCfgFullDuplex;
    mac_config_ptr->macCtlConfigure = kEnetRxCrcFwdEnable | kEnetRxFlowControlEnable,
    mac_config_ptr->isTxAccelEnabled = true;
    mac_config_ptr->isRxAccelEnabled = true;
    mac_config_ptr->isStoreAndFwEnabled = false;
    rxAcceler.isIpcheckEnabled = false;
    rxAcceler.isMacCheckEnabled = true;
    rxAcceler.isPadRemoveEnabled = false;
    rxAcceler.isProtocolCheckEnabled = false;
    rxAcceler.isShift16Enabled = false;
    mac_config_ptr->rxAcceler = rxAcceler;
    txAcceler.isIpCheckEnabled = false;
    txAcceler.isProtocolCheckEnabled = false;
    txAcceler.isShift16Enabled = false;
    mac_config_ptr->txAcceler = txAcceler;
    mac_config_ptr->isVlanEnabled = false;
    mac_config_ptr->isPhyAutoDiscover = true;
    mac_config_ptr->miiClock = ENET_MII_CLOCK;

    ethernet_iface_ptr->macContextPtr = MEM_ALLOC (sizeof(enet_mac_context_t));
    if (!ethernet_iface_ptr->macContextPtr) {
        tr_debug("Memory Allocation Failed. mac_context_ptr");
        return -1;
    }
    memset(ethernet_iface_ptr->macContextPtr, 0, sizeof(enet_mac_context_t));
    ethernet_iface_ptr->maxFrameSize = ENET_ETH_MAX_FLEN;
    ethernet_iface_ptr->macCfgPtr = mac_config_ptr;

    /* Setup PHY related configuration settings*/
    ethernet_iface_ptr->phyCfgPtr = &enetPhyCfg[BOARD_DEBUG_ENET_INSTANCE];
    ethernet_iface_ptr->macApiPtr = &g_enetMacApi;
    ethernet_iface_ptr->phyApiPtr = (void *)&g_enetPhyApi;

    /* Setup RX buffer descriptors*/
    retval = k64f_eth_rx_buf_desc_setup(buf_desc_ring, &rxbdCfg);
    if (retval==-1){
        return retval;
    }

    /* Setup TX buffer descriptors*/
    retval = k64f_eth_tx_buf_desc_setup(buf_desc_ring, &txbdCfg);
    if (retval==-1){
        return retval;
    }

    /* Initialize MAC layer*/
    if(enet_mac_init(ethernet_iface_ptr, &rxbdCfg, &txbdCfg)==kStatus_ENET_Success){
        /* Initialize PHY layer*/
        if (ethernet_iface_ptr->macCfgPtr->isPhyAutoDiscover) {
            if (((enet_phy_api_t *)(ethernet_iface_ptr->phyApiPtr))->phy_auto_discover(ethernet_iface_ptr) != kStatus_PHY_Success){
              tr_debug("INIT_MAC. Auto discover fail.");
              return -1;
            }
        }
        if (((enet_phy_api_t *)(ethernet_iface_ptr->phyApiPtr))->phy_init(ethernet_iface_ptr) != kStatus_PHY_Success){
            tr_debug("INIT_MAC. PHY was not initialized.");
            return -1;
        }

        ethernet_iface_ptr->isInitialized = true;

        /* Get link information from PHY */
        phy_get_link_speed(ethernet_iface_ptr, &phy_speed);
        phy_get_link_duplex(ethernet_iface_ptr, &phy_duplex);
        BW_ENET_RCR_RMII_10T(ethernet_iface_ptr->deviceNumber, phy_speed == kEnetSpeed10M ? kEnetCfgSpeed10M : kEnetCfgSpeed100M);
        BW_ENET_TCR_FDEN(ethernet_iface_ptr->deviceNumber, phy_duplex == kEnetFullDuplex ? kEnetCfgFullDuplex : kEnetCfgHalfDuplex);

        /* Enable Ethernet module*/
        enet_hal_config_ethernet(BOARD_DEBUG_ENET_INSTANCE_ADDR, true, true);

        /* Active Receive buffer descriptor must be done after module enable*/
        enet_hal_active_rxbd(ethernet_iface_ptr->deviceNumber);

        tr_info("Ethernet Interface Initialized. Succes");

        /* All went well, now enable interrupts*/
        eth_enable_interrupts();
        return 0;
    }

    return -1;
}

/* Interrupt handling Section*/

/* Enables Ethernet interrupts */
void eth_enable_interrupts(void) {
    enet_hal_config_interrupt(BOARD_DEBUG_ENET_INSTANCE_ADDR, (kEnetTxFrameInterrupt | kEnetRxFrameInterrupt), true);
    INT_SYS_EnableIRQ(enet_irq_ids[BOARD_DEBUG_ENET_INSTANCE][enetIntMap[kEnetRxfInt]]);
    INT_SYS_EnableIRQ(enet_irq_ids[BOARD_DEBUG_ENET_INSTANCE][enetIntMap[kEnetTxfInt]]);
}

/* Disables Ethernet interrupts */
void eth_disable_interrupts(void) {
    INT_SYS_DisableIRQ(enet_irq_ids[BOARD_DEBUG_ENET_INSTANCE][enetIntMap[kEnetRxfInt]]);
    INT_SYS_DisableIRQ(enet_irq_ids[BOARD_DEBUG_ENET_INSTANCE][enetIntMap[kEnetTxfInt]]);
}

/* Interrupt Service Routine for RX IRQ*/
void enet_mac_rx_isr(void *enetIfPtr) {

    Ethernet_BufferDesc_Ring_t *buf_desc_ring = &buffer_descriptor_ring[BOARD_DEBUG_ENET_INSTANCE];
    volatile enet_bd_struct_t *bdPtr = (enet_bd_struct_t*)buf_desc_ring->rx_buf_desc_start_addr;
    static uint8_t idx = 0;

    /* Clear interrupt */
    enet_hal_clear_interrupt(((enet_dev_if_t *)enetIfPtr)->deviceNumber, kEnetRxFrameInterrupt);

    while ((bdPtr[idx].control & kEnetRxBdEmpty) == 0) {
        k64f_eth_receive(buf_desc_ring, idx);
        idx = (idx + 1) % ENET_RX_RING_LEN;
    }
}

/* Interrupt Service Routine for TX IRQ*/
void enet_mac_tx_isr(void *enetIfPtr) {

    Ethernet_BufferDesc_Ring_t *buf_desc_ring = &buffer_descriptor_ring[BOARD_DEBUG_ENET_INSTANCE];

    /*Clear interrupt*/
    enet_hal_clear_interrupt(((enet_dev_if_t *)enetIfPtr)->deviceNumber, kEnetTxFrameInterrupt);
    tx_queue_reclaim(buf_desc_ring);
}

/* Transmit IRQ Handler*/
void ENET_Transmit_IRQHandler(void) {
    NVIC_SetPendingIRQ(ENET_Receive_IRQn);
}

/* Receive IRQ Handler*/
void ENET_Receive_IRQHandler(void)
{
    enet_dev_if_t *ethernet_iface_ptr =
            &ethernet_iface[BOARD_DEBUG_ENET_INSTANCE];

    if (enet_hal_get_interrupt_status(ethernet_iface_ptr->deviceNumber,
                                      kEnetRxFrameInterrupt)) {
        enet_mac_rx_isr(enetIfHandle);
    }

    if (enet_hal_get_interrupt_status(ethernet_iface_ptr->deviceNumber,
                                      kEnetTxFrameInterrupt)) {
        enet_mac_tx_isr(enetIfHandle);
    }
}


