/*
 * Copyright (c) 2014 ARM. All rights reserved.
 */

#include <string.h>
#include "platform/arm_hal_phy.h"
#include "platform/arm_hal_interrupt.h"
#include "ns_types.h"
#include "k64f_eth_nanostack_port.h"
#include "fsl_enet_driver.h"
#include "fsl_phy_driver.h"
#include "mbed-drivers/mbed_interface.h"
#include "fsl_interrupt_manager.h"
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
static int8_t k64f_eth_initialize(uint8_t *mac_ptr);
static int8_t k64f_eth_rx_buf_desc_setup();
static int8_t k64f_eth_tx_buf_desc_setup();
static int8_t rx_queue_alligned_to_buf_desc(int8_t index);
static void rx_queue(uint8_t *data_buf_ptr, uint8_t index);
static int8_t k64f_eth_send(uint8_t *data_ptr, uint16_t data_len);
extern void k64f_init_eth_hardware(void);

/*Internal Global Variables and declaration of data structures*/
static int8_t eth_driver_enabled = -1;
static phy_device_driver_s eth_device_driver;
static Ethernet_BufferDesc_Ring_t *buf_desc_ring;
static enet_dev_if_t *ethernet_iface_ptr;
static enet_mac_config_t ethernet_mac_config[HW_ENET_INSTANCE_COUNT];
static enet_phy_config_t enetPhyCfg[HW_ENET_INSTANCE_COUNT] =
{
  {0, false}
};
static IRQn_Type enet_irq_ids[HW_ENET_INSTANCE_COUNT][FSL_FEATURE_ENET_INTERRUPT_COUNT];
static uint8_t enetIntMap[kEnetIntNum];
static uint32_t device_id = BOARD_DEBUG_ENET_INSTANCE_ADDR;
static uint8_t isAutoIndex = 0;


/* This function registers the ethernet driver to the Nanostack */
/* After registration to the stack, it initializez the driver itself*/
void arm_eth_phy_device_register(uint8_t *mac_ptr, void (*driver_status_cb)(uint8_t, int8_t)){

    if (eth_driver_enabled==-1){

        eth_device_driver.PHY_MAC = mac_ptr;
        eth_device_driver.address_write = &arm_eth_phy_k64f_address_write;
        eth_device_driver.driver_description = "ETH";
        eth_device_driver.link_type = PHY_LINK_ETHERNET_TYPE;
        eth_device_driver.phy_MTU = 0;
        eth_device_driver.phy_header_length = 0;
        eth_device_driver.phy_tail_length = 0;
        eth_device_driver.state_control = &arm_eth_phy_k64f_interface_state_control;
        eth_device_driver.tx = &arm_eth_phy_k64f_tx;
        eth_driver_enabled = arm_net_phy_register(&eth_device_driver);
        driver_readiness_status_callback = driver_status_cb;

        if (eth_driver_enabled < 0){
            tr_error("Ethernet Driver failed to register with Stack. RetCode=%i", eth_driver_enabled);
            driver_readiness_status_callback(0, eth_driver_enabled);
        }

        else{
           eth_driver_enabled = k64f_eth_initialize(mac_ptr);
           if(eth_driver_enabled==-1){
               tr_error("Failed to Initialize Ethernet Driver.");
               driver_readiness_status_callback(0, eth_driver_enabled);
           }
           else{
               tr_info("Ethernet Driver Initialized.");
               driver_readiness_status_callback(1, eth_driver_enabled);
           }
        }
    }
}

static int8_t k64f_eth_initialize(uint8_t *mac_ptr){

    int8_t retval = -1;
    ethernet_iface_ptr = MEM_ALLOC(sizeof(enet_dev_if_t));
    if (!ethernet_iface_ptr) {
        tr_debug("Memory Allocation Failed. ethernet_iface_ptr");
        return -1;
     }
    buf_desc_ring = MEM_ALLOC(sizeof(Ethernet_BufferDesc_Ring_t));
    if (!buf_desc_ring) {
        tr_debug("Memory Allocation Failed. buf_desc_ring");
        return -1;
     }
    ethernet_iface_ptr->macContextPtr = MEM_ALLOC (sizeof(enet_mac_context_t));
    if (!ethernet_iface_ptr->macContextPtr) {
        tr_debug("Memory Allocation Failed. mac_context_ptr");
        return -1;
    }

    enet_rxbd_config_t rxbdCfg;
    enet_txbd_config_t txbdCfg;

    /*Initialize Ethernet Hardware on k64f board, so as to setup pins and clock*/
    k64f_init_eth_hardware();
    ethernet_iface_ptr->deviceNumber = device_id;

    /* Setup Ethernet MAC related settings and configuration*/
    k64f_eth_set_address(mac_ptr);
    ethernet_mac_config->speed = kEnetCfgSpeed100M;
    ethernet_mac_config->miiClock = ENET_MII_CLOCK;
    ethernet_mac_config->rxBdNumber = ENET_RXBD_NUM;
    ethernet_mac_config->rxBufferSize = kEnetMaxFrameSize;
    ethernet_mac_config->txBdNumber = ENET_TXBD_NUM;
    ethernet_mac_config->duplex = kEnetCfgFullDuplex;
    ethernet_mac_config->rmiiCfgMode = kEnetCfgRmii;
    ethernet_mac_config->isPhyAutoDiscover = true;
    ethernet_mac_config->isRxAccelEnabled = true;
    ethernet_mac_config->isTxAccelEnabled = true;
    ethernet_mac_config->isStoreAndFwEnabled = false;
    ethernet_mac_config->macCtlConfigure = kEnetRxCrcFwdEnable | kEnetRxFlowControlEnable;
    ethernet_mac_config->isVlanEnabled = true;
    ethernet_mac_config->rxAcceler.isIpcheckEnabled = false;
    ethernet_mac_config->rxAcceler.isMacCheckEnabled = false;
    ethernet_mac_config->rxAcceler.isPadRemoveEnabled = true;
    ethernet_mac_config->rxAcceler.isProtocolCheckEnabled = false;
    ethernet_mac_config->rxAcceler.isShift16Enabled = true;
    ethernet_mac_config->txAcceler.isIpCheckEnabled = false;
    ethernet_mac_config->txAcceler.isProtocolCheckEnabled = false;
    ethernet_mac_config->txAcceler.isShift16Enabled = true;

    /* Setup PHY related configuration settings*/
    ethernet_iface_ptr->phyCfgPtr = enetPhyCfg;
    ethernet_iface_ptr->macApiPtr = &g_enetMacApi;
    ethernet_iface_ptr->phyApiPtr = (void *)&g_enetPhyApi;

    /* Add MAC configuration to Ethernet_iface_ptr*/
    ethernet_iface_ptr->macCfgPtr = ethernet_mac_config;

    /* Setup RX buffer descriptors*/
    retval = k64f_eth_rx_buf_desc_setup(&rxbdCfg);
    if (retval==-1){
        return retval;
    }

    /* Setup TX buffer descriptors*/
    retval = k64f_eth_tx_buf_desc_setup(&txbdCfg);
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

        tr_info("Ethernet Interface Initialized. Succes");
        return 0;
    }

    return -1;
}

static int8_t k64f_eth_rx_buf_desc_setup(enet_rxbd_config_t *rxbdCfg){

    uint8_t *rxBdPtr;
    uint32_t rxBufferSizeAligned;

    /* Allocate memory to RX buffer descriptors - 29x16+8 Bytes*/
    rxBdPtr = MEM_ALLOC(sizeof(enet_bd_struct_t)*ethernet_iface_ptr->macCfgPtr->rxBdNumber+ENET_BD_ALIGNMENT);

    if(!rxBdPtr){
        tr_error("Could not allocate memory for rx buffer descriptors.");
        return -1;
      }

    buf_desc_ring->rx_buf_desc_start_addr = (uint8_t *)ENET_ALIGN((uint32_t)rxBdPtr, ENET_BD_ALIGNMENT);
    buf_desc_ring->rx_free_desc = ethernet_iface_ptr->macCfgPtr->rxBdNumber;
    buf_desc_ring->rx_fill_index = 0;

    rxBufferSizeAligned = ENET_ALIGN(ethernet_iface_ptr->macCfgPtr->rxBdNumber, ENET_RX_BUFFER_ALIGNMENT);
    ethernet_iface_ptr->macContextPtr->rxBufferSizeAligned = rxBufferSizeAligned;
    rxbdCfg->rxBdPtrAlign = buf_desc_ring->rx_buf_desc_start_addr ;
    rxbdCfg->rxBdNum = ethernet_iface_ptr->macCfgPtr->rxBdNumber;
    rxbdCfg->rxBufferNum = ethernet_iface_ptr->macCfgPtr->rxBdNumber;

    rx_queue_alligned_to_buf_desc(AUTO_INDEX);

    return 0;
}


static int8_t k64f_eth_tx_buf_desc_setup(enet_txbd_config_t *txbdCfg){

    uint8_t *txBdPtr;


    /* Allocate memory to TX buffer descriptors */
    txBdPtr = MEM_ALLOC(enet_hal_get_bd_size() * ethernet_iface_ptr->macCfgPtr->txBdNumber + ENET_BD_ALIGNMENT);
    if(!txBdPtr){
        tr_error("Could not allocate memory for tx buffer descriptors.");
        return -1;
    }

    buf_desc_ring->tx_buf_desc_start_addr = (uint8_t *)ENET_ALIGN((uint32_t)txBdPtr, ENET_BD_ALIGNMENT);
    buf_desc_ring->tx_buf_des_used = buf_desc_ring->tx_buf_des_unused = 0;

    txbdCfg->txBdPtrAlign = buf_desc_ring->tx_buf_desc_start_addr;
    txbdCfg->txBufferNum = ethernet_iface_ptr->macCfgPtr->txBdNumber;
    txbdCfg->txBufferSizeAlign = ENET_ALIGN(ethernet_iface_ptr->maxFrameSize, ENET_TX_BUFFER_ALIGNMENT);

    /* Initialize TX descriptor ring */
    enet_hal_init_txbds(buf_desc_ring->tx_buf_desc_start_addr + enet_hal_get_bd_size() * (ENET_TXBD_NUM- 1), 1);

    return 0;
}

static int8_t arm_eth_phy_k64f_tx(uint8_t *data_ptr, uint16_t data_len, uint8_t tx_handle,data_protocol_e data_flow){

    int retval = -1;

    if(data_len){
        retval = k64f_eth_send(data_ptr, data_len);
    }

    (void)data_flow;
    (void)tx_handle;


    return retval;
}

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

static int8_t rx_queue_alligned_to_buf_desc(int8_t index){

    uint8_t idx = 0;

    /* Get next free descriptor index */
    if (index == AUTO_INDEX){
        idx = buf_desc_ring->rx_fill_index;
        isAutoIndex = 1;
    }
    else{
        idx = (uint8_t)index;
    }

    while(buf_desc_ring->rx_free_desc > 0){
        /* Allocate memory for received data buffer and allign it to the
         * corresponding buffer descriptor*/
        buf_desc_ring->rx_data_buf_ptr[idx] = MEM_ALLOC(sizeof(ethernet_iface_ptr->macCfgPtr->rxBufferSize + RX_BUF_ALIGNMENT));
        if (!buf_desc_ring->rx_data_buf_ptr[idx]){
            tr_error("Could not allocate memory. rx_queue_alligned_to_buf_desc.");
            return -1;
        }
        buf_desc_ring->rx_data_buf_ptr[idx] = (uint8_t*)ENET_ALIGN((uint32_t)buf_desc_ring->rx_data_buf_ptr[idx], RX_BUF_ALIGNMENT);
        rx_queue(buf_desc_ring->rx_data_buf_ptr[idx], idx);
    }

    return 0;
}

static void rx_queue(uint8_t *data_buf_ptr, uint8_t index){

    enet_bd_struct_t *start = (enet_bd_struct_t *)buf_desc_ring->rx_buf_desc_start_addr;
    /* Setup descriptor and clear statuses */
    enet_hal_init_rxbds(start + index, data_buf_ptr, index == ENET_RXBD_NUM - 1);
    buf_desc_ring->rx_data_buf_ptr[index] = data_buf_ptr;
    /* Wrap at end of descriptor list */
    index = (index + 1) % ENET_RXBD_NUM;
    /* Update index of free descriptors*/
    buf_desc_ring->rx_free_desc -= 1;
    if (isAutoIndex){
        buf_desc_ring->rx_fill_index = index;
        isAutoIndex = 0;
    }
    /* Activate current buffer descriptor */
    enet_hal_active_rxbd(BOARD_DEBUG_ENET_INSTANCE_ADDR);
    tr_debug("Free RX buffer descriptors no. is = %i", buf_desc_ring->rx_free_desc);
}

static uint8_t *Buf_to_Nanostack(uint8_t idx, uint16_t *data_length){

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
        rx_queue(rx_data_buf_ptr, idx);
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
    if(rx_queue_alligned_to_buf_desc(idx)==-1){
        return NULL;
    }

    return rx_data_buf_ptr;
}

static void k64f_eth_receive(uint8_t idx){

    uint8_t *rx_data_buf_ptr = 0;
    int8_t retval = -1;
    uint16_t data_length=0;

    /* Prepare the push to Nanostack, if it ends up in error return
     * immidiately */
    rx_data_buf_ptr = Buf_to_Nanostack(idx, &data_length);
    if(rx_data_buf_ptr==NULL){
        return;
    }

    /* Otherwise Hand it over to Nanostack*/
    retval = arm_net_phy_rx(PHY_LAYER_PAYLOAD, rx_data_buf_ptr, data_length, 0xff, 0, eth_driver_enabled);
    if(retval>=0){
        tr_info("Data Pushed to Nanostack.");
    }
    else{
        tr_err("Nanostack rejected the push.");
    }

    MEM_FREE(rx_data_buf_ptr);

}

static void tx_queue_reclaim(volatile enet_bd_struct_t *bdPtr)
{
    uint8_t index = 0;

    /* Traverse all descriptors, looking for the ones modified by the uDMA */
    index = buf_desc_ring->tx_buf_des_used;
    while (index != buf_desc_ring->tx_buf_des_unused && !(bdPtr[index].control & kEnetTxBdReady)) {
        if (buf_desc_ring->txb_aligned[index]) {
            MEM_FREE(buf_desc_ring->txb_aligned[index]);
            buf_desc_ring->txb_aligned[index] = NULL;
        }
        else if (buf_desc_ring->tx_data_buf_ptr[index]) {
            MEM_FREE(buf_desc_ring->tx_data_buf_ptr[index]);
            buf_desc_ring->tx_data_buf_ptr[index] = NULL;
        }
        bdPtr[index].controlExtend2 &= ~TX_DESC_UPDATED_MASK;
        index = (index + 1) % ENET_TXBD_NUM;
    }
    buf_desc_ring->tx_buf_des_used = index;
}

uint8_t k64f_is_tx_ready()
{
    uint8_t fb;
    uint8_t idx, cidx;

    cidx = buf_desc_ring->tx_buf_des_used;
    idx = buf_desc_ring->tx_buf_des_unused;

    /* Determine number of free buffers */
    if (idx == cidx)
        fb = ENET_TXBD_NUM;
    else if (cidx > idx)
        fb = (ENET_TXBD_NUM - 1) - ((idx + ENET_TXBD_NUM) - cidx);
    else
        fb = (ENET_TXBD_NUM - 1) - (cidx - idx);

    return fb;
}

void k64f_update_txbds(uint8_t idx, uint8_t *buffer, uint16_t length, bool isLast)
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

static int8_t k64f_eth_send(uint8_t *data_ptr, uint16_t data_len){

    uint8_t *tx_buf = 0;
    uint8_t index = 0;

    /* Check which one of the TX buffer descriptor is free */
    index = buf_desc_ring->tx_buf_des_unused;

    if(data_len>ENET_TXBuff_SIZE){
        tr_error("Packet size bigger than ENET_TXBuff_SIZE.");
        return -1;
    }

    /* Allocate memory for the data to be sent out */
    tx_buf = MEM_ALLOC(data_len);
    if(!tx_buf){
        tr_error("Out of memory. tx_buf allocation.");
        return -1;
    }

    /* Check data alignment*/

    /* Tell that to the tx buffer descriptor*/
    buf_desc_ring->txb_aligned[index] = tx_buf;

    /* Copy the data buffer in the local buffer*/
    memcpy(tx_buf, data_ptr, data_len);

    /* Check if there are any buffer descriptors free */
    uint8_t descriptor_num = k64f_is_tx_ready();

    /* Iterate over the descriptors and send packet when the first
     * opportunity arrives */
    while (descriptor_num > 0) {

        descriptor_num--;

        if (tx_buf != NULL) {
            if (descriptor_num==0)
                k64f_update_txbds(index, tx_buf, data_len, 1);
            else
                k64f_update_txbds(index, tx_buf, data_len, 0);
            buf_desc_ring->tx_data_buf_ptr[index] = NULL;
            tr_debug("Packet Sent. Data length = %i, from BD = %i", data_len, index);
        }
    }

    index = (index + 1) % ENET_TXBD_NUM;
    buf_desc_ring->tx_buf_des_used = index;
    enet_hal_active_txbd(BOARD_DEBUG_ENET_INSTANCE_ADDR);

    return 0;
}

static void k64f_eth_set_address(uint8_t *address_ptr){

    /* When pointer to the MAC address is not given*/
    if (address_ptr==NULL){

        /* set MAC hardware address based upon UUID */
        #if (MBED_MAC_ADDRESS_SUM != MBED_MAC_ADDR_INTERFACE)
        ethernet_mac_config->macAddr[0] = MBED_MAC_ADDR_0;
        ethernet_mac_config->macAddr[1] = MBED_MAC_ADDR_1;
        ethernet_mac_config->macAddr[2] = MBED_MAC_ADDR_2;
        ethernet_mac_config->macAddr[3] = MBED_MAC_ADDR_3;
        ethernet_mac_config->macAddr[4] = MBED_MAC_ADDR_4;
        ethernet_mac_config->macAddr[5] = MBED_MAC_ADDR_5;
        #else
        /* set a semi unique MAC address. Not a preferred method */
        mbed_mac_address((char *)ethernet_mac_config->macAddr);
        #endif
    }
    /* When pointer to the MAC address is given. It could be 48-bit EUI generated
     * from Radio, like atmel RF or manually inserted. Preferred method.*/
    else{
        memcpy(ethernet_mac_config->macAddr, address_ptr, kEnetMacAddrLen);
    }

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

/* Interrupt handling*/

void eth_enable_interrupts(void) {
    enet_hal_config_interrupt(BOARD_DEBUG_ENET_INSTANCE_ADDR, (kEnetTxFrameInterrupt | kEnetRxFrameInterrupt), true);
    INT_SYS_EnableIRQ(enet_irq_ids[BOARD_DEBUG_ENET_INSTANCE][enetIntMap[kEnetRxfInt]]);
    INT_SYS_EnableIRQ(enet_irq_ids[BOARD_DEBUG_ENET_INSTANCE][enetIntMap[kEnetTxfInt]]);
}

void eth_disable_interrupts(void) {
    INT_SYS_DisableIRQ(enet_irq_ids[BOARD_DEBUG_ENET_INSTANCE][enetIntMap[kEnetRxfInt]]);
    INT_SYS_DisableIRQ(enet_irq_ids[BOARD_DEBUG_ENET_INSTANCE][enetIntMap[kEnetTxfInt]]);
}

void enet_mac_rx_isr(void *enetIfPtr) {

    volatile enet_bd_struct_t *bdPtr = (enet_bd_struct_t*)buf_desc_ring->rx_buf_desc_start_addr;
    static uint8_t idx = 0;
    /* Clear interrupt */
    enet_hal_clear_interrupt(((enet_dev_if_t *)enetIfPtr)->deviceNumber, kEnetRxFrameInterrupt);

    while ((bdPtr[idx].control & kEnetRxBdEmpty) == 0) {
        k64f_eth_receive(idx);
        idx = (idx + 1) % ENET_RXBD_NUM;
    }
}

void enet_mac_tx_isr(void *enetIfPtr) {

    volatile enet_bd_struct_t *bdPtr = (enet_bd_struct_t*)buf_desc_ring->tx_buf_desc_start_addr;

    /*Clear interrupt*/
    enet_hal_clear_interrupt(((enet_dev_if_t *)enetIfPtr)->deviceNumber, kEnetTxFrameInterrupt);
    tx_queue_reclaim(bdPtr);
}

void ENET_Transmit_IRQHandler(void) {
    NVIC_SetPendingIRQ(ENET_Receive_IRQn);
}

void ENET_Receive_IRQHandler(void) {

    if (enet_hal_get_interrupt_status(ethernet_iface_ptr->deviceNumber, kEnetRxFrameInterrupt))
        enet_mac_rx_isr(enetIfHandle);
    if (enet_hal_get_interrupt_status(ethernet_iface_ptr->deviceNumber, kEnetTxFrameInterrupt))
        enet_mac_tx_isr(enetIfHandle);
}


