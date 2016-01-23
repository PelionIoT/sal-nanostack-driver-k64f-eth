/*
 * Copyright (c) 2014 ARM. All rights reserved.
 */

#include <string.h>
#include "platform/arm_hal_phy.h"
#include "platform/arm_hal_interrupt.h"
#include "ns_types.h"
#include "k64f_eth_nanostack_port.h"
#include "fsl_enet_driver.h"
#include "mbed-drivers/mbed_interface.h"
#include "fsl_interrupt_manager.h"
#include "ns_trace.h"
#include "ns_list.h"
#include "nsdynmemLIB.h"


/* Any Pre-processor Macros*/
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
static void arm_eth_phy_k64f_interface_state_control(phy_interface_state_e state, uint8_t x);
static int8_t arm_eth_phy_k64f_tx(uint8_t *data_ptr, uint16_t data_len, uint8_t tx_handle,data_protocol_e data_flow);
void (*driver_readiness_status_callback)(uint8_t, int8_t) = 0;
void eth_enable_interrupts(void);
void eth_disable_interrupts(void);
static int8_t k64f_eth_initialize(uint8_t *mac_ptr);
static int8_t k64f_eth_rx_buf_desc_setup();
static int8_t k64f_eth_tx_buf_desc_setup();

/*Internal Global Variables and declaration of data structures*/
extern void k64f_init_eth_hardware(void);
static int8_t eth_driver_enabled = -1;
static phy_device_driver_s eth_device_driver;
static Ethernet_BufferDesc_Ring_t buf_desc_ring;
static enet_dev_if_t *ethernet_iface_ptr;
static enet_mac_config_t ethernet_mac_config[HW_ENET_INSTANCE_COUNT];
static IRQn_Type enet_irq_ids[HW_ENET_INSTANCE_COUNT][FSL_FEATURE_ENET_INTERRUPT_COUNT];
static uint8_t enetIntMap[kEnetIntNum];
static uint32_t device_id = BOARD_DEBUG_ENET_INSTANCE_ADDR;


/* This function registers the ethernet driver to the Nanostack */
/* After registration to the stack, it initializez the driver itself*/
void arm_eth_phy_device_register(uint8_t *mac_ptr, void (*driver_status_cb)(uint8_t, uint8_t)){

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

        if (!eth_driver_enabled){
            tr_error("Ethernet Driver failed to register with Stack.");
            driver_readiness_status_callback(0, eth_driver_enabled);
        }

        else{
           eth_driver_enabled = k64f_eth_initialize(mac_ptr);
           if(eth_driver_enabled==-1){
               tr_error("Failed to Initialize Ethernet Driver.");
               driver_readiness_status_callback(0, eth_driver_enabled);
           }
        }
    }
}

static int8_t k64f_eth_initialize(uint8_t *mac_ptr){

    int8_t retval = -1;
    ethernet_iface_ptr = MEM_ALLOC(sizeof(enet_dev_if_t));
    enet_rxbd_config_t *rxbdCfg = MEM_ALLOC(sizeof(enet_rxbd_config_t));
    enet_txbd_config_t *txbdCfg = MEM_ALLOC(sizeof(enet_txbd_config_t));

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
    ethernet_iface_ptr->phyCfgPtr->isLoopEnabled = false;
    ethernet_iface_ptr->phyCfgPtr->phyAddr = 0;

    /* Setup RX buffer descriptors*/
    retval = k64f_eth_rx_buf_desc_setup(rxbdCfg);
    if (retval==-1){
        return retval;
    }

    /* Setup TX buffer descriptors*/
    retval = k64f_eth_tx_buf_desc_setup(txbdCfg);
    if (retval==-1){
        return retval;
    }

    if(enet_mac_init(ethernet_iface_ptr, rxbdCfg, txbdCfg)==kStatus_ENET_Success){
        tr_info("Ethernet Interface Initialized. Succes");
        return 0;
    }

    return -1;
}

static int8_t k64f_eth_rx_buf_desc_setup(enet_rxbd_config_t *rxbdCfg){

    uint8_t *rxBdPtr;
    uint32_t rxBufferSizeAligned;

    /* Allocate memory to RX buffer descriptors */
    rxBdPtr = MEM_ALLOC(enet_hal_get_bd_size()*ethernet_iface_ptr->macCfgPtr->rxBdNumber+ENET_BD_ALIGNMENT);

    if(!rxBdPtr){
        tr_error("Could not allocate memory for rx buffer descriptors.");
        return -1;
      }

    buf_desc_ring.rx_buf_desc_start_addr = (uint8_t *)ENET_ALIGN((uint32_t)rxBdPtr, ENET_BD_ALIGNMENT);
    buf_desc_ring.rx_free_desc = ethernet_iface_ptr->macCfgPtr->rxBdNumber;
    buf_desc_ring.rx_fill_index = 0;

    rxBufferSizeAligned = ENET_ALIGN(ethernet_iface_ptr->macCfgPtr->rxBdNumber, ENET_RX_BUFFER_ALIGNMENT);
    ethernet_iface_ptr->macContextPtr->rxBufferSizeAligned = rxBufferSizeAligned;
    rxbdCfg->rxBdPtrAlign = buf_desc_ring.rx_buf_desc_start_addr ;
    rxbdCfg->rxBdNum = ethernet_iface_ptr->macCfgPtr->rxBdNumber;
    rxbdCfg->rxBufferNum = ethernet_iface_ptr->macCfgPtr->rxBdNumber;

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

    buf_desc_ring.tx_buf_desc_start_addr = (uint8_t *)ENET_ALIGN((uint32_t)txBdPtr, ENET_BD_ALIGNMENT);
    buf_desc_ring.tx_consume_index = buf_desc_ring.tx_produce_index = 0;

    txbdCfg->txBdPtrAlign = buf_desc_ring.tx_buf_desc_start_addr;
    txbdCfg->txBufferNum = ethernet_iface_ptr->macCfgPtr->txBdNumber;
    txbdCfg->txBufferSizeAlign = ENET_ALIGN(ethernet_iface_ptr->maxFrameSize, ENET_TX_BUFFER_ALIGNMENT);

    return 0;
}

static int8_t arm_eth_phy_k64f_tx(uint8_t *data_ptr, uint16_t data_len, uint8_t tx_handle,data_protocol_e data_flow){

    int retval = -1;
    eth_disable_interrupts();

    if(data_len){
      //  k64f_eth_send();
        retval = 0;
    }

    eth_enable_interrupts();
    return retval;
}

static void arm_eth_phy_k64f_interface_state_control(phy_interface_state_e state, uint8_t not_required){

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
        memcpy(&ethernet_mac_config->macAddr, address_ptr, kEnetMacAddrLen);
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


