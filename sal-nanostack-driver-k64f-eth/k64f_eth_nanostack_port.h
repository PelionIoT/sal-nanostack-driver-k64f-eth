/*
 * Copyright (c) 2014 ARM. All rights reserved.
 */

#ifndef K64F_ETH_NANOSTACK_PORT_H_
#define K64F_ETH_NANOSTACK_PORT_H_

#define ENET_RX_BUFFER_ALIGNMENT      (16)
#define ENET_TX_BUFFER_ALIGNMENT      (16)
#define ENET_BD_ALIGNMENT             (16)
#define ENET_MII_CLOCK                (2500000L)
#define RX_BUF_ALIGNMENT              (16)
#define TX_BUF_ALIGNMENT              (8)
#define BOARD_DEBUG_ENET_INSTANCE     (0)
#define BOARD_DEBUG_ENET_INSTANCE_ADDR (ENET_BASE)
#define ENET_RXBD_NUM                 (16)
#define ENET_TXBD_NUM                 (8)
#define ENET_RXBuff_SIZE              (kEnetMaxFrameSize)
#define ENET_TXBuff_SIZE              (kEnetMaxFrameSize)


extern void arm_eth_phy_device_register(uint8_t *mac_ptr, void (*app_ipv6_init_cb)(uint8_t, int8_t));

typedef struct Ethernet_BufferDesc_Ring_t{
    volatile uint8_t rx_free_desc; /* No. of free rx buffer descriptors*/
    uint8_t *rx_buf_desc_start_addr; /* Pointer to RX buffer descriptor start address*/
    uint8_t *tx_buf_desc_start_addr; /* Pointer to TX buffer descriptor start address*/
    uint8_t rx_fill_index; /* tells how much RX buffer descriptor ring is filled already*/
    uint8_t tx_buf_des_used;
    uint8_t tx_buf_des_unused;
    uint8_t *rx_data_buf_ptr[ENET_RXBD_NUM];
    uint8_t *tx_data_buf_ptr[ENET_TXBD_NUM];
    void *txb_aligned[ENET_TXBD_NUM]; /**< TX aligned buffers (if needed) */
}Ethernet_BufferDesc_Ring_t;

#endif /* K64F_ETH_NANOSTACK_PORT_H_ */
