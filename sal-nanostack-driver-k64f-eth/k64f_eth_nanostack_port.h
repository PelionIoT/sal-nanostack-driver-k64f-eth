/*
 * Copyright (c) 2014 ARM. All rights reserved.
 */

#ifndef K64F_ETH_NANOSTACK_PORT_H_
#define K64F_ETH_NANOSTACK_PORT_H_

#define ENET_RX_RING_LEN              (8)
#define ENET_TX_RING_LEN              (2)
#define ENET_RX_LARGE_BUFFER_NUM      (0)
#define ENET_RX_BUFFER_ALIGNMENT      (16)
#define ENET_TX_BUFFER_ALIGNMENT      (16)
#define ENET_BD_ALIGNMENT             (16)
#define ENET_MII_CLOCK                (2500000L)
#define RX_BUF_ALIGNMENT              (16)
#define TX_BUF_ALIGNMENT              (8)
#define BOARD_DEBUG_ENET_INSTANCE     (0)
#define BOARD_DEBUG_ENET_INSTANCE_ADDR (ENET_BASE)
#define ENET_ETH_MAX_FLEN             (1518)

#if FSL_FEATURE_ENET_DMA_BIG_ENDIAN_ONLY
#define BD_SHORTSWAP(n)                     __REV16(n)
#define BD_LONGSWAP(n)                      __REV(n)
#else
#define BD_SHORTSWAP(n)                      (n)
#define BD_LONGSWAP(n)                       (n)
#endif




//#define ENET_RX_BUFFER_ALIGNMENT      (16)
//#define ENET_TX_BUFFER_ALIGNMENT      (16)
//#define ENET_BD_ALIGNMENT             (16)
//#define ENET_MII_CLOCK                (2500000L)
//#define RX_BUF_ALIGNMENT              (16)
//#define TX_BUF_ALIGNMENT              (8)
//#define BOARD_DEBUG_ENET_INSTANCE     (0)
//#define BOARD_DEBUG_ENET_INSTANCE_ADDR (ENET_BASE)
//#define ENET_RXBD_NUM                 (16)
//#define ENET_TXBD_NUM                 (8)
//#define ENET_RXBuff_SIZE              (kEnetMaxFrameSize)
//#define ENET_TXBuff_SIZE              (kEnetMaxFrameSize)
//#define ENET_ETH_MAX_FLEN             (kEnetMaxFrameSize)
//#define ENET_RX_LARGE_BUFFER_NUM      (0)

/*#define ENET_RX_BUFFER_ALIGNMENT      (8)
#define ENET_TX_BUFFER_ALIGNMENT      (8)
#define ENET_BD_ALIGNMENT             (8)
#define ENET_MII_CLOCK                (2500000L)
#define RX_BUF_ALIGNMENT              (8)
#define TX_BUF_ALIGNMENT              (8)
#define BOARD_DEBUG_ENET_INSTANCE     (0)
#define BOARD_DEBUG_ENET_INSTANCE_ADDR (ENET_BASE)
#define ENET_RXBD_NUM                 (16)
#define ENET_TXBD_NUM                 (2)
#define ENET_RXBuff_SIZE              (128)
#define ENET_TXBuff_SIZE              (kEnetMaxFrameSize)*/

#ifdef __cplusplus
extern "C" {
#endif

extern void arm_eth_phy_device_register(uint8_t *mac_ptr, void (*app_ipv6_init_cb)(uint8_t, int8_t));

#ifdef __cplusplus
}
#endif


#endif /* K64F_ETH_NANOSTACK_PORT_H_ */
