# FRDM-K64F border router

This document describes how to configure, compile, and run a FRDM-K64F 6LoWPAN border router application on a [FRDM-K64F development board](https://www.mbed.com/en/platform/hardware/boards/nxp/frdm_k64f/). The border router can be configured to work in 6loWPAN ND or Thread mode. 

<span class="notes">**Note:** this repository is an updated version of the [public one](https://github.com/ARMmbed/k64f-border-router) which only supports 6LoWPAN ND.</span>

## Introduction

Border router is a network gateway between a wireless 6LoWPAN mesh network and a backhaul network. It controls and relays traffic between the two networks. In a typical setup, a 6LoWPAN border router is connected to another router in the backhaul network (over Ethernet or a serial line) which in turn forwards traffic to/from the internet or a private company LAN, for instance.

![](images/frdm_k64f_br_role.png)

## Software components

The FRDM-K64F border router application consists of four software components as shown in the image below:

![](images/frdm_k64f_br_components.png)

* [Nanostack Border Router](https://github.com/ARMmbed/nanostack-border-router-private) provides the core IPv6 gateway logic and the mesh network functionality. It can be configured to the 6LoWPAN ND or Thread mode.
* [RF driver](https://github.com/ARMmbed/atmel-rf-driver) for the Atmel AT86RF2xxx wireless 6LoWPAN shields.
* [Ethernet driver](https://github.com/ARMmbed/sal-nanostack-driver-k64f-eth) is the Ethernet driver for the FRDM-K64F development board.
* [SLIP driver](https://github.com/ARMmbed/sal-stack-nanostack-slip) is a generic Serial Line Internet Protocol version 6 (SLIPv6) driver for mbedOS boards.

## Required hardware

* Two FRDM-K64F development boards, one for the border router application and another one for the client application [the 6LoWPAN mbed client application](https://github.com/ARMmbed/mbed-os-example-client).
* Two mbed 6LoWPAN shields (AT86RF212B/[AT86RF233](http://uk.rs-online.com/web/p/radio-frequency-development-kits/9054107/)) for wireless 6LoWPAN mesh connectivity.
  * Alternatively, you can use [NXP MCR20A](http://www.nxp.com/products/software-and-tools/hardware-development-tools/freedom-development-boards/freedom-development-board-for-mcr20a-wireless-transceiver:FRDM-CR20A) shields.
  * See [Switching the RF shield](#switching-the-rf-shield).
* Two micro-USB cables to connect the development boards to a PC for debugging and power.
* An Ethernet cable to connect the development board to a backhaul network.

![](images/frdm_k64f_board_plus_shield.png)

## Required software

* [mbed-cli](https://github.com/ARMmbed/mbed-cli#installing-mbed-cli) command line interface.
* A compiler. Use one of the following:
    * [GCC ARM Embedded](https://launchpad.net/gcc-arm-embedded).
    * [ARM Compiler](https://developer.arm.com/products/software-development-tools/compilers/arm-compiler-5/downloads). (Requires license)
* [mbed account](https://www.mbed.com).
* A GitHub account.

Check out the [ARM mbed OS User Guide](https://docs.mbed.com/docs/mbed-os-handbook/en/5.3/) for instructions on how to develop applications on ARM mbed OS.

## Optional software

* [PuTTY](http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html) for serial terminal emulation, see [serial connection settings](#serial-connection-settings).
* [Wireshark](https://www.wireshark.org/) for packet inspection and network debugging.
* [mbed Windows serial port driver](https://developer.mbed.org/handbook/Windows-serial-configuration), a serial driver for Windows to enable serial connections.

## Configuration

To configure the FRDM-K64F border router, you need to make changes in the application configuration file `.json`. There are example configurations in the [configs](./configs) directory. The `mbed_app.json` file in the root directory is the default configuration, which is for 6LoWPAN ND. The example configurations also contain helpful parameter descriptions. For more information on the backhaul and 6LoWPAN ND configuration options, refer to the [Nanostack Border Router](https://github.com/ARMmbed/nanostack-border-router-private) documentation.

#### 6LoWPAN ND configuration

The essential configuration parameters are described in the following table:

Parameter|Description
---------|-----------
`network-mode`|Defines the 6LoWPAN mode, which can be `ND_WITH_MLE` or `ND_WITHOUT_MLE`.
`security-mode`|Can be `PSK`, `PANA` or `NONE`. 
`pana-mode`|Defines the PANA security mode (if PANA selected), which can be `ECC`, `ECC+PSK` or `PSK` (the default). 
`psk-key`|Is used when the `PSK` security is selected.
`tls-psk-key`|Is used when `PANA` is selected.
`ra-router-lifetime`|Defines the router advertisement interval in seconds (default 1024 if left out). 
`beacon-protocol-id`|Is used to identify beacons, this should not be changed (default 4 if left out).
`nanostack.configuration`|Is needed when building the 6LoWPAN ND border router from the nanostack sources.

The feature `LOWPAN_BORDER_ROUTER` is the nanostack library, which implements the 6LoWPAN ND border router networking stack.

More information on 6LoWPAN and the configuration parameters can be found from here [6LoWPAN overview] (https://docs.mbed.com/docs/arm-ipv66lowpan-stack/en/latest/quick_start_intro/index.html)

```
"config": {
    "network-mode": "ND_WITH_MLE",
        "security-mode": "PSK",
        "psk-key-id": 1,
        "psk-key": "{0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf}",
        "pana-mode": "",
        "tls-psk-key": "{0xcf, 0xce, 0xcd, 0xcc, 0xcb, 0xca, 0xc9, 0xc8, 0xc7, 0xc6, 0xc5, 0xc4, 0xc3, 0xc2, 0xc1, 0xc0}",
        "tls-psk-key-id": 1,
        "pan-id": "0x0700",
        "network-id": "network000000000",
        "beacon-protocol-id": 4,     
        "rf-channel": 12,
        "rf-channel-page": 0,
        "rf-channel-mask": "0x07fff800",
        "short-mac-address": "0xface",    
        "ra-router-lifetime": 1024,
        "rpl-instance-id": 1,
        "rpl-idoublings": 9,
        "rpl-imin": 12,
        "rpl-k": 10,
        "rpl-max-rank-inc": 2048,
        "rpl-min-hop-rank-inc": 128,
        "rpl-default-lifetime": 64,
        "rpl-lifetime-unit": 60,
        "rpl-pcs": 1,
        "rpl-ocp": 1
 },
    "target_overrides": {
        "*": {
            "target.features_add": ["NANOSTACK", "LOWPAN_BORDER_ROUTER", "COMMON_PAL"],
            "mbed-trace.enable": 1,
            "nanostack.configuration": "lowpan_border_router"
        }
    }
```

#### Thread configuration

The Thread specific parameters are listed below.

Parameter|Description
---------|-----------
`thread-br`|Must be set to true in order to build a Thread border router.
`commissioning-dataset-timestamp`|Is used for updating the Thread network parameters. The device with the highest value propagates the parameters to the network (in the same Realm-Local scope).
`nanostack.configuration`|Is needed when building the Thread border router from the Nanostack sources.

All devices must share the same network configuration parameters, when out of band commissioning is used. Special care must be taken when defining security related parameters. Note also that PSKc is generated from password, network name and extended PAN ID. The  configuration below is an example for testing purposes only; do not use them for production or expose them.

The `mbedtls_thread_config.h` file configures the mbed TLS for Thread use.

The feature `THREAD_BORDER_ROUTER` is the nanostack library, which implements the Thread border router networking stack.

For the Thread BR, there are two example configuration files for `SLIP` and `ETH` backhaul connectivity [configs](./configs). 

```
"config": {  
    "rf-channel": 22,
    "rf-channel-page": 0,
    "rf-channel-mask": "0x07fff800",
    "thread-br": "true",   
    "commissioning-dataset-timestamp": "0x00000001",
    "pan-id": "0x0700",
    "extended-pan-id": "{0xf1, 0xb5, 0xa1, 0xb2,0xc4, 0xd5, 0xa1, 0xbd }",
    "mesh-local-prefix": "{0xfd, 0xf1, 0xb5, 0xa1, 0xb2,0xc4, 0x0, 0x0}",
    "network-name": "\"Thread Network\"",
    "pskd": "\"abcdefghijklmno\"",
    "pskc": "{0xc8, 0xa6, 0x2e, 0xae, 0xf3, 0x68, 0xf3, 0x46, 0xa9, 0x9e, 0x57, 0x85, 0x98, 0x9d, 0x1c, 0xd0}",
    "thread-master-key": "{0x10, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff}"    
     },
    "macros": ["MBEDTLS_USER_CONFIG_FILE=\"source/mbedtls_thread_config.h\""],
    "target_overrides": {
        "*": {
            "target.features_add": ["NANOSTACK", "COMMON_PAL", "THREAD_BORDER_ROUTER"],
            "mbed-trace.enable": 1,
            "nanostack.configuration": "thread_border_router"
        }
    }
}
```

#### Backhaul connectivity

The FRDM-K64F border router application can be connected to a backhaul network. This enables you to connect the devices in a 6LoWPAN mesh network to the internet or a private LAN. Currently, the application supports SLIP (IPv6 encapsulation over a serial line) and Ethernet backhaul connectivity. 

```
"config": {               
    "backhaul-driver": "ETH",
    "backhaul-mac-src": "BOARD",
    "backhaul-mac": "{0x02, 0x00, 0x00, 0x00, 0x00, 0x01}",      
    "backhaul-dynamic-bootstrap": true,
    "backhaul-prefix": "\"fd00:300::\"",
    "backhaul-next-hop": "\"fe80::1\"",
    "backhaul-default-route": "\"::/0\"",
    .............
}
```

You can select your preferred option through the configuration file (field `backhaul-driver` in the `config` section). Value `SLIP` includes the SLIP driver, while the value `ETH` compiles the FRDM-K64F border router application with Ethernet backhaul support. You can define the MAC address on the backhaul interface manually (field `backhaul-mac-src` value `CONFIG`). Alternatively, you can use the MAC address provided by the development board (field `backhaul-mac-src` value `BOARD`). By default, the backhaul driver is set to be `ETH` and the MAC address source is `BOARD`. 

You can also set the backhaul bootstrap mode (field `backhaul-dynamic-bootstrap`). By default, the bootstrap mode is set to true, which means autonomous mode. With the autonomous mode, the border router learns the prefix information automatically from an IPv6 gateway in the ethernet/SLIP segment. When parameter is set to false, it enables you to set up a manual configuration of `backhaul-prefix` and `backhaul-default-route`.

If you use static bootstrap mode, you need to configure a default route on the backhaul interface to properly forward packets between the backhaul and the 6LoWPAN mesh network. In addition to this, you need to set a backhaul prefix. Static mode creates a site-local IPv6 network from where packets cannot be routed outside. 

When using the autonomous mode in the 6LoWPAN ND configuration, you can set the `prefix-from-backhaul` option to `true` to use the same backhaul prefix on the mesh network side as well. This allows for the mesh nodes to be directly connectable from the outside of the mesh network. In the Thread network, it is enough that `backhaul-dynamic-bootstrap` is set to true.

For more details on how to set the backhaul prefix and default route, refer to the [Nanostack Border Router](https://github.com/ARMmbed/nanostack-border-router-private) documentation.


#### Note on the SLIP backhaul driver

You need to use the UART1 serial line of the K64F board with the SLIP driver. See the `pins` section in the [mbed_app.json](./mbed_app.json) configuration. To use a different UART line, replace the `SERIAL_TX` and `SERIAL_RX` values with correct TX/RX pin names.

If you wish to use hardware flow control, set the configuration field `slip_hw_flow_control` to true. By default, it is set to false. Before using hardware flow control, make sure that the other end of your SLIP interface can handle flow control.

For the pin names of your desired UART line, refer to the [FRDM-K64F documentation](https://developer.mbed.org/platforms/FRDM-K64F/).

Example configuration for the SLIP driver:

```json
  "config" : {
        "SERIAL_TX": "PTE0",
        "SERIAL_RX": "PTE1",
        "SERIAL_CTS": "PTE2",
        "SERIAL_RTS": "PTE3"
  },
```

### Switching the RF shield

By default, the application uses an Atmel AT86RF233/212B RF driver. You can alternatively use the FRDM-MCR20A shield. The configuration for the RF driver can be set in the `json` file.

To select the Atmel radio shield, use the following:

```
        "radio-type":{
            "help": "options are ATMEL, MCR20",
            "value": "ATMEL"
        },
```

To select the NXP radio shield, use the following:

```
        "radio-type":{
            "help": "options are ATMEL, MCR20",
            "value": "MCR20"
        },
```

After changing the radio shield, you need to recompile the application.

## Build instructions

1. Install [mbed-cli](https://github.com/ARMmbed/mbed-cli).
2. Clone the repository: `git clone git@github.com:ARMmbed/k64f-border-router-private.git`
3. Modify the `mbed_app.json` file to reflect to your network setup or use the ready made configuration under the configs directory.
4. Deploy required libraries: `mbed deploy`
6. To build BR: 
   * Thread BR: `mbed compile -m K64F -t GCC_ARM --app-config configs/Thread_K64F_Atmel_RF_config.json`
   * 6LoWPAN ND BR: `mbed compile -m K64F -t GCC_ARM --app-config configs/6lowpan_K64F_Atmel_RF_config.json`

The binary is generated into `BUILD/K64F/GCC_ARM/k64f-border-router-private.bin`.

## Running the border router application

1. Find the  binary file `k64f-border-router-private.bin` in the folder `BUILD/K64F/GCC_ARM/`.
2. Copy the binary to the USB mass storage root of the FRDM-K64F development board. It is automatically flashed to the target MCU. When the flashing is complete, the board restarts itself. Press the **Reset** button of the development board if it does not restart automatically.
3. The program begins execution.
4. Open the [serial connection](#serial-connection-settings), for example PuTTY.

## Serial connection settings

Serial connection settings for the Thread test application are as follows:

	* Baud-rate = 115200
	* Data bits = 8
	* Stop bits = 1
	* flow control = xon/xoff

If there is no input from the serial terminal, press the **Reset** button of the development board.

In the PuTTY main screen, save the session and click **Open**. This opens a console window showing debug messages from the application. If the console screen is blank, you may need to press the **Reset** button of the board to see the debug information. The serial output from the 6LoWPAN border router looks something like this in the console:

```
[INFO][app ]: Starting K64F border router...
[INFO][brro]: NET_IPV6_BOOTSTRAP_AUTONOMOUS
[INFO][app ]: Using ETH backhaul driver...
[INFO][Eth ]: Ethernet cable is connected.
[INFO][addr]: Tentative Address added to IF 1: fe80::ac41:dcff:fe37:72c4
[INFO][addr]: DAD passed on IF 1: fe80::ac41:dcff:fe37:72c4
[INFO][addr]: Tentative Address added to IF 1: 2001:999:21:9ce:ac41:dcff:fe37:72c4
[INFO][addr]: DAD passed on IF 1: 2001:999:21:9ce:ac41:dcff:fe37:72c4
[INFO][brro]: Backhaul bootstrap ready, IPv6 = 2001:999:21:9ce:ac41:dcff:fe37:72c4
[INFO][brro]: Backhaul interface addresses:
[INFO][brro]:    [0] fe80::ac41:dcff:fe37:72c4
[INFO][brro]:    [1] 2001:999:21:9ce:ac41:dcff:fe37:72c4
[INFO][addr]: Address added to IF 0: fe80::ff:fe00:face
[INFO][br  ]: BR nwk base ready for start
[INFO][br  ]: Refresh Contexts
[INFO][br  ]: Refresh Prefixs
[INFO][addr]: Address added to IF 0: 2001:999:21:9ce:0:ff:fe00:face
[INFO][addr]: Address added to IF 0: fe80::fec2:3d00:4:a0cd
[INFO][brro]: RF bootstrap ready, IPv6 = 2001:999:21:9ce:0:ff:fe00:face
[INFO][brro]: RF interface addresses:
[INFO][brro]:    [0] fe80::ff:fe00:face
[INFO][brro]:    [1] fe80::fec2:3d00:4:a0cd
[INFO][brro]:    [2] 2001:999:21:9ce:0:ff:fe00:face
[INFO][brro]: 6LoWPAN Border Router Bootstrap Complete.

```

From the Thread border router:

```

[INFO][app ]: Starting K64F border router...
[DBG ][evlp]: event_loop_thread
[INFO][app ]: Using ETH backhaul driver...
[INFO][Eth ]: Ethernet cable is connected.
[DBG ][brro]: Backhaul driver ID: 0
[INFO][brro]: NET_IPV6_BOOTSTRAP_AUTONOMOUS
[INFO][brro]: mesh0 up
[DBG ][brro]: Create Mesh Interface
[INFO][brro]: network.mesh.net_rf_id: 0
[INFO][brro]: thread_interface_up
[DBG ][ThSA]: service init interface 0, port 61631, options 0
[DBG ][ThSA]: service tasklet init

coap messages......
...................

[INFO][brro]: mesh0 bootstrap ongoing..
[DBG ][brro]: backhaul_interface_up: 0
[DBG ][brro]: Backhaul interface ID: 1
[DBG ][brro]: Backhaul bootstrap started
[DBG ][ThSA]: service tasklet initialised
[INFO][brro]: BR interface_id 1.
[INFO][brro]: Ethernet (eth0) bootstrap ready. IP: 2001:999:21:9ce:a155:6b95:8384:f121
[DBG ][ThBrApp]: Eth0 connected
[DBG ][ThBrApp]: mesh0 is down
[INFO][brro]: Bootstrap ready
[DBG ][ThBrApp]: mesh0 connected

```
