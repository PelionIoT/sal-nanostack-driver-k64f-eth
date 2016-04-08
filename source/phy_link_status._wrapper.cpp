/*
 * Copyright (c) 2016 ARM Limited. All rights reserved.
 */

#include "mbed-drivers/mbed.h"
#include "sal-nanostack-driver-k64f-eth/phy_link_status_wrapper.h"

static void backhaul_link_status_polling()
{
    k64f_eth_phy_link_poll();
}

int8_t phy_link_wrapper_create()
{
    minar::Scheduler::postCallback(
            mbed::util::FunctionPointer0<void>(backhaul_link_status_polling).bind()).period(
            minar::milliseconds(1000));

    return 0;
}
