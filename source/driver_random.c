/*
 * Copyright (c) 2016 ARM Limited. All rights reserved.
 */

#ifndef MBED_CONF_RTOS_PRESENT
#include "ns_types.h"

void arm_random_module_init(void)
{
}

uint32_t arm_random_seed_get(void)
{
    return 0;
}
#endif
