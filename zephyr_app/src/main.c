// Copyright (c) 2024-2025, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/net/socket.h>

// static struct k_timer my_timer;

static void timer_expiry_function(struct k_timer *timer_id)
{
    static int count = 0;
    printk("Timer expired %d times\n", ++count);
}

void main(void)
{
    dds_write(0, 0, 0);
    return 0;
}

