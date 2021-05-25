/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>

void main(void)
{
	printf("Hello World! %s %f\n", CONFIG_BOARD, 3.14F);
}
