/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * Copyright (c) 2015 Xilinx, Inc.
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RPMSG_CONFIG_H
#define _RPMSG_CONFIG_H

//#define VRING_SIZE                  0x1000
#define RL_BUFFER_COUNT             32      // Max number of descriptors (buffers)
#define RL_BUFFER_PAYLOAD_SIZE     (64-16)  // Always decrement 16 to obtain 2^n


#endif /* _RPMSG_CONFIG_H */
