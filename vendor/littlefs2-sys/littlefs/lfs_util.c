/*
 * lfs util functions
 *
 * Copyright (c) 2022, The littlefs authors.
 * Copyright (c) 2017, Arm Limited. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "lfs_util.h"

// Only compile if user does not provide custom config
#ifndef LFS_CONFIG


// The application supplies the STM32F4 hardware-backed implementation.
// It preserves littlefs's incremental, unfinalized CRC state convention.
#ifndef LFS_CRC
extern uint32_t xcore_lfs_crc32_update(
        uint32_t crc, const void *buffer, size_t size);

uint32_t lfs_crc(uint32_t crc, const void *buffer, size_t size) {
    return xcore_lfs_crc32_update(crc, buffer, size);
}
#endif


#endif
