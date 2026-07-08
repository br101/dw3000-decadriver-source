/**
 * @file      deca_private.h
 *
 * @brief     QM33xxx UWB peripheral driver.
 *            This file is for driver-internal private objects.
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */
#ifndef DECA_PRIVATE_H
#define DECA_PRIVATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*! Entry point to AES IV 0, please see dw3xxx_deca_regs.h files */
#define DWT_AES_IV_ENTRY (0x10034)

/*!
 * This function is used to read from the UWB IC (e.g., DW3000, QM33xxx) device registers
 *
 * @param[in] regFileID : ID of register file or buffer being accessed
 * @param[in] index     : Byte index into register file or buffer being accessed
 * @param[in] length    : Number of bytes being read
 * @param[out] buffer   : Pointer to buffer in which to return the read data.
 *
 * @return None
 */
void dwt_writetodevice(uint32_t regFileID, uint16_t index, uint16_t length, uint8_t *buffer);

/*!
 * This function is used to write to the UWB IC (e.g., DW3000, QM33xxx) device registers
 *
 * @param[in] regFileID : ID of register file or buffer being accessed
 * @param[in] index     : Byte index into register file or buffer being accessed
 * @param[in] length    : Number of bytes to write
 * @param[in] buffer    : Pointer to buffer which contains the data to write.
 *
 * @return None
 */
void dwt_readfromdevice(uint32_t regFileID, uint16_t index, uint16_t length, uint8_t *buffer);

#ifdef __cplusplus
}
#endif

#endif // DECA_PRIVATE_H
