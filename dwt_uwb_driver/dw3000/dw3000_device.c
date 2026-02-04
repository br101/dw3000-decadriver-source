/**
 * @file      dw3000_device.c
 *
 * @brief     Decawave device configuration and control functions
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */
#include "deca_device_api.h"
#include "deca_interface.h"
#include "deca_types.h"
#include "dw3000_deca_regs.h"
#include "dw3000_deca_vals.h"
#include "deca_version.h"
#include "deca_rsl.h"
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define OPTSPEED __attribute__((optimize("O3")))

#if 0
#define DWT_API_ERROR_CHECK  /* API checks config input parameters */
#endif

#ifndef assert
  #define qassert(e)
#else
  #define qassert(e) assert(e)
#endif

// -------------------------------------------------------------------------------------------------------------------
// Module Macro definitions and enumerations

#define LOCAL_DATA(dw) (&((dw)->priv))

/* MACRO wrappers for SPI read/write : should be used only internally to DecaDriver */
#define dwt_write32bitreg(dw, addr, value) dwt_write32bitoffsetreg(dw, addr, 0U, value)
#define dwt_read32bitreg(dw, addr)         dwt_read32bitoffsetreg(dw, addr, 0U)
#define dwt_writefastCMD(dw, cmd)          ull_writetodevice(dw, cmd, 0U, 0U, NULL)

#define dwt_or8bitoffsetreg(dw, addr, offset, or_val)              dwt_modify8bitoffsetreg(dw, addr, offset, UINT8_MAX, or_val)
#define dwt_and8bitoffsetreg(dw, addr, offset, and_val)            dwt_modify8bitoffsetreg(dw, addr, offset, and_val, 0U)
#define dwt_and_or8bitoffsetreg(dw, addr, offset, and_val, or_val) dwt_modify8bitoffsetreg(dw, addr, offset, and_val, or_val)
#define dwt_set_bit_num_8bit_reg(dw, addr, bit_num)                dwt_modify8bitoffsetreg(dw, addr, 0U, UINT8_MAX, DWT_BIT_MASK(bit_num))
#define dwt_clr_bit_num_8bit_reg(dw, addr, bit_num)                dwt_modify8bitoffsetreg(dw, addr, 0U, ~DWT_BIT_MASK(bit_num), 0U)

#define dwt_or16bitoffsetreg(dw, addr, offset, or_val)              dwt_modify16bitoffsetreg(dw, addr, offset, UINT16_MAX, or_val)
#define dwt_and16bitoffsetreg(dw, addr, offset, and_val)            dwt_modify16bitoffsetreg(dw, addr, offset, and_val, 0U)
#define dwt_and_or16bitoffsetreg(dw, addr, offset, and_val, or_val) dwt_modify16bitoffsetreg(dw, addr, offset, and_val, or_val)
#define dwt_set_bit_num_16bit_reg(dw, addr, bit_num)                dwt_modify16bitoffsetreg(dw, addr, 0U, UINT16_MAX, DWT_BIT_MASK(bit_num))
#define dwt_clr_bit_num_16bit_reg(dw, addr, bit_num)                dwt_modify16bitoffsetreg(dw, addr, 0U, ~DWT_BIT_MASK(bit_num), 0U)

#define dwt_or32bitoffsetreg(dw, addr, offset, or_val)              dwt_modify32bitoffsetreg(dw, addr, offset, UINT32_MAX, or_val)
#define dwt_and32bitoffsetreg(dw, addr, offset, and_val)            dwt_modify32bitoffsetreg(dw, addr, offset, and_val, 0U)
#define dwt_and_or32bitoffsetreg(dw, addr, offset, and_val, or_val) dwt_modify32bitoffsetreg(dw, addr, offset, and_val, or_val)
#define dwt_set_bit_num_32bit_reg(dw, addr, bit_num)                dwt_modify32bitoffsetreg(dw, addr, 0U, UINT32_MAX, DWT_BIT_MASK(bit_num))
#define dwt_clr_bit_num_32bit_reg(dw, addr, bit_num)                dwt_modify32bitoffsetreg(dw, addr, 0U, ~DWT_BIT_MASK(bit_num), 0U)

// OTP addresses definitions :: See DW3000 Datasheet for calibration parameters OTP addresses
#define LDOTUNELO_ADDRESS    (0x04U)
#define LDOTUNEHI_ADDRESS    (0x05U)
#define PARTID_ADDRESS       (0x06U)
#define VBAT_ADDRESS         (0x08U)
#define VTEMP_ADDRESS        (0x09U)
#define WSLOTID_LOW_ADDRESS  (0x0DU)
#define WSLOTID_HIGH_ADDRESS (0x0EU)
#define XTRIM_ADDRESS        (0x1EU)
#define OTPREV_ADDRESS       (0x1FU)
#define BIAS_TUNE_ADDRESS    (0xAU)
#define DGC_TUNE_ADDRESS     (0x20U)
#define PLL_CC_ADDRESS       (0x35U)

/* internal arithmetic */
#define INT21_SIGN_BIT_MASK 0x00100000UL // sign bit mask for 21-bit signed integer values
#define INT21_SIGN_POWN     0x00200000UL // two's complement conversion constant for 21-bit signed integer values
#define DRX_CARRIER_INT_LEN 3U

#define CIA_MANUALLOWERBOUND_TH (0x10U) // CIA lower bound threshold values for STS for 64 MHz PRF
#define STSQUAL_THRESH_64_SH15     19661UL // = 0.60 * 32768

#define INT13_SIGN_BIT_MASK 0x1000U // sign bit mask for 13-bit signed integer values
#define INT13_SIGN_POWN     0x2000U // two's complement conversion constant for 13-bit signed integer values

#define INT14_SIGN_BIT_MASK 0x2000U // sign bit mask for 14-bit signed integer values
#define INT14_SIGN_POWN     0x4000U // two's complement conversion constant for 14-bit signed integer values

#define DWT_REG_DATA_MAX_LENGTH (0x3100U)

/* @note STS Minimum Threshold (STS_MNTH) needs to be adjusted with changing STS length.
 To adjust the STS_MNTH following formula can be used: STS_MNTH = SQRT(X/Y)*default_STS_MNTH
 The default_STS_MNTH is 0x10,
 X is the length of the STS in units of 8 (i.e. 8 for 64 length, 16 for 128 length etc.)
 Y is either 8 or 16. It is set to 8 when no PDOA or PDOA mode 1 and 16 for PDOA mode 3.

 The API does not use the formula and the STS_MNTH value is derived from approximation formula as given by get_sts_mnth()
 function. The API here supports STS lengths as listed in: @ref dwt_sts_lengths_e enum, which are: @ref DWT_STS_LEN_16, DWT_STS_LEN_32, .., @ref DWT_STS_LEN_2048
 The enum value is used as the index into @ref sts_length_factors array. The array has values which are generated by:
 val = SQRT(stsLength/16)*2048
 */
static const uint16_t sts_length_factors[STS_LEN_SUPPORTED] = { 724U, 1024U, 1448U, 2048U, 2896U, 4096U, 5793U, 8192U };

/* regNames contains register name, value pairs which are used for debug output/logging by  externa applications e.g. DecaRanging. */
static register_name_add_t regNames[] =
{
#ifdef _DGB_LOG
    { "IP_TOA_LO", IP_TOA_LO_ID }, { "IP_TOA_HI", IP_TOA_HI_ID }, { "CY0_TOA_LO", STS_TOA_LO_ID }, { "CY0_TOA_HI", STS_TOA_HI_ID }, { "CY1_TOA_LO", STS1_TOA_LO_ID },
    { "CY1_TOA_HI", STS1_TOA_HI_ID }, { "CIA_TDOA_0", CIA_TDOA_0_ID }, { "CIA_TDOA_1_PDOA", CIA_TDOA_1_PDOA_ID }, { "CIA_DIAG_0", CIA_DIAG_0_ID }, { "CIA_DIAG_1", CIA_DIAG_1_ID },
    { "IP_DIAG_0", IP_DIAG_0_ID }, { "IP_DIAG_1", IP_DIAG_1_ID }, { "IP_DIAG_2", IP_DIAG_2_ID }, { "IP_DIAG_3", IP_DIAG_3_ID }, { "IP_DIAG_4", IP_DIAG_4_ID }, { "IP_DIAG_5", IP_DIAG_5_ID },
    { "IP_DIAG_6", IP_DIAG_6_ID }, { "IP_DIAG_7", IP_DIAG_7_ID }, { "IP_DIAG_8", IP_DIAG_8_ID }, { "IP_DIAG_9", IP_DIAG_9_ID }, { "IP_DIAG_10", IP_DIAG_10_ID },
    { "IP_DIAG_11", IP_DIAG_11_ID }, { "IP_DIAG_12", IP_DIAG_12_ID }, { "CY0_DIAG_0", STS_DIAG_0_ID }, { "CY0_DIAG_1", STS_DIAG_1_ID }, { "CY0_DIAG_2", STS_DIAG_2_ID },
    { "CY0_DIAG_3", STS_DIAG_3_ID }, { "CY0_DIAG_4", STS_DIAG_4_ID }, { "CY0_DIAG_5", STS_DIAG_5_ID }, { "CY0_DIAG_6", STS_DIAG_6_ID }, { "CY0_DIAG_7", STS_DIAG_7_ID },
    { "CY0_DIAG_8", STS_DIAG_8_ID }, { "CY0_DIAG_9", STS_DIAG_9_ID }, { "CY0_DIAG_10", STS_DIAG_10_ID }, { "CY0_DIAG_11", STS_DIAG_11_ID }, { "CY0_DIAG_12", STS_DIAG_12_ID },
    { "CY0_DIAG_13", STS_DIAG_13_ID }, { "CY0_DIAG_14", STS_DIAG_14_ID }, { "CY0_DIAG_15", STS_DIAG_15_ID }, { "CY0_DIAG_16", STS_DIAG_16_ID }, { "CY0_DIAG_17", STS_DIAG_17_ID },
    { "CY1_DIAG_0", STS1_DIAG_0_ID }, { "CY1_DIAG_1", STS1_DIAG_1_ID }, { "CY1_DIAG_2", STS1_DIAG_2_ID }, { "CY1_DIAG_3", STS1_DIAG_3_ID }, { "CY1_DIAG_4", STS1_DIAG_4_ID },
    { "CY1_DIAG_5", STS1_DIAG_5_ID }, { "CY1_DIAG_6", STS1_DIAG_6_ID }, { "CY1_DIAG_7", STS1_DIAG_7_ID }, { "CY1_DIAG_8", STS1_DIAG_8_ID }, { "CY1_DIAG_9", STS1_DIAG_9_ID },
    { "CY1_DIAG_10", STS1_DIAG_10_ID }, { "CY1_DIAG_11", STS1_DIAG_11_ID }, { "CY1_DIAG_12", STS1_DIAG_12_ID }, { "RX_ANTENNA_DELAY", CIA_CONF_ID },
    { "FP_CONFIDENCE_LIMIT", FP_CONF_ID }, { "IP_CONFIG_LO", IP_CONFIG_LO_ID }, { "IP_CONFIG_HI",IP_CONFIG_HI_ID }, { "CY_CONFIG_LO", STS_CONFIG_LO_ID },
    { "CY_CONFIG_HI", STS_CONFIG_HI_ID }, { "PGF_DELAY_COMP_LO", PGF_DELAY_COMP_LO_ID }, { "PGF_DELAY_COMP_HI",PGF_DELAY_COMP_HI_ID },
    { "SAR_CTRL", SAR_CTRL_ID }, { "CP_CFG0", STS_CFG0_ID }, { "CP_CTRL_ID", STS_CTRL_ID },{ "CP_STS", STS_STS_ID },{ "LCSS_MARGIN", LCSS_MARGIN_ID },{ NULL, 0 }
#else
    { NULL, 0 }
#endif
};

// -------------------------------------------------------------------------------------------------------------------
#define FORCE_CLK_SYS_TX      1
#define FORCE_CLK_AUTO        5
#define FORCE_SYSCLK_PLL      2U
#define FORCE_SYSCLK_FOSCDIV4 1U
#define FORCE_SYSCLK_FOSC     3U
#define FORCE_CLK_PLL         2U

/* Fast Access Commands (FAC)
 * only write operation is possible for this mode
 * bit_7=one is W operation, bit_6=zero: FastAccess command, bit_[5..1] addr, bits_0=one: MODE of FastAccess
 */
#define DW3000_SPI_FAC (0U << 6U | 1U << 0U)

/* Fast Access Commands with Read/Write support (FACRW)
 * bit_7 is R/W operation, bit_6=zero: FastAccess command, bit_[5..1] addr, bits_0=zero: MODE of FastAccess
 */
#define DW3000_SPI_FACRW (0U << 6U | 0U << 0U)

/* Extended Address Mode with Read/Write support (EAMRW)
 * b[0] = bit_7 is R/W operation, bit_6 one = ExtendedAddressMode;
 * b[1] = addr<<2 | (mode&0x3)
 */
#define DW3000_SPI_EAMRW (1U << 6U)

#define RSL_QUANTIZATION_FACTOR (21)

// -------------------------------------------------------------------------------------------------------------------
// TxPower Adjustment
//
// Each value corresponds to the output TxPower difference between the
// coarse gain setting (i+1) and (i)
// The step is in unit of 0.1dB

static const uint8_t lut_coarse_gain[NUM_COARSE_GAIN] =
{
    32,         // TxPower gain from coarse setting 0x0 to 0x1
    13,         // TxPower gain from coarse setting 0x1 to 0x2
    5          // TxPower gain from coarse setting 0x2 to 0x3
};

// Each value corresponds to the output TxPower difference between the
// fine gain setting (i+1) and (i)
// The step is in unit of 0.1dB

static const uint8_t fine_gain_lut_chan5[LUT_COMP_SIZE] =
{
    0, 32, 29, 28, 20, 18, 12, 13,
    10, 10, 7, 8, 6, 7, 5, 6,
    5, 5, 4, 4, 4, 4, 3, 3,
    3, 3, 2, 3, 2, 3, 2, 3,
    3, 2, 2, 2, 1, 2, 1, 2,
    1, 2, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1
};

static const uint8_t fine_gain_lut_chan9[LUT_COMP_SIZE] =
{
    0, 11, 14, 18, 15, 15, 10, 12,
    9, 9, 7, 8, 6, 7, 5, 6,
    5, 5, 4, 5, 4, 4, 3, 4,
    3, 3, 3, 3, 3, 3, 2, 3,
    3, 2, 2, 2, 2, 2, 1, 2,
    2, 2, 2, 2, 1, 2, 1, 2,
    1, 1, 1, 2, 1, 1, 1, 1,
    1, 1, 1, 1, 0, 1, 1, 1
};

/* Linear Tx Power */

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

//Naming convention tx power look up tables: PX_BY_CZ
//  PX => pa state ; 1 for ON, 0 for OFF
//  BY => bias trim value
//  CZ => channel

typedef struct
{
    const uint8_t* lut;
    uint8_t lut_size;
    uint8_t start_index;
    uint8_t end_index;
    uint8_t offset_index;
    uint8_t bias;
} tx_adj_lut_t;

typedef struct
{
    tx_adj_lut_t tx_frame_lut;
} txp_lut_t;

static const uint8_t dwt_txp_lut_p0_b1_c5[] = {
    0xfe, 0xda, 0xc2, 0xb2, 0xa2, 0x96, 0x8a,
    0x82, 0xe1, 0xc5, 0xb1, 0xa1, 0x95, 0x89,
    0x81, 0x79, 0x52, 0x4e, 0x4a, 0x61, 0x5d,
    0x42, 0x55, 0x3e, 0x4d, 0x3a, 0x45, 0x36,
    0x41, 0x32, 0x3d, 0x2e, 0x39, 0x35, 0x2a,
    0x31, 0x31, 0x26, 0x2d, 0x50, 0x22, 0x29,
    0x48, 0x25, 0x1e, 0x1e, 0x3c, 0x21, 0x21,
    0x34, 0x1d, 0x1d, 0x30, 0x2c, 0x2c, 0x2c,
    0x28, 0x28, 0x28, 0x24, 0x24, 0x24, 0x20,
    0x20, 0x20, 0x1c, 0x1c
};

static const uint8_t dwt_txp_lut_p0_b7_c5[] = {
    0xfe, 0xee, 0xe2, 0xd6, 0xca, 0xc2, 0xba,
    0xb2, 0xf9, 0xe9, 0x9a, 0xd1, 0xc5, 0x8a,
    0xb5, 0xad, 0xa5, 0x9d, 0x95, 0x91, 0x89,
    0x85, 0x81, 0x7d, 0x5e, 0x75, 0x71, 0x6d,
    0x52, 0x65, 0x61, 0x5d, 0x59, 0x46, 0x55,
    0x51, 0x4d, 0x3e, 0x49, 0x3a, 0x45, 0x36,
    0x41, 0x3d, 0x32, 0x39, 0x39, 0x2e, 0x35,
    0x58, 0x31, 0x31, 0x50, 0x2d, 0x2d, 0x48,
    0x44, 0x40, 0x40, 0x3c, 0x3c, 0x3c, 0x38,
    0x38, 0x34, 0x34, 0x30, 0x30, 0x30, 0x2c
};

static const uint8_t dwt_txp_lut_p0_b1_c9[] = {
    0xfe, 0xe6, 0xd6, 0xc6, 0xba, 0xb2, 0xa6,
    0x9e, 0xed, 0x8e, 0xc9, 0xbd, 0x7e, 0x7a,
    0x9d, 0x95, 0x91, 0x89, 0x81, 0x7d, 0x79,
    0x75, 0x56, 0x52, 0x65, 0x61, 0x4a, 0x46,
    0x59, 0x55, 0x51, 0x3e, 0x4d, 0x49, 0x3a,
    0x45, 0x36, 0x41, 0x32, 0x3d, 0x39, 0x2e,
    0x35, 0x35, 0x2a, 0x31, 0x31, 0x26, 0x2d,
    0x4c, 0x29, 0x22, 0x44, 0x25, 0x25, 0x1e,
    0x3c, 0x21, 0x21, 0x34, 0x34, 0x1d, 0x1d,
    0x30, 0x2c, 0x2c, 0x28, 0x28, 0x28, 0x24,
    0x24, 0x24, 0x24, 0x20, 0x20, 0x20, 0x20,
    0x1c
};

static const uint8_t dwt_txp_lut_p0_b7_c9[] = {
    0xfe, 0xf2, 0xea, 0xde, 0xd6, 0xce, 0xc6,
    0xbe, 0xf9, 0xb2, 0xe5, 0xa6, 0x9e, 0xc9,
    0xc1, 0x8e, 0x8a, 0xad, 0xa5, 0x7e, 0x7a,
    0x76, 0x72, 0x6e, 0x85, 0x81, 0x7d, 0x62,
    0x5e, 0x5a, 0x6d, 0x56, 0x52, 0x65, 0x61,
    0x5d, 0x4a, 0x46, 0x55, 0x42, 0x51, 0x4d,
    0x3e, 0x49, 0x3a, 0x45, 0x36, 0x41, 0x3d,
    0x32, 0x39, 0x2e, 0x35, 0x35, 0x54, 0x31,
    0x31, 0x2d, 0x2d, 0x48, 0x48, 0x44, 0x40,
    0x40, 0x3c, 0x3c, 0x3c, 0x38, 0x38, 0x34,
    0x34, 0x30, 0x30, 0x30, 0x2c
};

#define GET_TXP_LUT(chan, pa, bias) dwt_txp_lut_p ## pa ## _b ## bias ## _c ## chan

// RAW LUTs sizes
#define ARRAY_SIZE_LUT(X) sizeof(X) / sizeof(X[0])
#define GET_TXP_LUT_SIZE(chan, pa, bias) ARRAY_SIZE_LUT(GET_TXP_LUT(chan, pa, bias))

// Start index allowed for each LUT
#define MIN_IDX_P0_B7_C5_SOC 0U
#define MIN_IDX_P0_B1_C5_SOC 0U

#define MIN_IDX_P0_B7_C9_SOC 0U
#define MIN_IDX_P0_B1_C9_SOC 0U

// Max index allowed for each LUT
#define MAX_IDX_P0_B7_C5_SOC 28U
#define MAX_IDX_P0_B1_C5_SOC (GET_TXP_LUT_SIZE(5, 0, 1) - 1U - MIN_IDX_P0_B1_C5_SOC + 1U) + (MAX_IDX_P0_B7_C5_SOC - MIN_IDX_P0_B7_C5_SOC + 1U) - 1U

#define MAX_IDX_P0_B7_C9_SOC 16U
#define MAX_IDX_P0_B1_C9_SOC (GET_TXP_LUT_SIZE(9, 0, 1) - 1U - MIN_IDX_P0_B1_C9_SOC + 1U) + (MAX_IDX_P0_B7_C9_SOC - MIN_IDX_P0_B7_C9_SOC + 1U) - 1U


#define GET_MIN_INDEX_SIP(chan, pa, bias) (MIN_IDX_P## pa ## _B ## bias ## _C ## chan ## _SIP)
#define GET_MIN_INDEX_SOC(chan, pa, bias) (MIN_IDX_P## pa ## _B ## bias ## _C ## chan ## _SOC)

#define GET_MAX_INDEX_SIP(chan, pa, bias) (MAX_IDX_P## pa ## _B ## bias ## _C ## chan ## _SIP)
#define GET_MAX_INDEX_SOC(chan, pa, bias) (MAX_IDX_P## pa ## _B ## bias ## _C ## chan ## _SOC)

/* the CIR accumulator offset to read from*/
static const uint16_t dwt_cir_acc_offset[NUM_OF_DWT_ACC_IDX] = {0x0U, 0x400U, 0x600U};

// -------------------------------------------------------------------------------------------------------------------
// Internal functions prototypes for controlling and configuring the device
//
static uint32_t dwt_read32bitoffsetreg(dwchip_t *dw, uint32_t regFileID, uint16_t regOffset);
static uint8_t dwt_read8bitoffsetreg(dwchip_t *dw, uint32_t regFileID, uint16_t regOffset);
static void dwt_write8bitoffsetreg(dwchip_t *dw, uint32_t regFileID, uint16_t regOffset, uint8_t regval);
static uint32_t dwt_otpreadword32(dwchip_t *dw, uint16_t address);
static void dwt_otpprogword32(dwchip_t *dw, uint32_t data, uint16_t address);
static void ull_force_clocks(dwchip_t *dw, int32_t clocks);
static uint8_t ull_calcbandwidthadj(dwchip_t *dw, uint16_t target_count);
static int32_t ull_run_pgfcal(dwchip_t *dw);
static int32_t ull_pgf_cal(dwchip_t *dw, int32_t ldoen);
static int32_t ull_setplenfine(dwchip_t *dw, uint16_t preambleLength);
static int ull_setpllrxprebufen(dwchip_t *dw, dwt_pll_prebuf_cfg_e pll_rx_prebuf_cfg);
static uint16_t ull_getframelength(dwchip_t *dw, uint8_t *rng_bit);
static int32_t ull_check_dev_id(dwchip_t *dw);
static void ull_enable_rftx_blocks(dwchip_t *dw);
static void ull_disable_rftx_blocks(dwchip_t *dw);
static void ull_increase_ch5_pll_ldo_tune(dwchip_t *dw);
static int32_t ull_setchannel(dwchip_t *dw, uint8_t ch);
static void ull_dis_otp_ips(dwchip_t *dw, int32_t mode);
static float ull_convertrawtemperature(dwchip_t *dw, uint8_t raw_temp);
static uint16_t ull_readtempvbat(dwchip_t *dw);
static uint16_t ull_readsar(dwchip_t *dw, uint8_t input_mux, uint8_t attn);
static int32_t ull_pll_ch5_auto_cal(dwchip_t *dw, uint32_t coarse_code, uint16_t sleep_us, uint8_t steps, uint8_t *p_num_steps_lock, int8_t temperature);
static int32_t ull_pll_ch9_auto_cal(dwchip_t *dw, uint32_t coarse_code, uint16_t sleep_us, uint8_t steps, uint8_t *p_num_steps_lock);
static void ull_update_ststhreshold(dwchip_t *dw, uint8_t stsBlocks);
static void ull_setstslength(dwchip_t *dw, dwt_sts_lengths_e sts_len);
static inline uint8_t ull_getrxcode(dwchip_t *dw);
static void ull_configuresfdtype(dwchip_t *dw, uint8_t sfdType);
static void ull_settxcode(dwchip_t *dw, uint8_t tx_code);
static void ull_setrxcode(dwchip_t *dw, uint8_t rx_code);
static int32_t ull_pll_cal(dwchip_t *dw);
static void ull_update_dgc_config(struct dwchip_s *dw, uint32_t channel);
static uint8_t ull_get_dgcdecision(dwchip_t *dw);
static uint16_t get_sts_mnth(uint16_t len_factor, uint8_t threshold, uint8_t shift_val);
static void config_sts_mnth(dwchip_t *dw, dwt_pdoa_mode_e pdoaMode);
static void ull_restore_common(dwchip_t *dw);
static int32_t ull_restore_txrx(dwchip_t *dw, uint8_t restore_mask);
static void ull_setpllbiastrim(struct dwchip_s *dw, uint8_t pll_bias_trim);

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function returns configured RX code
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return Configured RX code.
 */
static inline uint8_t ull_getrxcode(dwchip_t *dw)
{
    return (uint8_t)((dwt_read32bitoffsetreg(dw, CHAN_CTRL_ID, 0) & CHAN_CTRL_RX_PCODE_BIT_MASK) >> (uint32_t)CHAN_CTRL_RX_PCODE_BIT_OFFSET);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function updates the current STS threshold.
 *        The threshold is used to check STS quality (@ref STSQUAL_THRESH_64) and report if the received STS is OK.
 *
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] stsBlocks: The STS length in 8 "symbol" blocks - 1, e.g., stsBlocks of 3 will set a length of 32, @ref dwt_sts_lengths_e
 *
 * @return None
 */
static void ull_update_ststhreshold(dwchip_t *dw, uint8_t stsBlocks)
{
    const uint32_t stslen = (uint32_t)stsBlocks + 1UL;
    uint32_t ststhresh;

    ststhresh = (stslen * 8UL * (uint32_t)STSQUAL_THRESH_64_SH15) >> 15UL;
    LOCAL_DATA(dw)->ststhreshold = (int16_t)ststhresh;

    /* Cache variables needed for ull_setpdoamode(). */
    LOCAL_DATA(dw)->stsLength = (dwt_sts_lengths_e)stsBlocks;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will set the STS length. The STS length is specified in blocks. A block is 8 us duration,
 *        0 value specifies 8 us duration, the max value is 255 == 2048 us.
 *
 *        @note dwt_configure() must be called prior to this to configure the PRF
 *        To set the PRF dwt_settxcode() is used.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] sts_len: Number of STS 8us blocks (0 == 8us, 1 == 16us, etc), @ref dwt_sts_lengths_e
 *
 * @return  None
 */
static void ull_setstslength(dwchip_t *dw, dwt_sts_lengths_e sts_len)
{
    dwt_write8bitoffsetreg(dw, STS_CFG0_ID, 0U, (uint8_t)sts_len);
    ull_update_ststhreshold(dw, (uint8_t)sts_len);
    
    config_sts_mnth(dw, LOCAL_DATA(dw)->pdoaMode); //configure the STS Minimum Threshold
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  This function wakes up the device by an IO pin. UWB IC SPI_CS or WAKEUP pins can be used for this, as
 *         configured by ull_configuresleep()
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return  None
 */
static void ull_wakeup_ic(dwchip_t *dw)
{
#ifndef WIN32
    dw->wakeup_device_with_io();
#else
    (void)dw;
#endif
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  This function is used to read/write to the DW3000 device registers
 *
 * @param[in] dw          : DW3000 chip descriptor handler.
 * @param[in] regFileID   : ID of register file or buffer being accessed
 * @param[in] index       : Byte index into register file or buffer being accessed
 * @param[in] length      : Number of bytes being written
 * @param[in] buffer      : Pointer to buffer containing the 'length' bytes to be written or read
 * @param[in] mode        : DW3000_SPI_WR_BIT/DW3000_SPI_RD_BIT
 *
 * @return  None
 */
static void dwt_xfer3xxx(dwchip_t *dw,
    uint32_t regFileID, // 0x0, 0x04-0x7F ; 0x10000, 0x10004, 0x10008-0x1007F; 0x20000 etc
    uint16_t indx,      // sub-index, calculated from regFileID 0..0x7F,
    uint16_t length, uint8_t *buffer, const spi_modes_e mode)
{
    uint8_t header[2]; // Buffer to compose header in
    uint16_t cnt = 0U;  // Counter for length of a header

    uint16_t reg_file = (uint16_t)(0x1FUL & ((regFileID + indx) >> 16UL));
    uint16_t reg_offset = (uint16_t)(0x7FUL & (regFileID + indx));

    bool loop_forever = false;

    qassert(reg_file <= 0x1FU);
    qassert(reg_offset <= 0x7FU);
    qassert(length < DWT_REG_DATA_MAX_LENGTH); /* Check if length is correct */
    qassert(
        mode == DW3000_SPI_WR_BIT || mode == DW3000_SPI_RD_BIT || mode == DW3000_SPI_AND_OR_8 || mode == DW3000_SPI_AND_OR_16 || mode == DW3000_SPI_AND_OR_32);

    uint16_t addr;
    addr = (reg_file << 9U) | (reg_offset << 2U);

    header[0] = (uint8_t)(((const uint16_t)mode | addr) >> 8U);   // bit7 + addr[4:0] + sub_addr[6:6]
    header[1] = (uint8_t)(addr | ((const uint16_t)mode & 0x03U)); // Extended Address Mode: subaddr[5:0]+ R/W/AND_OR

    if (/*reg_offset == 0 && */ (length == 0U) && (mode != DW3000_SPI_RD_BIT)) // do not enter here if reading 0 bytes
    { /* Fast Access Commands (FAC) */
        qassert(mode == DW3000_SPI_WR_BIT);

        header[0] = (uint8_t)(((uint32_t)DW3000_SPI_WR_BIT >> 8UL) | (regFileID << 1UL) | (uint32_t)DW3000_SPI_FAC);
        cnt = 1U;
    }
    else if (reg_offset == 0U /*&& length > 0*/
             && (mode == DW3000_SPI_WR_BIT || mode == DW3000_SPI_RD_BIT))
    { /* Fast Access Commands with Read/Write support (FACRW) */
        header[0] |= DW3000_SPI_FACRW;
        cnt = 1U;
    }
    else
    { /* Extended Address Mode with Read/Write support (EAMRW) */
        header[0] |= DW3000_SPI_EAMRW;
        cnt = 2U;
    }

    switch (mode)
    {
    case DW3000_SPI_AND_OR_8:
    case DW3000_SPI_AND_OR_16:
    case DW3000_SPI_AND_OR_32:
    case DW3000_SPI_WR_BIT:
    {
        uint8_t crc8 = 0U;
        if (LOCAL_DATA(dw)->spicrc != DWT_SPI_CRC_MODE_NO)
        {
            // generate 8 bit CRC
            crc8 = dwt_generatecrc8(header, cnt, 0U);
            crc8 = dwt_generatecrc8(buffer, length, crc8);

            // Write it to the SPI
            (void)dw->SPI->writetospiwithcrc(cnt, header, length, buffer, crc8);
        }
        else
        {
            // Write it to the SPI
            (void)dw->SPI->writetospi(cnt, header, length, buffer);
        }
        break;
    }
    case DW3000_SPI_RD_BIT:
    {
        (void)dw->SPI->readfromspi(cnt, header, length, buffer);

        // check that the SPI read has correct CRC-8 byte
        // also don't do for SPICRC_CFG_ID register itself to prevent infinite recursion
        if ((LOCAL_DATA(dw)->spicrc == DWT_SPI_CRC_MODE_WRRD) && (regFileID != SPICRC_CFG_ID))
        {
            uint8_t crc8, dwcrc8;
            // generate 8 bit CRC from the read data
            crc8 = dwt_generatecrc8(header, cnt, 0U);
            crc8 = dwt_generatecrc8(buffer, length, crc8);

            // read the CRC that was generated in the DW3000 for the read transaction
            dwcrc8 = dwt_read8bitoffsetreg(dw, SPICRC_CFG_ID, 0U);

            // if the two CRC don't match report SPI read error
            // potential problem in callback if it will try to read/write SPI with CRC again.
            if (crc8 != dwcrc8)
            {
                if (dw->callbacks.cbSPIRDErr != NULL)
                {
                    dw->callbacks.cbSPIRDErr();
                }
            }
        }
        break;
    }
    default:
        loop_forever = true;
        break;
    }

    if (loop_forever == true) {
        while (true)
            {}
    }

} // end dwt_xfer3xxx()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  This function is used to write to the DW3000 device registers
 *
 * @param[in] dw         : DW3000 chip descriptor handler.
 * @param[in] regFileID  : ID of register file or buffer being accessed
 * @param[in] index      : Byte index into register file or buffer being accessed
 * @param[in] length     : Number of bytes being written
 * @param[in] buffer     : Pointer to buffer containing the 'length' bytes to be written
 *
 * @return None
 */
static void ull_writetodevice(dwchip_t *dw, uint32_t regFileID, uint16_t index, uint16_t length, uint8_t *buffer)
{
    dwt_xfer3xxx(dw, regFileID, index, length, buffer, DW3000_SPI_WR_BIT);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  This function is used to read from the DW3000 device registers
 *
 * @param[in] dw        : DW3000 chip descriptor handler.
 * @param[in] regFileID : ID of register file or buffer being accessed
 * @param[in] index     : Byte index into register file or buffer being accessed
 * @param[in] length    : Number of bytes being read
 * @param[out] buffer   : Pointer to buffer in which to return the read data.
 *
 * @return None
 */
static void ull_readfromdevice(dwchip_t *dw, uint32_t regFileID, uint16_t index, uint16_t length, uint8_t *buffer)
{
    dwt_xfer3xxx(dw, regFileID, index, length, buffer, DW3000_SPI_RD_BIT);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  This function is used to read 32-bit value in the device register
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] regFileID: ID of register file or buffer being accessed
 * @param[in] regOffset: The index into register file or buffer being accessed
 *
 * @returns 32-bit register value
 */
static uint32_t dwt_read32bitoffsetreg(dwchip_t *dw, uint32_t regFileID, uint16_t regOffset)
{
    uint32_t regval = 0UL;
    uint8_t buffer[4];

    ull_readfromdevice(dw, regFileID, regOffset, 4U, buffer); // Read 4 bytes (32-bits) register into buffer

    for (int32_t j = 3; j >= 0; j--)
    {
        regval = (regval << 8U) + buffer[j];
    }

    return regval;

} // end dwt_read32bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  This function is used to read 16-bit value in the device register
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] regFileID: ID of register file or buffer being accessed
 * @param[in] regOffset: The index into register file or buffer being accessed
 *
 * @returns 16-bit register value
 */
static uint16_t dwt_read16bitoffsetreg(dwchip_t *dw, uint32_t regFileID, uint16_t regOffset)
{
    uint16_t regval = 0U;
    uint8_t buffer[2];

    // Read 2 bytes (16-bits) register into buffer
    ull_readfromdevice(dw, regFileID, regOffset, 2U, buffer);

    regval = ((uint16_t)buffer[1] << 8U) + (uint16_t)buffer[0];

    return regval;
} // end dwt_read16bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  This function is used to read 8-bit value in the device register
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] regFileID: ID of register file or buffer being accessed
 * @param[in] regOffset: The index into register file or buffer being accessed
 *
 * @returns 8-bit register value
 */
static uint8_t dwt_read8bitoffsetreg(dwchip_t *dw, uint32_t regFileID, uint16_t regOffset)
{
    uint8_t regval;

    ull_readfromdevice(dw, regFileID, regOffset, 1U, &regval);

    return regval;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  This function is used to write 32-bit value into the device's register
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] regFileID: ID of register file or buffer being accessed
 * @param[in] regOffset: The index into register file or buffer being accessed
 * @param[in] regval   : The 32-bit value to write
 *
 * @return None
 */
static void dwt_write32bitoffsetreg(dwchip_t *dw, uint32_t regFileID, uint16_t regOffset, uint32_t regval)
{
    uint8_t buffer[4];

    for (uint8_t j = 0U; j < 4U; j++)
    {
        buffer[j] = (uint8_t)regval;
        regval >>= 8U;
    }

    ull_writetodevice(dw, regFileID, regOffset, 4U, buffer);
} // end dwt_write32bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  This function is used to write 16-bit value into the device's register
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] regFileID: ID of register file or buffer being accessed
 * @param[in] regOffset: The index into register file or buffer being accessed
 * @param[in] regval   : The 16-bit value to write
 *
 * @return None
 */
static void dwt_write16bitoffsetreg(dwchip_t *dw, uint32_t regFileID, uint16_t regOffset, uint16_t regval)
{
    uint8_t buffer[2];

    buffer[0] = (uint8_t)regval;
    buffer[1] = (uint8_t)(regval >> 8U);

    ull_writetodevice(dw, regFileID, regOffset, 2U, buffer);
} // end dwt_write16bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  This function is used to write 8-bit value into the device's register
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] regFileID: ID of register file or buffer being accessed
 * @param[in] regOffset: The index into register file or buffer being accessed
 * @param[in] regval   : The 8-bit value to write
 *
 * @return None
 */
static void dwt_write8bitoffsetreg(dwchip_t *dw, uint32_t regFileID, uint16_t regOffset, uint8_t regval)
{
    ull_writetodevice(dw, regFileID, regOffset, 1U, &regval);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  This function is used to modify a 32-bit value in the device's register
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] regFileID: ID of register file or buffer being accessed
 * @param[in] regOffset: The index into register file or buffer being accessed
 * @param[in] regval_and: The value to AND to register
 * @param[in] regval_or: The value to OR to register
 *
 * @return None
 */
static void dwt_modify32bitoffsetreg(dwchip_t *dw, const uint32_t regFileID, const uint16_t regOffset, const uint32_t and_value, const uint32_t or_value)
{
    uint8_t buf[8];
    buf[0] = (uint8_t)and_value;           // &0xFF;
    buf[1] = (uint8_t)(and_value >> 8UL);  // &0xFF;
    buf[2] = (uint8_t)(and_value >> 16UL); // &0xFF;
    buf[3] = (uint8_t)(and_value >> 24UL); // &0xFF;
    buf[4] = (uint8_t)or_value;            // &0xFF;
    buf[5] = (uint8_t)(or_value >> 8UL);   // &0xFF;
    buf[6] = (uint8_t)(or_value >> 16UL);  // &0xFF;
    buf[7] = (uint8_t)(or_value >> 24UL);  // &0xFF;
    dwt_xfer3xxx(dw, regFileID, regOffset, (const uint16_t)sizeof(buf), buf, DW3000_SPI_AND_OR_32);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  This function is used to modify a 16-bit value in the device's register
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] regFileID: ID of register file or buffer being accessed
 * @param[in] regOffset: The index into register file or buffer being accessed
 * @param[in] regval_and: The value to AND to register
 * @param[in] regval_or: The value to OR to register
 *
 * @return None
 */
static void dwt_modify16bitoffsetreg(dwchip_t *dw, const uint32_t regFileID, const uint16_t regOffset, const uint16_t and_value, const uint16_t or_value)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)and_value;        // &0xFF;
    buf[1] = (uint8_t)(and_value >> 8U); // &0xFF;
    buf[2] = (uint8_t)or_value;         // &0xFF;
    buf[3] = (uint8_t)(or_value >> 8U);  // &0xFF;
    dwt_xfer3xxx(dw, regFileID, regOffset, (const uint16_t)sizeof(buf), buf, DW3000_SPI_AND_OR_16);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  This function is used to modify a 8-bit value in the device's register
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] regFileID: ID of register file or buffer being accessed
 * @param[in] regOffset: The index into register file or buffer being accessed
 * @param[in] regval_and: The value to AND to register
 * @param[in] regval_or: The value to OR to register
 *
 * @return None
 */
static void dwt_modify8bitoffsetreg(dwchip_t *dw, const uint32_t regFileID, const uint16_t regOffset, const uint8_t and_value, const uint8_t or_value)
{
    uint8_t buf[2];
    buf[0] = and_value;
    buf[1] = or_value;
    dwt_xfer3xxx(dw, regFileID, regOffset, (const uint16_t)sizeof(buf), buf, DW3000_SPI_AND_OR_8);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to enable SPI CRC check in DW3000
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] crc_mode: If set to @ref DWT_SPI_CRC_MODE_WR then SPI CRC checking will be performed in DW3000 on each SPI write
 *                   last byte of the SPI write transaction needs to be the 8-bit CRC, if it does not match
 *                   the one calculated by DW3000 SPI CRC ERROR event will be set in the status register (SYS_STATUS_SPICRC)
 * @param[in] spireaderr_cb: This needs to contain the callback function pointer which will be called when SPI read error
 *                        is detected (when the DW3000 generated CRC does not match the one calculated by  dwt_generatecrc8
 *                        following the SPI read transaction)
 *
 * @return None
 */
static void ull_enablespicrccheck(dwchip_t *dw, dwt_spi_crc_mode_e crc_mode, dwt_spierrcb_t spireaderr_cb)
{
    // enable CRC check in DW3000
    if (crc_mode != DWT_SPI_CRC_MODE_NO)
    {
        dwt_or8bitoffsetreg(dw, SYS_CFG_ID, 0U, SYS_CFG_SPI_CRC_BIT_MASK);

        // enable CRC generation on SPI read transaction which the DW3000 will store in SPICRC_CFG_ID register
        if (crc_mode == DWT_SPI_CRC_MODE_WRRD)
        {
            dw->callbacks.cbSPIRDErr = spireaderr_cb;
        }
    }
    else
    {
        dwt_and8bitoffsetreg(dw, SYS_CFG_ID, 0U, ~(uint8_t)SYS_CFG_SPI_CRC_BIT_MASK);
    }
    LOCAL_DATA(dw)->spicrc = crc_mode;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function is a static function used to 'kick' the LDO bias upon wakeup from sleep. It will load the
 *        required LDO bias configuration from OTP memory.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return None
 */
static void dwt_prog_ldo_and_bias_tune(dwchip_t *dw)
{
    dwt_or16bitoffsetreg(dw, OTP_CFG_ID, 0U, LDO_BIAS_KICK);
    dwt_and_or16bitoffsetreg(dw, BIAS_CTRL_ID, 0U, (uint16_t)~BIAS_CTRL_BIAS_BIT_MASK, LOCAL_DATA(dw)->bias_tune);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function is a static function used to 'kick' the desired operating parameter set (OPS) table upon wakeup from sleep.
 *        It will load the required OPS table configuration based upon what OPS table was set to be used in dwt_configure().
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return None
 */
static void dwt_kick_ops_table_on_wakeup(dwchip_t *dw)
{
    /* Restore OPS table config and kick. */
    /* Correct sleep mode should be set by dwt_configure() */

    /* Using the mask of all available OPS table options, check for OPS table options in the sleep mode mask */
    switch (LOCAL_DATA(dw)->sleep_mode & ((uint16_t)DWT_ALT_OPS | (uint16_t)DWT_SEL_OPS0 | (uint16_t)DWT_SEL_OPS1 | (uint16_t)DWT_SEL_OPS2 | (uint16_t)DWT_SEL_OPS3))
    {
    /* If preamble length >= 256 and set by dwt_configure(), the OPS table should be kicked off like so upon wakeup. */
    case ((uint16_t)DWT_ALT_OPS | (uint16_t)DWT_SEL_OPS0):
        dwt_modify32bitoffsetreg(dw, OTP_CFG_ID, 0U, ~(uint32_t)(OTP_CFG_OPS_ID_BIT_MASK), DWT_OPSET_LONG | OTP_CFG_OPS_KICK_BIT_MASK);
        break;
        /* If SCP mode is enabled by dwt_configure(), the OPS table should be kicked off like so upon wakeup. */
    case ((uint16_t)DWT_ALT_OPS | (uint16_t)DWT_SEL_OPS1):
        dwt_modify32bitoffsetreg(dw, OTP_CFG_ID, 0U, ~(uint32_t)(OTP_CFG_OPS_ID_BIT_MASK), DWT_OPSET_SCP | OTP_CFG_OPS_KICK_BIT_MASK);
        break;
    case ((uint16_t)DWT_ALT_OPS | (uint16_t)DWT_SEL_OPS2): // Short OPS table - set to be loaded as default
        dwt_modify32bitoffsetreg(dw, OTP_CFG_ID, 0U, ~(uint32_t)(OTP_CFG_OPS_ID_BIT_MASK), DWT_OPSET_SHORT | OTP_CFG_OPS_KICK_BIT_MASK);
        break;
    default:
        // Do nothing
        break;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function is a static function used to 'kick' the DGC upon wakeup from sleep. It will load the
 *        required DGC configuration from OTP memory based upon what channel was set, e.g., configured with dwt_configure().
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param channel: Specifies the operating channel (e.g. 5 or 9)
 *
 * @return None
 */
static void dwt_kick_dgc_on_wakeup(dwchip_t *dw, int8_t channel)
{
    /* The DGC_SEL bit must be set to '0' for channel 5 and '1' for channel 9 */
    if (channel == 5)
    {
        dwt_modify32bitoffsetreg(dw, OTP_CFG_ID, 0U, ~(uint32_t)(OTP_CFG_DGC_SEL_BIT_MASK), ((uint32_t)DWT_DGC_SEL_CH5 << OTP_CFG_DGC_SEL_BIT_OFFSET) | OTP_CFG_DGC_KICK_BIT_MASK);
    }
    else if (channel == 9)
    {
        dwt_modify32bitoffsetreg(dw, OTP_CFG_ID, 0U, ~(uint32_t)(OTP_CFG_DGC_SEL_BIT_MASK), ((uint32_t)DWT_DGC_SEL_CH9 << OTP_CFG_DGC_SEL_BIT_OFFSET) | OTP_CFG_DGC_KICK_BIT_MASK);
    }
    else
    {
        // Do nothing
    }
}

static void dwt_localstruct_init(dwt_local_data_t *data)
{
    data->dblbuffon = (uint8_t)DBL_BUFF_OFF; // Double buffer mode off by default / clear the flag
    data->sleep_mode = 0U; // No need to set RUN_SAR on wake as PGF cal is done by software in dwt_restore_txrx()
    data->spicrc = DWT_SPI_CRC_MODE_NO;
    data->stsconfig = (uint8_t)DWT_STS_MODE_OFF; // STS off
    data->channel = 0U;
    data->temperature = TEMP_INIT;
    data->vdddig_otp = 0U;
    data->vdddig_current = 0U;
    data->sys_cfg_dis_fce_bit_flag = 0U;
    data->otp_ldo_tune_lo = 0UL;
    data->coarse_code_pll_cal_ch5 = 0U;
    data->coarse_code_pll_cal_ch9 = 0U;
    data->pll_bias_trim = 0U;
    data->pdoaMode = DWT_PDOA_M0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function is a static function which is used to configure a specific VDDDIG value (86 mV, 88 mV or 93 mV).
 * It is required to have a reference VDDDIG value of 86 mV in the device's OTP memory to ensure the voltage is correctly scaled
 * relative to this reference value.
 *
 * If the OTP is not programmed (i.e., LOCAL_DATA(dw)->vdddig_otp was not set during initialisaton)
 * the API will return an error.
 *
 * If OTP is programmed, the API can still return an @ref DWT_ERROR if the target setting cannot be achieved relative to the reference level.
 * This should never occur on DW3000 devices as mass production data shows reference VDDDIG is within 0x5 and 0xD. Within this range, the function
 * shall always find a solution and the error check is for software protection / fault detection.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
 */
static int32_t ull_set_vdddig_mv(dwchip_t *dw,  dwt_vdddig_mv_t vdddig)
{
    int32_t retVal = (int32_t)DWT_SUCCESS;
    if(LOCAL_DATA(dw)->vdddig_otp == 0U)
    {
        // OTP is not provisionned. AON register default value = 0xC was configured at boot.
        // We cannot configure voltage accurately for this part
        return (int32_t)DWT_ERROR;
    }

    uint8_t vdddig_coarse = (LOCAL_DATA(dw)->vdddig_otp & 0x30U) >> 4U;
    uint8_t vdddig_fine = (LOCAL_DATA(dw)->vdddig_otp & 0x0FU);

    switch (vdddig)
    {
        case VDDDIG_86mV:
            break;

        case VDDDIG_88mV:
            if(vdddig_fine <= 13U) // Should always be true as OTP setting distribution is 0x5 to 0xD
            {
                vdddig_fine += 2U;
            }
            else if (vdddig_coarse != 3U) // Safety fallback
            {
                vdddig_coarse += 1U;
                vdddig_fine -= 8U;
            }
            else {
                retVal = (int32_t)DWT_ERROR;
            }
            break;

        case VDDDIG_93mV:
            if(vdddig_fine >= 3U && vdddig_coarse != 3U) // Should always be true as vdddig setting distribution is 0x5 to 0xD
            {
                vdddig_coarse += 1U;
                vdddig_fine -= 3U;
            }
            else if (vdddig_fine < 3U)
            {
                vdddig_fine += 7U;
            }
            else{
                retVal = (int32_t)DWT_ERROR;
            }
            break;

        default:
            retVal = (int32_t)DWT_ERROR;
            break;
    }
    
    if(retVal == (int32_t)DWT_SUCCESS)
    {
        LOCAL_DATA(dw)->vdddig_current = vdddig_coarse << 4U | vdddig_fine;

        dwt_aon_write((uint16_t)AON_VDD_DIG, LOCAL_DATA(dw)->vdddig_current);
    }
    
    return retVal;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function initialises the DW3000 transceiver:
 * it reads its DEV_ID register (address 0x00) to verify the IC is one supported
 * by this software (e.g. DW3000 32-bit device ID value is 0xDECA03xx).  Then it
 * does any initial once only device configurations needed for use and initialises
 * as necessary any static data items belonging to this low-level driver.
 *
 * @note
 * 1. This function needs to be run before dwt_configuresleep(), also the SPI frequency has to be < 7MHz.
 * 2. It also reads and applies LDO and BIAS tune and crystal trim values from OTP memory
 * 3. It is assumed this function is called after a reset or on power up of the DW3000
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] mode: A bit mask to alter the default initialisation flow.
 *                  - @ref DWT_READ_OTP_ALL        - reads Part ID, Lot ID, reference battery voltage and temprature from OTP.
 *                  - @ref DWT_READ_OTP_PLID_DIS   - don't read part ID or lot ID from OTP
 *                  - @ref DWT_READ_OTP_VTBAT_DIS  - don't read reference battery voltage from OTP 
 *                  - @ref DWT_READ_OTP_TMP_DIS    - don't read reference temperature from OTP
 * 
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
 */
static int32_t ull_initialise(dwchip_t *dw, int32_t mode)
{
    uint32_t ldo_tune_lo;
    uint32_t ldo_tune_hi;
    uint32_t pll_coarse_code;

    dwt_local_data_t *pdw3000local = &(dw->priv);

    dwt_localstruct_init(pdw3000local);

    pdw3000local->vBatP = 0U;
    pdw3000local->tempP = 0U;

    // Read LDO_TUNE and BIAS_TUNE from OTP
    ldo_tune_lo = dwt_otpreadword32(dw, LDOTUNELO_ADDRESS);
    ldo_tune_hi = dwt_otpreadword32(dw, LDOTUNEHI_ADDRESS);
    pdw3000local->bias_tune = (uint8_t)((dwt_otpreadword32(dw, BIAS_TUNE_ADDRESS) >> 16UL) & BIAS_CTRL_BIAS_BIT_MASK);

    // Store it for later use
    pdw3000local->otp_ldo_tune_lo = ldo_tune_lo;

    // Saving VDDDIG value from OTP in chip local context to prevent future OTP reading
    // OTP contains trim code value only. Coarse set to 0 by default.
    uint8_t otp_vdddig = (uint8_t)((((ldo_tune_hi & LDO_TUNE_HI_VDDDIG_COARSE_MASK) >> LDO_TUNE_HI_VDDDIG_COARSE_OFFSET) << 4UL) | ((ldo_tune_hi & LDO_TUNE_HI_VDDDIG_TRIM_MASK) >> LDO_TUNE_HI_VDDDIG_TRIM_OFFSET));

    if(otp_vdddig != 0U)
    {
        pdw3000local->vdddig_otp = otp_vdddig;
    }
    else
    {
        pdw3000local->vdddig_current = dwt_aon_read((uint16_t)AON_VDD_DIG);
    }

    if ((ldo_tune_lo != 0UL) && (ldo_tune_hi != 0UL) && (pdw3000local->bias_tune != 0U))
    {
        dwt_prog_ldo_and_bias_tune(dw);
    }

    (void)ull_set_vdddig_mv(dw, VDDDIG_88mV);

    // Read DGC_CFG from OTP
    if (dwt_otpreadword32(dw, DGC_TUNE_ADDRESS) == DWT_DGC_CFG0)
    {
        pdw3000local->dgc_otp_set = DWT_DGC_LOAD_FROM_OTP;
    }
    else
    {
        pdw3000local->dgc_otp_set = DWT_DGC_LOAD_FROM_SW;
    }

    // Load Part and Lot ID from OTP
    if (((uint8_t)mode & (uint8_t)DWT_READ_OTP_PLID_DIS) == 0U)
    {
        pdw3000local->partID = dwt_otpreadword32(dw, PARTID_ADDRESS);
        uint32_t lot_id[2];
        lot_id[0] = dwt_otpreadword32(dw, WSLOTID_LOW_ADDRESS);
        lot_id[1] = dwt_otpreadword32(dw, WSLOTID_HIGH_ADDRESS);
        pdw3000local->lotID = ((uint64_t)lot_id[1] << 32) | lot_id[0];
    }

    if (((uint8_t)mode & (uint8_t)DWT_READ_OTP_VTBAT_DIS) == 0U)
    {
        // We save three voltage levels in OTP during production testing:
        // [7:0] = Vbat @ 1.62V
        // [15:8] = Vbat @ 3.6V
        // [23:16] = Vbat @ 3.0V
        pdw3000local->vBatP = (uint8_t)(dwt_otpreadword32(dw, VBAT_ADDRESS) >> 16UL);
    }

    if (((uint8_t)mode & (uint8_t)DWT_READ_OTP_TMP_DIS) == 0U)
    {
        pdw3000local->tempP = (uint8_t)dwt_otpreadword32(dw, VTEMP_ADDRESS);
    }

    // if the reference temperature has not been programmed in OTP (early eng samples) set to default value
    if (pdw3000local->tempP == 0U)
    {
        pdw3000local->tempP = 0x85U; //@temp of 22 deg
    }

    // if the reference voltage has not been programmed in OTP (early eng samples) set to default value
    if (pdw3000local->vBatP == 0U)
    {
        pdw3000local->vBatP = 0x74U; //@Vref of 3.0V
    }

    pdw3000local->otprev = (uint8_t)dwt_otpreadword32(dw, OTPREV_ADDRESS);

    pdw3000local->init_xtrim = (uint8_t)dwt_otpreadword32(dw, XTRIM_ADDRESS) & XTAL_TRIM_BIT_MASK;
    if (pdw3000local->init_xtrim == 0U)
    {
        // set the default value
        pdw3000local->init_xtrim = DEFAULT_XTAL_TRIM;
    }
    dwt_write8bitoffsetreg(dw, XTAL_ID, 0U, pdw3000local->init_xtrim);

    pll_coarse_code = dwt_otpreadword32(dw, PLL_CC_ADDRESS);
    if (pll_coarse_code != 0UL)
    {
        dwt_write32bitoffsetreg(dw, PLL_COARSE_CODE_ID, 0U, pll_coarse_code);
        // PLL_COARSE_CODE = [24] Ch9 RVCO Frequency Boost + [21:8] Ch5 coarse code (Test 8180) + [4:0] Ch9 coarse code (Test 8550)
        // Coarse code for CH9
        pdw3000local->coarse_code_pll_cal_ch9 = pll_coarse_code & PLL_COARSE_CODE_CH9_RVCO_FREQ_BOOST_BIT_MASK;  // [24]
        pdw3000local->coarse_code_pll_cal_ch9 >>= (PLL_COARSE_CODE_CH9_RVCO_FREQ_BOOST_BIT_OFFSET - (PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_OFFSET + PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_LEN));
        pdw3000local->coarse_code_pll_cal_ch9 += pll_coarse_code & PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_MASK;  // [4:0]
        // Coarse code for CH5
        pdw3000local->coarse_code_pll_cal_ch5 = (pll_coarse_code & PLL_COARSE_CODE_CH5_VCO_COARSE_TUNE_BIT_MASK) >> 8UL;  // [21:8]
    }
    else
    {
        // If the PLL coarse code is not programmed in OTP, set the default value
        pdw3000local->coarse_code_pll_cal_ch5 = DEFAULT_PLL_VTUNE_CODE_CH5;
        pdw3000local->coarse_code_pll_cal_ch9 = DEFAULT_PLL_VTUNE_CODE_CH9;
    }

    // Set the temperature of the device so PLL calibration can use it.
    if (pdw3000local->temperature == TEMP_INIT)
    {
        uint16_t tempvbat = ull_readtempvbat(dw);
        pdw3000local->temperature = (int8_t)ull_convertrawtemperature(dw, (uint8_t)(tempvbat >> 8U));  // Temperature in upper 8 bits
    }

    dwt_write32bitreg(dw, TX_CTRL_LO_ID, TX_CTRL_LO_DEF);

    pdw3000local->sys_cfg_dis_fce_bit_flag = ((dwt_read32bitreg(dw, SYS_CFG_ID) & SYS_CFG_DIS_FCE_BIT_MASK) != 0UL) ? 1U : 0U;

    return (int32_t)DWT_SUCCESS;
} // end ull_initialise()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function checks if PLL is locked or not.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] retries: Number of retries to check if PLL is locked.
 *
 * @returns @ref DWT_SUCCESS if PLL is locked otherwise @ref DWT_ERR_PLL_LOCK.
 */
static int32_t is_pll_locked(dwchip_t *dw, const uint8_t retries)
{
    // check if PLL is locked
    bool locked = (bool)((dwt_read8bitoffsetreg(dw, PLL_STATUS_ID, 0) & PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK) != 0U);

    // if retries is > 1, wait for 20 us and check lock flag again
    for (uint8_t cnt = 1; (cnt < retries) && (!locked); cnt++)
    {
        deca_usleep(DELAY_20uUSec);
        if ((dwt_read8bitoffsetreg(dw, PLL_STATUS_ID, 0) & PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK) != 0U)
        {
            // PLL is locked
            locked = true;
            break;
        }
    }

    return (locked) ? (int32_t)DWT_SUCCESS : (int32_t)DWT_ERR_PLL_LOCK;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function can place UWB IC into IDLE/IDLE_PLL or IDLE_RC mode when it is not actively in TX or RX.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] state: See @ref dwt_idle_init_modes_e
 * @parblock
 *                DWT_DW_IDLE (1) to put UWB IC into IDLE/IDLE_PLL state
 *                DWT_DW_INIT (0) to put UWB IC into INIT_RC state
 *                DWT_DE_IDLE_RC (2) to put UWB IC into IDLE_RC state
 * @endparblock
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERR_PLL_LOCK for error
 */
static int32_t ull_setdwstate(dwchip_t *dw, int32_t state)
{
    int32_t  ret = (int32_t) DWT_SUCCESS;
    uint8_t dw_state = dwt_read8bitoffsetreg(dw, SYS_STATE_LO_ID, 2U);

    // If the current state is the same as the requested state, return success without changing anything
    bool state_is_same = (((dw_state ==  DW_SYS_STATE_INIT) && (state ==(int32_t) DWT_DW_INIT)) ||
                            ((dw_state ==  DW_SYS_STATE_IDLE) && (state == (int32_t)DWT_DW_IDLE)) ||
                            ((dw_state ==  DW_SYS_STATE_IDLE_RC) && (state == (int32_t)DWT_DW_IDLE_RC)));
    
    if (state_is_same)
    {
        return (int32_t) DWT_SUCCESS;
    }

    // Check if UWB radio is in TX or RX state, if true return an error. User should call dwt_forcetrxoff() prior to changing state
    if (dw_state > DW_SYS_STATE_IDLE) 
    {
        return (int32_t) DWT_ERR_WRONG_STATE;
    }

    // Set the auto INIT2IDLE bit so that DW3000 enters IDLE mode before switching clocks to system_PLL
    if (state == (int32_t)DWT_DW_IDLE)
    // NOTE: PLL should be configured prior to this, and the device should be in IDLE_RC (if the PLL does not lock device will remain in IDLE_RC)
    {
        // switch clock to auto - if coming here from INIT_RC the clock will be FOSC/4, need to switch to auto prior to setting auto INIT2IDLE bit
        ull_force_clocks(dw, FORCE_CLK_AUTO);
        dwt_or8bitoffsetreg(dw, SYS_STATUS_ID, 0, SYS_STATUS_CP_LOCK_BIT_MASK); 
        // Clear PLL lock status before calibrating PLL
        dwt_or8bitoffsetreg(dw, PLL_CAL_ID, 0x01, (uint8_t)(PLL_CAL_PLL_CAL_EN_BIT_MASK >> 8U));
        dwt_or8bitoffsetreg(dw, SEQ_CTRL_ID, 0x01, (uint8_t)(SEQ_CTRL_AINIT2IDLE_BIT_MASK >> 8U));
        ret = is_pll_locked(dw, MAX_RETRIES_FOR_PLL);
    }
    else if (state == (int32_t)DWT_DW_IDLE_RC) // Change state to IDLE_RC and clear auto INIT2IDLE bit
    {
        // switch clock to FOSC
        dwt_or8bitoffsetreg(dw, CLK_CTRL_ID, 0U, FORCE_SYSCLK_FOSC);
        // clear the auto INIT2IDLE bit and set FORCE2INIT
        dwt_modify32bitoffsetreg(dw, SEQ_CTRL_ID, 0x0U, ~(uint32_t)SEQ_CTRL_AINIT2IDLE_BIT_MASK, SEQ_CTRL_FORCE2INIT_BIT_MASK);
        // clear force bits (device will stay in IDLE_RC)
        dwt_and8bitoffsetreg(dw, SEQ_CTRL_ID, 0x2U, (uint8_t)~(SEQ_CTRL_FORCE2INIT_BIT_MASK >> 16UL));
        // switch clock to auto
        ull_force_clocks(dw, FORCE_CLK_AUTO);
    }
    else
    // NOTE: the SPI rate needs to be <= 7MHz as device is switching to INIT_RC state
    {
        dwt_or8bitoffsetreg(dw, CLK_CTRL_ID, 0U, FORCE_SYSCLK_FOSCDIV4);
        // clear the auto INIT2IDLE bit and set FORCE2INIT
        dwt_modify32bitoffsetreg(dw, SEQ_CTRL_ID, 0x0U, ~(uint32_t)SEQ_CTRL_AINIT2IDLE_BIT_MASK, SEQ_CTRL_FORCE2INIT_BIT_MASK);
        dwt_and8bitoffsetreg(dw, SEQ_CTRL_ID, 0x2U, (uint8_t)~(SEQ_CTRL_FORCE2INIT_BIT_MASK >> 16UL));
    }
    return ret;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to enable GPIO clocks. The clocks are needed to ensure correct GPIO operation
 *
 * @param dw - DW3000 chip descriptor handler.
 *
 * @return  None
 */
static void ull_enablegpioclocks(dwchip_t *dw)
{
    dwt_or32bitoffsetreg(dw, CLK_CTRL_ID, 0U, CLK_CTRL_GPIO_CLK_EN_BIT_MASK);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to configure GPIO mode
 *
 * input parameters
 * @param dw        - DW3000 chip descriptor handler.
 * @param[in] gpio_mask: The full mask of all of the GPIOs for which to change the mode. Typically built from @ref dwt_gpio_mask_e values.
 * @param[in] gpio_modes: The GPIO modes to set. Typically built from @ref dwt_gpio_pin_e values.
 *
 *
 * @return  None
 */
static void ull_setgpiomode(dwchip_t *dw, uint32_t gpio_mask, uint32_t gpio_modes)
{
    uint32_t mask = 0UL;
    for (uint32_t i = 0UL; i <= 8UL; i++)
    {
        if ((gpio_mask & (1UL << i)) != 0UL)
        {
            mask |= (GPIO_MFIO_MODE_MASK << (3UL * i));
        }
    }

    dwt_and_or32bitoffsetreg(dw, GPIO_MODE_ID, 0U, ~mask, mask & gpio_modes);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to configure the GPIOs as inputs or outputs. All are set to input (i.e. 1) by default.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] in_out: If corresponding GPIO bit is set to 1 then it is input, otherwise it is set as an output,
 *                    GPIO 0 = bit 0, GPIO 1 = bit 1 etc...
 *
 * @return  None
 */
static void ull_setgpiodir(dwchip_t *dw, uint16_t in_out)
{
    /*Set GPIOs direction*/
    dwt_write16bitoffsetreg(dw, GPIO_DIR_ID, 0U, in_out);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the current GPIO configuration. All GPIOs are set to input (i.e. 1) by default.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] in_out: Will return the current GPIO configuration, if GPIO is an input the bit will be set to 1, otherwise 0 means the GPIO is configured as an output,
 *                    GPIO 0 = bit 0, GPIO 1 = bit 1 etc...
 *
 * @return  None
 */
static void ull_getgpiodir(dwchip_t *dw, uint16_t* in_out)
{
    /*Get GPIOs direction*/
    *in_out = dwt_read16bitoffsetreg(dw, GPIO_DIR_ID, 0U);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to set output value on GPIOs that have been configured for output via dwt_setgpiodir() API
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] gpio: Should be one or multiple of @ref dwt_gpio_mask_e values
 * @param[in] value: Logic value for GPIO or GPIOs if multiple set at same time.
 *
 * @return  None
 */
static void ull_setgpiovalue(dwchip_t *dw, uint16_t gpio_mask, int32_t value)
{
    /* Set output level for output pin to high. */
    if (value == 1)
    {
        dwt_or16bitoffsetreg(dw, GPIO_OUT_ID, 0U, gpio_mask);
    }
    /* Set output level for output pin to low. */
    else
    {
        dwt_and16bitoffsetreg(dw, GPIO_OUT_ID, 0U, ~gpio_mask);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the raw value of the GPIO pins.
 *        It is presumed that functions such as dwt_setgpiomode(), dwt_setgpiovalue() and dwt_setgpiodir() are called before this function.
 *        GPIO 0 = bit 0, GPIO 1 = bit 1 etc...
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return a value that holds the value read on the GPIO pins.
 */
static uint16_t ull_readgpiovalue(dwchip_t *dw)
{
    return dwt_read16bitoffsetreg(dw, GPIO_RAW_ID, 0U);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to enable GPIO for external LNA or PA functionality. HW dependent, consult the DW3xxx User Manual.
 *        This can also be used for debug as enabling TX and RX GPIOs is quite handy to monitor DW3xxx TRX activity.
 *
 * @note Enabling PA functionality requires that fine grain TX sequencing is deactivated. This can be done using
 *       dwt_setfinegraintxseq().
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] lna_pa: Bit field (only bits 0 and 1 are used).
 *                    Bit 0 if set will enable LNA functionality.
 *                    Bit 1 if set will enable PA functionality.
 *                    To disable LNA/PA set all the bits to 0.
 *
 * @return  None
 */
static void ull_setlnapamode(dwchip_t *dw, int32_t lna_pa)
{
    uint32_t gpio_mode = dwt_read32bitreg(dw, GPIO_MODE_ID);
    // clear GPIO 4, 5, 6, configuration
    gpio_mode &= (~(GPIO_MODE_MSGP0_MODE_BIT_MASK | GPIO_MODE_MSGP1_MODE_BIT_MASK | GPIO_MODE_MSGP4_MODE_BIT_MASK | GPIO_MODE_MSGP5_MODE_BIT_MASK
                    | GPIO_MODE_MSGP6_MODE_BIT_MASK));
    if (((uint32_t)lna_pa & (uint32_t)DWT_LNA_ENABLE) != 0UL)
    {
        gpio_mode |= (uint32_t)DW3000_GPIO_PIN6_EXTRXE;
    }

    if (((uint32_t)lna_pa & (uint32_t)DWT_PA_ENABLE) != 0UL)
    {
        gpio_mode |= ((uint32_t)DW3000_GPIO_PIN4_EXTPA | (uint32_t)DW3000_GPIO_PIN5_EXTTXE);
    }

    if (((uint32_t)lna_pa & (uint32_t)DWT_TXRX_EN) != 0UL)
    {
        gpio_mode |= ((uint32_t)DW3000_GPIO_PIN0_PDOA_SW_TX | (uint32_t)DW3000_GPIO_PIN1_PDOA_SW_RX);
    }

    dwt_write32bitreg(dw, GPIO_MODE_ID, gpio_mode);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read OTP revision
 *
 * @note dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return the read OTP revision value
 */
static uint8_t ull_otprevision(dwchip_t *dw)
{
    return LOCAL_DATA(dw)->otprev;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Returns the PG delay value of the TX
 *
 * input parameters
 * @param dw - DW3000 chip descriptor handler.
 *
 * @return PG delay (6-bit value)
 */
static uint8_t ull_readpgdelay(dwchip_t *dw)
{
    return (dwt_read8bitoffsetreg(dw, TX_CTRL_HI_ID, 0U) & TX_CTRL_HI_TX_PG_DELAY_BIT_MASK);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read voltage (Vbat) value measured @ 3.0 V and recorded in OTP address 0x8 (VBAT_ADDRESS)
 *
 * @note dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return the 8-bit Vbat value as programmed into the OTP memory in the factory
 */
static uint8_t ull_geticrefvolt(dwchip_t *dw)
{
    return LOCAL_DATA(dw)->vBatP;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read temperature (Vtemp) value measured @ 22 C and recorded in OTP address 0x9 (VTEMP_ADDRESS)
 *
 * @note dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return the 8-bit Vtemp value as programmed into the OTP memory in the factory
 */
static uint8_t ull_geticreftemp(dwchip_t *dw)
{
    return LOCAL_DATA(dw)->tempP;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read part ID of the device
 *
 * @note dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return the 32-bit part ID value as programmed into the OTP memory in the factory
 */
static uint32_t ull_getpartid(dwchip_t *dw)
{
    return LOCAL_DATA(dw)->partID;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read lot ID of the device
 *
 * @note dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return the 64-bit lot ID value as programmed into the OTP memory in the factory
 */
static uint64_t ull_getlotid(dwchip_t *dw)
{
    return LOCAL_DATA(dw)->lotID;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function enables/disables the fine grain TX sequencing (enabled by default).
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] enable: Set to 1 to enable fine grain TX sequencing, 0 to disable it.
 *
 * @return  None
 */
static void ull_setfinegraintxseq(dwchip_t *dw, int32_t enable)
{
    if (enable != 0)
    {
        dwt_write32bitoffsetreg(dw, PWR_UP_TIMES_TXFINESEQ_ID, 2U, PMSC_TXFINESEQ_ENABLE);
    }
    else
    {
        dwt_write32bitoffsetreg(dw, PWR_UP_TIMES_TXFINESEQ_ID, 2U, PMSC_TXFINESEQ_DISABLE);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function provides the API for the configuration of the TX power
 * The input is the desired TX power to set.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] power: TX power to configure, 32-bits.
 * @parblock
 * TX_POWER register contains:
 * [31:24]     TX_CP_PWR (STS power)
 * [23:16]     TX_SHR_PWR
 * [15:8 ]     TX_PHR_PWR
 * [7:0]       TX_DATA_PWR
 * @endparblock
 * @note The TX power can also be cofigured with PGdly and PGcount See @ref dwt_txconfig_t via dwt_configuretxrf()
 *
 * @return  None
 */
static void ull_settxpower(dwchip_t *dw, uint32_t power)
{
    dwt_write32bitreg(dw, TX_POWER_ID, power);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function provides the API for the configuration of the TX spectrum
 * including the power and pulse generator delay. The input is a pointer to the data structure
 * of type @ref dwt_txconfig_t that holds all the configurable items.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] config: Pointer to the @ref dwt_txconfig_t configuration structure, which contains the TX RF config data
 * @note If config->PGcount != 0 the the PG calibration (bandwith calibration) will run,
 *       otherwise PGdelay value will be used to set the bandwidth.
 *
 * @return  None
 */
static void ull_configuretxrf(dwchip_t *dw, dwt_txconfig_t *config)
{
    if (config->PGcount == 0U)
    {
        // Configure RF TX PG_DELAY
        dwt_write8bitoffsetreg(dw, TX_CTRL_HI_ID, 0U, config->PGdly);
    }
    else
    {
        (void)ull_calcbandwidthadj(dw, config->PGcount);
    }

    // Configure TX power
    dwt_write32bitreg(dw, TX_POWER_ID, config->power);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function sets the default values of the lookup tables depending on the channel selected.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] channel: Channel that the device will be receiving on. This is a receiver tuning configuration.
 *
 * @return  None
 */
static void ull_configmrxlut(dwchip_t *dw, int32_t channel)
{
    uint32_t lut0, lut1, lut2, lut3, lut4, lut5, lut6;

    if (channel == 5)
    {
        lut0 = (uint32_t)CH5_DGC_LUT_0;
        lut1 = (uint32_t)CH5_DGC_LUT_1;
        lut2 = (uint32_t)CH5_DGC_LUT_2;
        lut3 = (uint32_t)CH5_DGC_LUT_3;
        lut4 = (uint32_t)CH5_DGC_LUT_4;
        lut5 = (uint32_t)CH5_DGC_LUT_5;
        lut6 = (uint32_t)CH5_DGC_LUT_6;
    }
    else
    {
        lut0 = (uint32_t)CH9_DGC_LUT_0;
        lut1 = (uint32_t)CH9_DGC_LUT_1;
        lut2 = (uint32_t)CH9_DGC_LUT_2;
        lut3 = (uint32_t)CH9_DGC_LUT_3;
        lut4 = (uint32_t)CH9_DGC_LUT_4;
        lut5 = (uint32_t)CH9_DGC_LUT_5;
        lut6 = (uint32_t)CH9_DGC_LUT_6;
    }

    dwt_write32bitoffsetreg(dw, DGC_LUT_0_CFG_ID, 0x0U, lut0);
    dwt_write32bitoffsetreg(dw, DGC_LUT_1_CFG_ID, 0x0U, lut1);
    dwt_write32bitoffsetreg(dw, DGC_LUT_2_CFG_ID, 0x0U, lut2);
    dwt_write32bitoffsetreg(dw, DGC_LUT_3_CFG_ID, 0x0U, lut3);
    dwt_write32bitoffsetreg(dw, DGC_LUT_4_CFG_ID, 0x0U, lut4);
    dwt_write32bitoffsetreg(dw, DGC_LUT_5_CFG_ID, 0x0U, lut5);
    dwt_write32bitoffsetreg(dw, DGC_LUT_6_CFG_ID, 0x0U, lut6);
    dwt_write32bitoffsetreg(dw, DGC_CFG0_ID, 0x0U, DWT_DGC_CFG0);
    dwt_write32bitoffsetreg(dw, DGC_CFG1_ID, 0x0U, DWT_DGC_CFG1);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function configures the STS AES 128-bit KEY value.
 * The default value is [31:00]0xc9a375fa,
 *                      [63:32]0x8df43a20,
 *                      [95:64]0xb5e5a4ed,
 *                     [127:96]0x0738123b
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] pStsKey: The pointer to the structure of @ref dwt_sts_cp_key_t type, which holds the 128-bit AES KEY value to generate STS
 *
 * @return  None
 */
static void ull_configurestskey(dwchip_t *dw, dwt_sts_cp_key_t *pStsKey)
{
    dwt_write32bitreg(dw, STS_KEY0_ID, pStsKey->key0);
    dwt_write32bitreg(dw, STS_KEY1_ID, pStsKey->key1);
    dwt_write32bitreg(dw, STS_KEY2_ID, pStsKey->key2);
    dwt_write32bitreg(dw, STS_KEY3_ID, pStsKey->key3);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function configures the STS AES 128-bit initial value (IV). The default value is 1, i.e., UWB IC reset value is 1.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] pStsIv: The pointer to the structure of @ref dwt_sts_cp_iv_t type, which holds the IV value to generate STS
 *
 * @return  None
 */
static void ull_configurestsiv(dwchip_t *dw, dwt_sts_cp_iv_t *pStsIv)
{
    dwt_write32bitreg(dw, STS_IV0_ID, pStsIv->iv0);
    dwt_write32bitreg(dw, STS_IV1_ID, pStsIv->iv1);
    dwt_write32bitreg(dw, STS_IV2_ID, pStsIv->iv2);
    dwt_write32bitreg(dw, STS_IV3_ID, pStsIv->iv3);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function re-loads the STS AES initial value (IV)
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return  None
 */
static void ull_configurestsloadiv(dwchip_t *dw)
{
    dwt_or8bitoffsetreg(dw, STS_CTRL_ID, 0U, STS_CTRL_LOAD_IV_BIT_MASK);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function approximates the formula used to calculate the STS Minimum Threshold (STS_MNTH).The threshold
 * needs to be adjusted for various STS lengths. The formula is STS_MNTH = SQRT(X/Y)*default_STS_MNTH
 *
 * The API does not use the formula and the STS_MNTH value is derived from approximation formula as given by get_sts_mnth()
 *
 * @param[in] len_factor: The STS len factor given by @ref sts_length_factors[]
 * @param[in] threshold: The CIA lower bound threshold values for 64 MHz PRF, @ref CIA_MANUALLOWERBOUND_TH
 * @param[in] shift_val: This either 3 or 4, 3 for PDOA mode 1 or without PDOA (@ref DWT_PDOA_M0 or DWT_PDOA_M1) and 4 otherwise
 *
 */
static uint16_t get_sts_mnth(uint16_t len_factor, uint8_t threshold, uint8_t shift_val)
{
    uint32_t value;
    uint16_t mod_val;

    value = len_factor * (uint32_t)threshold;
    if (shift_val == 3U)
    {
        value *= SQRT_FACTOR; // Factor to sqrt(2)
        value >>= SQRT_SHIFT_VAL;
    }

    mod_val = (uint16_t)(value % MOD_VALUE + HALF_MOD);
    value >>= SHIFT_VALUE;
    /* Check if modulo greater than MOD_VALUE, if yes add 1 */
    if (mod_val >= MOD_VALUE)
    {
        value += 1U;
    }

    return (uint16_t)value;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function configures the STS Minimum Threshold based on the STS length and PDOA mode.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] pdoaMode: The PDOA mode (@ref dwt_pdoa_mode_e)
 *
 */
static void config_sts_mnth(dwchip_t *dw, dwt_pdoa_mode_e pdoaMode)
{
    if (LOCAL_DATA(dw)->stsconfig != (uint8_t)DWT_STS_MODE_OFF)
    {
        uint32_t sts_mnth;
        // configure CIA STS lower bound
        int32_t sts_len_idx = (int32_t) GET_STS_LEN_IDX(LOCAL_DATA(dw)->stsLength);

        if((uint8_t)sts_len_idx < STS_LEN_SUPPORTED)
        {
            if ((pdoaMode == DWT_PDOA_M1) || (pdoaMode == DWT_PDOA_M0))
            {
                // In PDOA mode 1, number of accumulated symbols is the whole length of the STS
                sts_mnth = (uint32_t) get_sts_mnth(sts_length_factors[(uint8_t)sts_len_idx], CIA_MANUALLOWERBOUND_TH, 3U);
            }
            else
            {
                // In PDOA mode 3 number of accumulated symbols is half of the length of STS symbols
                sts_mnth = (uint32_t) get_sts_mnth(sts_length_factors[(uint8_t)sts_len_idx], CIA_MANUALLOWERBOUND_TH, 4U);
            }
        }
        else
        {
            sts_mnth = CIA_MANUALLOWERBOUND_TH;
        }
        sts_mnth <<= STS_CONFIG_LO_STS_MAN_TH_BIT_OFFSET;
        sts_mnth &= STS_CONFIG_LO_STS_MAN_TH_BIT_MASK;

        dwt_modify32bitoffsetreg(dw, STS_CONFIG_LO_ID, 0U, ~(uint32_t)(STS_CONFIG_LO_STS_MAN_TH_BIT_MASK | STS_CONFIG_LO_STS_NTM_BIT_MASK), sts_mnth | STS_CONFIG_LO_NTM);
    }
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @deprecated This function is now deprecated. Will be removed in the future.
 *           Use ull_restore_common() and ull_restore_txrx() instead.
 * 
 * @brief This function needs to be called after device is woken up from DEEPSLEEP/SLEEP state. It will restore the
 * configuration which has not been automatically restored from AON
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] restore_mask: See @ref dwt_restore_type_e for more info on types of restore
 *
 * @return @ref DWT_SUCCESS or error (@ref dwt_error_e DWT_ERR_RX_CAL*) if PGF/ADC calibration returns an error.
 *
 */
static int32_t ull_restoreconfig(dwchip_t *dw, dwt_restore_type_e restore_mask)
{
    int32_t retVal = (int32_t)DWT_SUCCESS;

    ull_restore_common(dw);

    uint8_t rxtx_mask = (uint8_t)DWT_RESTORE_TXRX_MODE;
    (void)restore_mask;

    retVal = ull_restore_txrx(dw, rxtx_mask);

    return retVal;
}

/*!
 * This function needs to be called after device is woken up from DEEPSLEEP/SLEEP state, to restore the
 * configuration which has not been automatically restored from AON.
 * 
 * @param[in] dw: DW3000 chip descriptor handler.
 * 
 * @returns None
 */
static void ull_restore_common(dwchip_t *dw)
{
    // Restore/enable the OTP IPS for normal OTP use
    ull_dis_otp_ips(dw, 0);

    if (LOCAL_DATA(dw)->bias_tune != 0U)
    {
        dwt_prog_ldo_and_bias_tune(dw);
    }

    dwt_write8bitoffsetreg(dw, LDO_RLOAD_ID, 1U, LDO_RLOAD_VAL_B1);
}

/*!
 * This function is for internal use only, it's called by ull_restore_txrx() to restore PLL.
 * 
 * @param[in] dw: DW3000 chip descriptor handler.
 * 
 * @returns @ref DWT_SUCCESS if PLL is locked, otherwise @ref DWT_ERR_PLL_LOCK.
 */
static int32_t ull_restore_pll(dwchip_t *dw)
{
    int32_t retVal = (int32_t)DWT_SUCCESS;

    /*
        In DW3000, if PLL is already locked then it's necessary to re-run PLL cal 
        after kicking te LDO bias to prevent the PLL to be disturebed by new LDO bias settings.
        If PLL is not locked and we are in IDLE_RC, we just transition to IDLE_PLL.
    */ 
    if((dwt_read8bitoffsetreg(dw, PLL_STATUS_ID, 0) & PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK) != 0U)
    {
        // Check whether SPI_RDY is enabled, if it is then disable it before re-calibrating the PLL
        uint32_t spi_rdy_enable = ((uint32_t)dwt_read8bitoffsetreg(dw, SYS_ENABLE_LO_ID, 2U) << 16UL) & SYS_ENABLE_LO_SPIRDY_ENABLE_BIT_MASK;
        if(spi_rdy_enable != 0U)
        {
            // Disable SPI_RDY interrupt
            uint8_t sys_enable_spi_rdy_mask = (uint8_t)(SYS_ENABLE_LO_SPIRDY_ENABLE_BIT_MASK >> 16UL);
            dwt_and8bitoffsetreg(dw, SYS_ENABLE_LO_ID, 2U, ~sys_enable_spi_rdy_mask);
            retVal = ull_pll_cal(dw);
            // Clear SPI_RDY interrupt status
            uint8_t sys_status_spi_rdy_mask = (uint8_t)(SYS_STATUS_SPIRDY_BIT_MASK >> 16UL);
            dwt_write8bitoffsetreg(dw, SYS_STATUS_ID, 2U, sys_status_spi_rdy_mask);
            // Enable SPI_RDY interrupt
            dwt_or8bitoffsetreg(dw, SYS_ENABLE_LO_ID, 2U, sys_enable_spi_rdy_mask);
        }
        else
        {
            retVal = ull_pll_cal(dw);
        }
    }
    else
    {
        retVal = ull_setdwstate(dw, (int32_t)DWT_DW_IDLE);
    }
    
    return retVal;
}

/*!
 * This function needs to be called after device is woken up from DEEPSLEEP/SLEEP state, and afeter ull_restore_common().
 * It restores the TX and RX configuration which has not been automatically restored from AON.
 * If no TX or RX is needed, this function does not need to be called.
 * If only RX is needed, use DWT_RESTORE_RX_ONLY_MODE in restore_mask.
 * If only TX is needed, use DWT_RESTORE_TX_ONLY_MODE in restore_mask.
 * If both TX and RX are needed, use DWT_RESTORE_TXRX_MODE in restore_mask.
 * 
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] restore_mask: See @ref dwt_restore_type_e for more info on types of restore
 * 
 * @returns @ref DWT_SUCCESS if PLL is locked, otherwise @ref DWT_ERR_PLL_LOCK.
 */
static int32_t ull_restore_txrx(dwchip_t *dw, uint8_t restore_mask)
{
    int32_t retVal = (int32_t)DWT_SUCCESS;
    uint8_t channel = (uint8_t) DWT_CH5;
    uint16_t chan_ctrl;
    dwt_pll_prebuf_cfg_e pll_rx_prebuf_cfg = (dwt_pll_prebuf_cfg_e)LOCAL_DATA(dw)->pll_rx_prebuf_cfg;

    bool restore_rx, restore_tx;

    restore_rx = ((restore_mask & (uint8_t)DWT_RESTORE_RX_ONLY_MODE) == (uint8_t)DWT_RESTORE_RX_ONLY_MODE) || ((restore_mask & (uint8_t)DWT_RESTORE_TXRX_MODE) == (uint8_t)DWT_RESTORE_TXRX_MODE);
    restore_tx = ((restore_mask & (uint8_t)DWT_RESTORE_TX_ONLY_MODE) == (uint8_t)DWT_RESTORE_TX_ONLY_MODE) || ((restore_mask & (uint8_t)DWT_RESTORE_TXRX_MODE) == (uint8_t)DWT_RESTORE_TXRX_MODE);

    if(restore_rx || restore_tx)
    {
        chan_ctrl = dwt_read16bitoffsetreg(dw, CHAN_CTRL_ID, 0U);
        if ((chan_ctrl & 0x1U) != 0U)
        {
            channel = (uint8_t) DWT_CH9;
        }
        else
        {
            ull_increase_ch5_pll_ldo_tune(dw);
        }

        /* Restore RX/TX Pre-buffers Enable config*/
        if (pll_rx_prebuf_cfg != DWT_PLL_RX_PREBUF_DISABLE)
        {
            retVal = ull_setpllrxprebufen(dw, pll_rx_prebuf_cfg);
            if(retVal != (int32_t)DWT_SUCCESS)
            {
                return retVal;
            }
        }

        retVal = ull_restore_pll(dw);
        if(retVal != (int32_t)DWT_SUCCESS)
        {
            return retVal;
        }

        /*Restoring indirect access register B configuration as this is not preserved when device is in DEEPSLEEP/SLEEP state.
        * Indirect access register B is configured to point to the "Double buffer diagnostic SET 2"*/
        dwt_write32bitreg(dw, INDIRECT_ADDR_B_ID, (BUF1_RX_FINFO >> 16UL));
        dwt_write32bitreg(dw, ADDR_OFFSET_B_ID, (BUF1_RX_FINFO & 0xFFFFUL));

        /* Restore OPS table configuration */
        dwt_kick_ops_table_on_wakeup(dw);
    }

    if(restore_tx)
    {
        dwt_write32bitreg(dw, TX_CTRL_LO_ID, TX_CTRL_LO_DEF);
    }

    if(restore_rx)
    {
        // CIA diagnostic should be enabled when in DB mode, otherwise the data in diagnostic
        // SETs will be empty/invalid
        if ((LOCAL_DATA(dw)->cia_diagnostic >> 1U) == 0U)
        {
            dwt_write8bitoffsetreg(dw, RDB_DIAG_MODE_ID, 0U, (uint8_t)DW_CIA_DIAG_LOG_MIN >> 1U);
            LOCAL_DATA(dw)->cia_diagnostic |= (uint8_t)DW_CIA_DIAG_LOG_MIN;
        }
        else
        {
            dwt_write8bitoffsetreg(dw, RDB_DIAG_MODE_ID, 0U, LOCAL_DATA(dw)->cia_diagnostic >> 1U);
        }

        // assume RX code is the same as TX (e.g. we will not RX on 16 MHz or SCP and TX on 64 MHz)
        // only enable DGC for PRF 64
        if ((((chan_ctrl & (uint16_t)CHAN_CTRL_TX_PCODE_BIT_MASK) >> (uint16_t)CHAN_CTRL_TX_PCODE_BIT_OFFSET) >= 9U)
            && (((chan_ctrl & (uint16_t)CHAN_CTRL_TX_PCODE_BIT_MASK) >> (uint16_t)CHAN_CTRL_TX_PCODE_BIT_OFFSET) <= 24U))
        {
            /* If the OTP has DGC info programmed into it, do a manual kick from OTP. */
            if (LOCAL_DATA(dw)->dgc_otp_set == DWT_DGC_LOAD_FROM_OTP)
            {
                dwt_kick_dgc_on_wakeup(dw, (int8_t)channel);
            }
            /* Else we manually program hard-coded values into the DGC registers. */
            else
            {
                ull_configmrxlut(dw, (int32_t)channel);
            }
        }
        
        // Currently PGF is always calibrated when DWT_FAST_RESTORE is not requested
        retVal = ull_pgf_cal(dw, 1);
    }

    return retVal;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function configures STS mode: e.g., @ref DWT_STS_MODE_OFF, @ref DWT_STS_MODE_1 etc
 * The dwt_configure should be called prior to this to configure other parameters
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] stsMode: e.g., @ref DWT_STS_MODE_OFF, @ref DWT_STS_MODE_1 etc. See @ref dwt_sts_mode_e.
 *
 * @return  None
 *
 */
static void ull_configurestsmode(dwchip_t *dw, uint8_t stsMode)
{
    LOCAL_DATA(dw)->stsconfig = stsMode;

    dwt_modify16bitoffsetreg(dw, SYS_CFG_ID, 0U, (uint16_t)~(SYS_CFG_CP_SPC_BIT_MASK | SYS_CFG_CP_SDC_BIT_MASK),
        ((uint16_t)stsMode & (uint16_t)DWT_STS_CONFIG_MASK) << SYS_CFG_CP_SPC_BIT_OFFSET);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will configure the PDOA mode.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] pdoaMode: PDOA mode, see @ref dwt_pdoa_mode_e
 *
 * @note The dwt_configure() must be called prior to updating PDOA mode with this API. 
 *       To modify preamble length or STS length call dwt_configure() first or dwt_setstslength().
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error.
 */
static int32_t ull_setpdoamode(dwchip_t *dw, dwt_pdoa_mode_e pdoaMode)
{
    if ((pdoaMode != DWT_PDOA_M0) && (pdoaMode != DWT_PDOA_M1) && (pdoaMode != DWT_PDOA_M3))
    {
        return (int32_t)DWT_ERROR;
    }

    LOCAL_DATA(dw)->pdoaMode = pdoaMode;

    /* Set PDoA mode in SYS_CFG register. */
    dwt_modify8bitoffsetreg(dw, SYS_CFG_ID, 0x2U, ~((uint8_t)((uint32_t)SYS_CFG_PDOA_MODE_BIT_MASK >> 16U)), (uint8_t)pdoaMode);

    config_sts_mnth(dw, pdoaMode); //configure the STS Minimum Threshold
     
    return (int32_t)DWT_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function provides the main API for the configuration of the
 * UWB transceiver and this low-level driver. The input is a pointer to the data structure
 * of type @ref dwt_config_t that holds all the configurable items.
 * The @ref dwt_config_t structure shows which ones are supported.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] config: Pointer to the configuration structure @ref dwt_config_t, which contains the device configuration data.
 *
 * @note If the RX calibration routine fails the device receiver performance will be severely affected,
 * the application should reset device and try again
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR (e.g., when PLL CAL fails / PLL fails to lock)
 */
static int32_t ull_configure(dwchip_t *dw, dwt_config_t *config)
{
    uint8_t chan = config->chan;
    uint32_t temp;
    uint8_t scp = ((config->rxCode > 24U) || (config->txCode > 24U)) ? 1U : 0U;
    uint32_t mode = (config->phrMode == DWT_PHRMODE_EXT) ? SYS_CFG_PHR_MODE_BIT_MASK : 0UL;
    int32_t ret_val = (int32_t)DWT_SUCCESS;
#if DWT_DEBUG_PRINT
    printf("dwt_configure=PAC>%d:BR>%d:PC>%d:PL>%d:CH>%d:CPMode>%d:CPLen>%d:PDOA>%d\n", config->rxPAC, config->dataRate, config->rxCode, config->txPreambLength,
        config->chan, config->stsMode, config->stsLength, config->pdoaMode);
#endif

#ifdef DWT_API_ERROR_CHECK
    qassert((config->dataRate == DWT_BR_6M8) || (config->dataRate == DWT_BR_850K));
    qassert(config->rxPAC <= DWT_PAC4);
    qassert((chan == (uint8_t) DWT_CH5) || (chan == (uint8_t) DWT_CH9));
    qassert(CHECK_PREAMBLE_LEN_VALIDITY(config->txPreambLength));
    qassert((config->phrMode == DWT_PHRMODE_STD) || (config->phrMode == DWT_PHRMODE_EXT));
    qassert((config->phrRate == DWT_PHRRATE_STD) || (config->phrRate == DWT_PHRRATE_DTA));
    qassert((config->pdoaMode == DWT_PDOA_M0) || (config->pdoaMode == DWT_PDOA_M1) || (config->pdoaMode == DWT_PDOA_M3));
    qassert(((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_MODE_OFF) || ((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_MODE_1)
           || ((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_MODE_2) || ((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_MODE_ND)
           || ((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_MODE_SDC) || ((config->stsMode & DWT_STS_CONFIG_MASK) == (DWT_STS_MODE_1 | DWT_STS_MODE_SDC))
           || ((config->stsMode & DWT_STS_CONFIG_MASK) == (DWT_STS_MODE_2 | DWT_STS_MODE_SDC))
           || ((config->stsMode & DWT_STS_CONFIG_MASK) == (DWT_STS_MODE_ND | DWT_STS_MODE_SDC))
           || ((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_CONFIG_MASK));
#endif

    uint16_t preamble_len_sts = 0U;
    uint16_t preamble_len_ip = 0U;

    preamble_len_ip = ((config->txPreambLength + 1U) * 8U);

    LOCAL_DATA(dw)->sleep_mode &= (~((uint16_t)DWT_ALT_OPS | (uint16_t)DWT_SEL_OPS3)); // clear the sleep mode ALT_OPS bit
    LOCAL_DATA(dw)->longFrames = (uint8_t)config->phrMode;
    uint32_t sts_threshold_calc = (((((uint32_t)config->stsLength + 1UL)) * 8UL * STSQUAL_THRESH_64_SH15) >> 15UL);
    LOCAL_DATA(dw)->ststhreshold = (int16_t)sts_threshold_calc;
    LOCAL_DATA(dw)->stsconfig = (uint8_t)config->stsMode;

    // Set the temperature of the device so calibration can use it.
    uint16_t tempvbat = ull_readtempvbat(dw);
    LOCAL_DATA(dw)->temperature = (int8_t)ull_convertrawtemperature(dw, (uint8_t)(tempvbat >> 8U));  // Temperature in upper 8 bits

    if((LOCAL_DATA(dw)->temperature >= 0) && (LOCAL_DATA(dw)->vdddig_otp != 0U)) // If OTP is not provisioned, we cannot use set_vdddig_mv
    {
        (void)ull_set_vdddig_mv(dw, VDDDIG_88mV);
    }
    else
    {
        (void)ull_set_vdddig_mv(dw, VDDDIG_93mV);
    }

    /////////////////////////////////////////////////////////////////////////
    // SYS_CFG
    // clear the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
    // then set the relevant bits according to configuration of the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
    dwt_modify32bitoffsetreg(dw, SYS_CFG_ID, 0U,
        ~(uint32_t)(SYS_CFG_PHR_MODE_BIT_MASK | SYS_CFG_PHR_6M8_BIT_MASK | SYS_CFG_CP_SPC_BIT_MASK | SYS_CFG_PDOA_MODE_BIT_MASK | SYS_CFG_CP_SDC_BIT_MASK),
        ((uint32_t)config->pdoaMode << (uint32_t)SYS_CFG_PDOA_MODE_BIT_OFFSET) |
        (((uint32_t)config->stsMode & (uint32_t)DWT_STS_CONFIG_MASK) << (uint32_t)SYS_CFG_CP_SPC_BIT_OFFSET) |
        (SYS_CFG_PHR_6M8_BIT_MASK & ((uint32_t)config->phrRate << SYS_CFG_PHR_6M8_BIT_OFFSET)) | mode);

    /* Cache variables needed for ull_setpdoamode(). */
    LOCAL_DATA(dw)->stsLength = config->stsLength;
    LOCAL_DATA(dw)->pdoaMode = config->pdoaMode;

    if (scp != 0U)
    {
        // configure OPS tables for SCP mode
        LOCAL_DATA(dw)->sleep_mode |= (uint16_t)DWT_ALT_OPS | (uint16_t)DWT_SEL_OPS1; // configure correct OPS table is kicked on wakeup
        dwt_modify32bitoffsetreg(dw, OTP_CFG_ID, 0U, ~(uint32_t)(OTP_CFG_OPS_ID_BIT_MASK), DWT_OPSET_SCP | OTP_CFG_OPS_KICK_BIT_MASK);

        dwt_write32bitoffsetreg(dw, IP_CONFIG_LO_ID, 0U, IP_CONFIG_LO_SCP); // Set this if Ipatov analysis is used in SCP mode
        dwt_write32bitoffsetreg(dw, IP_CONFIG_HI_ID, 0U, IP_CONFIG_HI_SCP);

        dwt_write32bitoffsetreg(dw, STS_CONFIG_LO_ID, 0U, STS_CONFIG_LO_SCP);
        /* Disable ADC count and peak growth checks */
        dwt_modify32bitoffsetreg(dw, STS_CONFIG_HI_ID, 0U, ~(uint32_t)(STS_CONFIG_HI_STS_PGR_EN_BIT_MASK | STS_CONFIG_HI_STS_SS_EN_BIT_MASK | STS_CONFIG_HI_B0_MASK), STS_CONFIG_HI_SCP);
    }
    else
    {
        if (LOCAL_DATA(dw)->stsconfig != (uint8_t)DWT_STS_MODE_OFF)
        {
            int32_t sts_len_idx = GET_STS_LEN_IDX(LOCAL_DATA(dw)->stsLength);
            preamble_len_sts = (uint16_t)1U << ((uint16_t)sts_len_idx + 4U);

            config_sts_mnth(dw, config->pdoaMode); //configure the STS Minimum Threshold
        }

        // configure OPS tables for non-SCP mode
        if ((preamble_len_ip + preamble_len_sts) >= 256U)
        {
            LOCAL_DATA(dw)->sleep_mode |= (uint16_t)DWT_ALT_OPS | (uint16_t)DWT_SEL_OPS0;
            dwt_modify16bitoffsetreg(dw, OTP_CFG_ID, 0U, ~((uint16_t)OTP_CFG_OPS_ID_BIT_MASK),
                                     (uint16_t)DWT_OPSET_LONG | (uint16_t)OTP_CFG_OPS_KICK_BIT_MASK);
        }
        else
        {
            LOCAL_DATA(dw)->sleep_mode |= (uint16_t)DWT_ALT_OPS | (uint16_t)DWT_SEL_OPS2; // Short OPS table - set to be loaded as default
            dwt_modify16bitoffsetreg(dw, OTP_CFG_ID, 0U, (uint16_t) ~(OTP_CFG_OPS_ID_BIT_MASK),
                                     (uint16_t)DWT_OPSET_SHORT | (uint16_t)OTP_CFG_OPS_KICK_BIT_MASK);
        }
    
        /* Disable ADC count and peak growth checks */
        dwt_modify32bitoffsetreg(dw, STS_CONFIG_HI_ID, 0U, ~(uint32_t)(STS_CONFIG_HI_STS_PGR_EN_BIT_MASK | STS_CONFIG_HI_STS_SS_EN_BIT_MASK | STS_CONFIG_HI_B0_MASK), STS_CONFIG_HI_RES);

    }

    dwt_modify8bitoffsetreg(dw, DTUNE0_ID, 0U,  (uint8_t)~(DTUNE0_PRE_PAC_SYM_BIT_MASK), (const uint8_t)config->rxPAC); /* configure PAC size */

    dwt_write8bitoffsetreg(dw, STS_CFG0_ID, 0U, (uint8_t)(config->stsLength));

        // configure optimal preamble detection threshold.
    dwt_write32bitoffsetreg(dw, DTUNE3_ID, 0U, PD_THRESH_OPTIMAL);

    /////////////////////////////////////////////////////////////////////////
    // CHAN_CTRL
    temp = dwt_read32bitoffsetreg(dw, CHAN_CTRL_ID, 0U);
    temp &= (~(CHAN_CTRL_RX_PCODE_BIT_MASK | CHAN_CTRL_TX_PCODE_BIT_MASK | CHAN_CTRL_SFD_TYPE_BIT_MASK));

    temp |= (CHAN_CTRL_RX_PCODE_BIT_MASK & ((uint32_t)config->rxCode << CHAN_CTRL_RX_PCODE_BIT_OFFSET));
    temp |= (CHAN_CTRL_TX_PCODE_BIT_MASK & ((uint32_t)config->txCode << CHAN_CTRL_TX_PCODE_BIT_OFFSET));
    temp |= (CHAN_CTRL_SFD_TYPE_BIT_MASK & ((uint32_t)config->sfdType << CHAN_CTRL_SFD_TYPE_BIT_OFFSET));

    // to minimise SPI transactions - we are setting RX, TX Pcode and SFD type in a single SPI transaction
    dwt_write32bitoffsetreg(dw, CHAN_CTRL_ID, 0U, temp);

    ret_val = ull_setplenfine(dw, config->txPreambLength);
    if(ret_val != (int32_t)DWT_SUCCESS)
    {
        return ret_val;
    }

    /*
        TX_FCTRL
        Set up TX Preamble Size, PRF and Data Rate
    */
    dwt_modify32bitoffsetreg(dw, TX_FCTRL_ID, 0U, (uint32_t)~TX_FCTRL_TXBR_BIT_MASK,
        ((uint32_t)config->dataRate << TX_FCTRL_TXBR_BIT_OFFSET));

    // DTUNE (SFD timeout)
    // Don't allow 0 - SFD timeout will always be enabled
    if (config->sfdTO == 0U)
    {
        config->sfdTO = DWT_SFDTOC_DEF;
    }
    dwt_write16bitoffsetreg(dw, DTUNE0_ID, 2U, config->sfdTO);

    // Make sure PLL_COMMON is set to default value and update the PLL bias trim in the local structure
    dwt_write16bitoffsetreg(dw, PLL_COMMON_ID, 0U, (uint16_t)RF_PLL_COMMON);
    LOCAL_DATA(dw)->pll_bias_trim = DWT_DEF_PLLBIASTRIM;

    ret_val = ull_setchannel(dw, chan);
    if(ret_val != (int32_t)DWT_SUCCESS)
    {
        return ret_val;
    }

    ull_update_dgc_config(dw, chan);

    if (preamble_len_ip > 64U)
    {
        dwt_modify32bitoffsetreg(dw, DTUNE4_ID, 0x0U, (uint32_t)~DTUNE4_RX_SFD_HLDOFF_BIT_MASK, RX_SFD_HLDOFF);
    }
    else //set default value for <= 64
    {
        dwt_modify32bitoffsetreg(dw, DTUNE4_ID, 0x0U, (uint32_t)~DTUNE4_RX_SFD_HLDOFF_BIT_MASK, RX_SFD_HLDOFF_DEF);
    }

    dwt_write32bitreg(dw, TX_CTRL_LO_ID, TX_CTRL_LO_DEF);

    ///////////////////////
    // PGF

    // if the RX calibration routine fails the device receiver performance will be severely affected, the application should reset and try again
    ret_val = ull_pgf_cal(dw, 1);
#if DWT_DEBUG_PRINT
    if (error != (int32_t)DWT_SUCCESS)
    {
        printf("\nPGF CAL FAIL!!\n\n");
        printf("Result:%d\n", error);
    }
#endif

    return ret_val;
} // end dwt_configure()

 /*! ------------------------------------------------------------------------------------------------------------------
  * @brief This function runs the PGF calibration. This is needed prior to reception.
  *
  * @note If the RX calibration routine fails the device receiver performance will be severely affected, the application should reset and try again
  *
  * @param[in] dw: DW3000 chip descriptor handler.
  * @param[in] ldoen: If set to 1 the function will enable LDOs prior to calibration and disable afterwards.
  *
  * @return result of PGF calibration (@ref dwt_error_e DWT_ERR_RX_CAL*)
  *
 */
static int32_t ull_pgf_cal(dwchip_t *dw, int32_t ldoen)
{
    int32_t ret_val;
    uint16_t ldo_ctrl_val = 0U;

    // PGF needs LDOs turned on - ensure PGF LDOs are enabled
    if (ldoen == 1)
    {
        ldo_ctrl_val = dwt_read16bitoffsetreg(dw, LDO_CTRL_ID, 0U);

        dwt_or16bitoffsetreg(dw, LDO_CTRL_ID, 0U, (LDO_CTRL_LDO_VDDIF2_EN_BIT_MASK | LDO_CTRL_LDO_VDDMS3_EN_BIT_MASK | LDO_CTRL_LDO_VDDMS1_EN_BIT_MASK));
    }
    // Give the reference time to settle after enabling 4 LDOs simultaneously
    deca_usleep(DELAY_20uUSec);

    // Run PGF Cal
    ret_val = ull_run_pgfcal(dw);

    // Turn off RX LDOs if previously off
    if (ldoen == 1)
    {
        dwt_and16bitoffsetreg(dw, LDO_CTRL_ID, 0U, ldo_ctrl_val); // restore LDO values
    }
    return ret_val;
}

/*! ------------------------------------------------------------------------------------------------------------------
 *
 * @brief This function runs the PGF calibration. This is needed prior to reception.
 *
 * @note If the RX calibration routine fails the device receiver performance will be severely affected, the application should reset and try again
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return result of PGF calibration (@ref dwt_error_e DWT_ERR_RX_CAL*)
 *
 */
static int32_t ull_run_pgfcal(dwchip_t *dw)
{
    dwt_error_e result = DWT_ERR_RX_CAL_PGF;
    uint32_t data, val;
    // put into cal mode
    // Turn on delay mode
    data = ((2UL << RX_CAL_CFG_COMP_DLY_BIT_OFFSET) | (RX_CAL_CFG_CAL_MODE_BIT_MASK & 0x1UL));
    dwt_write32bitoffsetreg(dw, RX_CAL_CFG_ID, 0x0U, data);
    // Trigger PGF Cal
    dwt_or8bitoffsetreg(dw, RX_CAL_CFG_ID, 0x0U, RX_CAL_CFG_CAL_EN_BIT_MASK);

    for (uint8_t cnt = 0U; cnt < MAX_RETRIES_FOR_PGF; cnt++)
    {
        deca_usleep(DELAY_20uUSec);
        if (dwt_read8bitoffsetreg(dw, RX_CAL_STS_ID, 0x0U) == 1U)
        { // PGF cal is complete
            result = DWT_SUCCESS;
            break;
        }
    }

    // Put into normal mode
    dwt_write8bitoffsetreg(dw, RX_CAL_CFG_ID, 0x0U, 0U);
    dwt_write8bitoffsetreg(dw, RX_CAL_STS_ID, 0x0U, 1U); // clear the status

    if(result == DWT_SUCCESS)
    {
        dwt_or8bitoffsetreg(dw, RX_CAL_CFG_ID, 0x2U, 0x1U);  // enable reading
        val = dwt_read32bitoffsetreg(dw, RX_CAL_RESI_ID, 0x0U);
        if (val == ERR_RX_CAL_FAIL)
        {
            // PGF I Cal Fail
            result = DWT_ERR_RX_CAL_RESI;
        }
        else
        {
            // PGF I Cal Success
            val = dwt_read32bitoffsetreg(dw, RX_CAL_RESQ_ID, 0x0U);
            if (val == ERR_RX_CAL_FAIL)
            {
                // PGF Q Cal Fail
                result = DWT_ERR_RX_CAL_RESQ;
            }
        }
    }    

    return (int32_t)result;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API function writes the receiver antenna delay (in device time units) to RX registers
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] rxAntennaDelay: This is the total (RX) antenna delay value, which
 *                         will be programmed into the RX register, and subtracted from the raw RX timestamp
 *
 * @return  None
 */
static void ull_setrxantennadelay(dwchip_t *dw, uint16_t rxAntennaDelay)
{
    // Set the RX antenna delay for auto RX timestamp adjustment
    dwt_write16bitoffsetreg(dw, CIA_CONF_ID, 0U, rxAntennaDelay);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API function reads the antenna delay (in time units) from the RX antenna delay register
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return 16-bit RX antenna delay value which is currently programmed in CIA_CONF_ID register
 */
static uint16_t ull_getrxantennadelay(dwchip_t* dw)
{
    // Get the RX antenna delay used for auto RX timestamp adjustment
    return dwt_read16bitoffsetreg(dw, CIA_CONF_ID, 0U);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API function writes the receiver antenna delay (in device time units) to TX registers
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] txAntennaDelay: This is the total (TX) antenna delay value, which
 *                         will be programmed into the TX register, and added to the raw TX timestamp
 *
 * @return  None
 */
static void ull_settxantennadelay(dwchip_t *dw, uint16_t txAntennaDelay)
{
    // Set the TX antenna delay for auto TX timestamp adjustment
    dwt_write16bitoffsetreg(dw, TX_ANTD_ID, 0U, txAntennaDelay);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @brief This API function reads the antenna delay (in time units) from the TX antenna delay register
*
* @param[in] dw: DW3000 chip descriptor handler.
*
* @return 16-bit TX antenna delay value which is currently programmed in TX_ANTD_ID register
*/
static uint16_t ull_gettxantennadelay(dwchip_t* dw)
{
    // Get the TX antenna delay used for auto TX timestamp adjustment
    return dwt_read16bitoffsetreg(dw, TX_ANTD_ID, 0U);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API function writes the supplied TX data into the UWB radio TX buffer.
 * The input parameters are the data length in bytes and a pointer to those data bytes.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] txDataLength: This is the total length of data (in bytes) to write to the tx buffer.
 *                         Note the size of TX buffer is 1024 bytes.
 *                         The standard PHR mode allows to transmit frames of up to 127 bytes (including 2 byte CRC).
 *                         The extended PHR mode allows to transmit frames of up to 1023 bytes (including 2 byte CRC).
 *                         If > 127 is programmed, @ref DWT_PHRMODE_EXT needs to be set in the phrMode configuration
 *                         see dwt_configure() function
 * @param[in] txDataBytes: Pointer to the user's buffer containing the data to send.
 * @param[in] txBufferOffset: This specifies an offset in the UWB IC's TX Buffer at which to start writing data.
 *
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
 */
static int32_t ull_writetxdata(dwchip_t *dw, uint16_t txDataLength, uint8_t *txDataBytes, uint16_t txBufferOffset)
{
    int32_t retVal = (int32_t)DWT_ERROR;
#ifdef DWT_API_ERROR_CHECK
    qassert(((LOCAL_DATA(dw)->longFrames != 0U) && (txDataLength <= EXT_FRAME_LEN)) || (txDataLength <= STD_FRAME_LEN));
    qassert((txBufferOffset + txDataLength) < TX_BUFFER_MAX_LEN);
#endif

    if ((txBufferOffset + txDataLength) < TX_BUFFER_MAX_LEN)
    {
        if (txBufferOffset <= REG_DIRECT_OFFSET_MAX_LEN)
        {
            /* Directly write the data to the IC TX buffer */
            ull_writetodevice(dw, TX_BUFFER_ID, txBufferOffset, txDataLength, txDataBytes);
        }
        else
        {
            /* Program the indirect offset register A for specified offset to TX buffer */
            dwt_write32bitreg(dw, INDIRECT_ADDR_A_ID, (TX_BUFFER_ID >> 16U));
            dwt_write32bitreg(dw, ADDR_OFFSET_A_ID, txBufferOffset);

            /* Indirectly write the data to the IC TX buffer */
            ull_writetodevice(dw, INDIRECT_POINTER_A_ID, 0, txDataLength, txDataBytes);
        }
        retVal = (int32_t)DWT_SUCCESS;
    }

    return retVal;
} // end dwt_writetxdata()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API function configures the TX frame control register before the transmission of a frame
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] txFrameLength: This is the length of TX message (including the 2 byte CRC) - max is 1023
 *                              Note the standard PHR mode allows up to 127 bytes
 *                              if > 127 is programmed, @ref DWT_PHRMODE_EXT needs to be set in the phrMode configuration
 *                              see dwt_configure() function
 * @param[in] txBufferOffset: The offset in the tx buffer to start writing the data
 * @param[in] ranging: Set to 1 if this is a ranging frame, else can be set to 0.
 *
 * @return  None
 */
static void ull_writetxfctrl(dwchip_t *dw, uint16_t txFrameLength, uint16_t txBufferOffset, uint8_t ranging)
{
    uint32_t reg32;
#ifdef DWT_API_ERROR_CHECK
    qassert(((LOCAL_DATA(dw)->longFrames != 0U) && (txFrameLength <= EXT_FRAME_LEN)) || (txFrameLength <= STD_FRAME_LEN));
#endif

    // DW3000/3700 - if offset is > 127, 128 needs to be added before data is written, this will be subtracted internally
    // prior to writing the data
    if (txBufferOffset <= 127U)
    {
        // Write the frame length to the TX frame control register
        reg32 = (uint32_t)txFrameLength | ((uint32_t)txBufferOffset << TX_FCTRL_TXB_OFFSET_BIT_OFFSET) |
        ((uint32_t)ranging << TX_FCTRL_TR_BIT_OFFSET);
        dwt_modify32bitoffsetreg(dw, TX_FCTRL_ID, 0U, ~(uint32_t)(TX_FCTRL_TXB_OFFSET_BIT_MASK | TX_FCTRL_TR_BIT_MASK | TX_FCTRL_TXFLEN_BIT_MASK), reg32);
    }
    else
    {
        // Write the frame length to the TX frame control register
        reg32 = txFrameLength | (((uint32_t)txBufferOffset + DWT_TX_BUFF_OFFSET_ADJUST) << TX_FCTRL_TXB_OFFSET_BIT_OFFSET) |
        ((uint32_t)ranging << TX_FCTRL_TR_BIT_OFFSET);
        dwt_modify32bitoffsetreg(dw, TX_FCTRL_ID, 0U, ~(uint32_t)(TX_FCTRL_TXB_OFFSET_BIT_MASK | TX_FCTRL_TR_BIT_MASK | TX_FCTRL_TXFLEN_BIT_MASK), reg32);
        // DW3000/3700 - need to read this to load the correct TX buffer offset value
        reg32 = dwt_read8bitoffsetreg(dw, SAR_CTRL_ID, 0U);
    }

} // end dwt_writetxfctrl()

/*!------------------------------------------------------------------------------------------------------------------
* @brief This function configures frame Ipatov preamble length in fine steps of 8 symbols, from 16 to 4096.
*
* Valid range for the preamble length is [16, ... , 2048, 4096], and value should always be a multiple of 8.
* You can use convenience constants @ref DWT_PLEN_32 - @ref DWT_PLEN_4096 defined for some
* common preamble lengths.
*
* @note DW3xxx/QM33xxx parts do not accept preamble length values between 2048 and 4096.
*
* @note Setting preamble length of 16 symbols should be used for testing only and
* will likely result in poor performance. Not recommended on these parts.
*
* @param[in] dw: DW3000 chip descriptor handler.
* @param[in] preambleLength: The preamble length value in symbols.
*
* @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
*/
static int32_t ull_setplenfine(dwchip_t *dw, uint16_t preambleLength)
{
    dwt_error_e retVal = DWT_SUCCESS;

    if(CHECK_PREAMBLE_LEN_VALIDITY(preambleLength) == false)
    {
        retVal = DWT_ERROR;
    }
    else
    {
        if(preambleLength == DWT_PLEN_4096)
        {
            // DW3000 accepts DWT_PLEN_4096 only via TXPSR field
            // clear the setting in the FINE_PLEN register.
            dwt_write8bitoffsetreg(dw, TX_FCTRL_HI_ID, 1U, 0U);

            dwt_modify32bitoffsetreg(dw, TX_FCTRL_ID, 0U, (uint32_t)~TX_FCTRL_TXPSR_BIT_MASK, (0x3UL) << TX_FCTRL_TXPSR_BIT_OFFSET);
        }
        else
        {
            uint16_t pLenCode = (preambleLength >> 3U) - 1U;
            dwt_write16bitoffsetreg(dw, TX_FCTRL_HI_ID, 1U, pLenCode);
        }
    }

    return (int32_t)retVal;
}

/*!------------------------------------------------------------------------------------------------------------------
 * @brief This function enables/disables the PLL RX prebuffer (when the PLL is active)
 *
 * @note To enable the RX Pre-buffers, this function should be called when the device is in 
 * IDLE_RC mode, before calibrating the PLL. To disable the RX Pre-buffers, the PLL should be
 * re-calibrate after, if no other parameters have been changed.
 * 
 * @note Enabling the RX PLL Pre-buffers is recommended when using two standalone ICs to perform
 * PDoA, it will mitigate any phase ambiguity that may be observed, particularly in channel 5.
 * This is not required and not recommended when calculating PDoA with a single IC
 * (standard PDoA usage with QM33 or DW3000, PDoA mode 3 or PDoA mode 5), to avoid an increase
 * in power consumption.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] pll_rx_prebuf_cfg: New "PLL RX Prebuffer Enable" Configuration.
 *    DWT_PLL_RX_PREBUF_DISABLE - Disable the DW RX PLL Pre-Buffers
 *    DWT_PLL_RX_PREBUF_ENABLE - Enable the DW RX PLL Pre-Buffers   
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
 */
static int ull_setpllrxprebufen(dwchip_t *dw, dwt_pll_prebuf_cfg_e pll_rx_prebuf_cfg)
{
    dwt_error_e retVal = DWT_SUCCESS;
    uint32_t enable_mask = 0UL;

    if ((pll_rx_prebuf_cfg != DWT_PLL_RX_PREBUF_DISABLE) &&
        (pll_rx_prebuf_cfg != DWT_PLL_RX_PREBUF_ENABLE))
    {
        return (int32_t)DWT_ERROR;
    }

    if (pll_rx_prebuf_cfg == DWT_PLL_RX_PREBUF_ENABLE)
    {
        enable_mask |= (uint32_t)RF_ENABLE_PLL_RX_PRE_EN_BIT_MASK;
    }

    dwt_and_or8bitoffsetreg(dw, RF_ENABLE_ID, 3,
                            (uint8_t)(~RF_ENABLE_PLL_RX_PRE_EN_BIT_MASK >> 24), 
                            (uint8_t)(enable_mask >> 24));

    LOCAL_DATA(dw)->pll_rx_prebuf_cfg = pll_rx_prebuf_cfg;

    return (int)retVal;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to write the data to the scratch buffer, to an offset location given by offset parameter.
 * The scratch buffer size is 128 bytes. The buffer can be used by the AES engine depending on the configuration of
 * destination and source ports:  @ref dwt_aes_src_port_e and @ref dwt_aes_dst_port_e
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] buffer: Pointer to the buffer which contains the data to write to the device
 * @param[in] length: The length of data to write (in bytes)
 * @param[in] bufferOffset: The offset in the scratch buffer to which to write the data
 *
 * @return  None
 */
static void ull_write_scratch_data(dwchip_t *dw, uint8_t *buffer, uint16_t length, uint16_t bufferOffset)
{
    //!!Check later if needs range protection.

    /* Directly write data to the IC buffer */
    ull_writetodevice(dw, SCRATCH_RAM_ID, bufferOffset, length, buffer);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the data from the scratch buffer, from an offset location given by offset parameter.
 * The scratch buffer size is 128 bytes. The buffer can be used by the AES engine depending on the configuration of
 * destination and source ports:  @ref dwt_aes_src_port_e and @ref dwt_aes_dst_port_e
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] buffer: Pointer to the buffer into which the data will be read
 * @param[in]  length: The length of data to read (in bytes)
 * @param[in]  bufferOffset: The offset in the scratch buffer from which to read the data
 *
 * @return None
 */
static void ull_read_scratch_data(dwchip_t *dw, uint8_t *buffer, uint16_t length, uint16_t bufferOffset)
{
    //!!Check later if needs range protection.

    /* Directly read data from the IC buffer */
    ull_readfromdevice(dw, SCRATCH_RAM_ID, bufferOffset, length, buffer);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the data from the RX buffer, from an offset location give by offset parameter
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] buffer: Pointer to the buffer into which the data will be read
 * @param[in] length: The length of data to read (in bytes)
 * @param[in] rxBufferOffset: The offset in the RX buffer from which to read the data
 *
 * @return  None
 */
static void ull_readrxdata(dwchip_t *dw, uint8_t *buffer, uint16_t length, uint16_t rxBufferOffset)
{
    uint32_t rx_buff_addr;

    if (LOCAL_DATA(dw)->dblbuffon == (uint8_t)DBL_BUFF_ACCESS_BUFFER_1) // if the flag is 0x3 we are reading from RX_BUFFER_1
    {
        rx_buff_addr = RX_BUFFER_1_ID;
    }
    else // reading from RX_BUFFER_0 - also when non-double buffer mode
    {
        rx_buff_addr = RX_BUFFER_0_ID;
    }

    if ((rxBufferOffset + length) <= RX_BUFFER_MAX_LEN)
    {
        if (rxBufferOffset <= REG_DIRECT_OFFSET_MAX_LEN)
        {
            /* Directly read data from the IC to the buffer */
            ull_readfromdevice(dw, rx_buff_addr, rxBufferOffset, length, buffer);
        }
        else
        {
            /* Program the indirect offset registers B for specified offset to RX buffer */
            dwt_write32bitreg(dw, INDIRECT_ADDR_A_ID, (rx_buff_addr >> 16UL));
            dwt_write32bitreg(dw, ADDR_OFFSET_A_ID, (uint32_t)rxBufferOffset);

            /* Indirectly read data from the IC to the buffer */
            ull_readfromdevice(dw, INDIRECT_POINTER_A_ID, 0U, length, buffer);
        }
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the 18-bit data from the Accumulator (CIR) buffer, from an offset location give by offset parameter
 *        for 18-bit complex samples, each sample is 6 bytes (3 real and 3 imaginary)
 *
 *
 * @note Because of an internal memory access delay when reading the CIR the first octet output is a dummy octet
 *       that should be discarded. This is true no matter what sub-index the read begins at.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] buffer: The buffer into which the data will be read
 * @param[in] len:  The length of data to read (in bytes)
 * @param[in] accOffset: The offset in the acc buffer from which to read the data, this is a complex sample index
 *                       e.g., to read 10 samples starting at sample 100
 *                       buffer would need to be >= 10*6 + 1, length is 61 (1 is for dummy), accOffset is 100
 *
 *
 * @return None
 *
 * @deprecated This function is now deprecated for new development. Plase use dwt_readcir() or dwt_readcir_48b()
 */
static void ull_readaccdata(dwchip_t *dw, uint8_t *buffer, uint16_t length, uint16_t accOffset)
{
    // Force on the ACC clocks if we are sequenced
    dwt_or16bitoffsetreg(dw, CLK_CTRL_ID, 0x0U, CLK_CTRL_ACC_MCLK_EN_BIT_MASK | CLK_CTRL_ACC_CLK_EN_BIT_MASK);

    if ((accOffset + length) <= ACC_BUFFER_MAX_LEN)
    {
        if (accOffset <= REG_DIRECT_OFFSET_MAX_LEN)
        {
            /* Directly read data from the IC to the buffer */
            ull_readfromdevice(dw, ACC_MEM_ID, accOffset, length, buffer);
        }
        else
        {
            /* Program the indirect offset registers B for specified offset to ACC */
            dwt_write32bitreg(dw, INDIRECT_ADDR_A_ID, (ACC_MEM_ID >> 16UL));
            dwt_write32bitreg(dw, ADDR_OFFSET_A_ID, accOffset);

            /* Indirectly read data from the IC to the buffer */
            ull_readfromdevice(dw, INDIRECT_POINTER_A_ID, 0, length, buffer);
        }
    }
    else
    {
        qassert(0 == 1);
    }

    // Revert clocks back
    dwt_and16bitoffsetreg(dw, CLK_CTRL_ID, 0x0U, (uint16_t) ~(CLK_CTRL_ACC_MCLK_EN_BIT_MASK | CLK_CTRL_ACC_CLK_EN_BIT_MASK));
}

/*! ------------------------------------------------------------------------------------------------------------------
* This is used to read complex samples from the CIR/Accumulator buffer. There are two supported modes:
*
* - Full sample mode: (@ref DWT_CIR_READ_FULL),
* 48-bit complex samples with 24-bit real and 24-bit imaginary (18-bit dynamic range)
* - Reduced sample mode: (@ref DWT_CIR_READ_LO, @ref DWT_CIR_READ_MID, @ref DWT_CIR_READ_HI)
* 32-bit complex samples with 16-bit real and 16-bit imaginary.
*
*
* CIR sizes depend on the accumulator and on the PRF setting, see the following constants
*     @ref DWT_CIR_LEN_STS
*     @ref DWT_CIR_LEN_IP_PRF16
*     @ref DWT_CIR_LEN_IP_PRF64
*
* @param[in] dw: DW3000 chip descriptor handler.
* @param[out] buffer: The buffer into which the data will be read. The buffer should be big enough to accommodate
*                 num_samples of size 64-bit (2 words) for DWT_CIR_READ_FULL, or 32-bit (1 word) for the "faster"
*                 reading modes.
* @param[in] cir_idx: CIR/accumulator index. It is used to define the CIR accumulator address offset to read from (@ref dwt_acc_idx_e)
* @param[in] sample_offs: The sample index offset within the selected accumulator to start reading from
* @param[in] num_samples: The number of complex samples to read
* @param[in] mode: CIR read mode, see documentation for @ref dwt_cir_read_mode_e
*
* @return DWT_SUCCESS or DWT_ERROR if wrong parameters were passed
*/
static int ull_readcir(dwchip_t *dw, uint32_t *buffer, dwt_acc_idx_e cir_idx, uint16_t sample_offs,
                    uint16_t num_samples, dwt_cir_read_mode_e mode)
{
    static uint8_t buf_read[ 1U + (6U * CHUNK_CIR_NB_SAMP)];/* +1 as one leading byte unused when reading from Accumulator */
    uint16_t accOffset;
    uint16_t nb_samp_out = 0U, samp_to_read;
    uint8_t *p_wr = (uint8_t*)buffer;
    int16_t* p_wr_s16 = (int16_t*)(void*)buffer;

    //calculate the CIR accumulator offset
    uint16_t acc_offs = 0x0U;
    if ( cir_idx <= DWT_ACC_IDX_STS1_M )
    {
        acc_offs = dwt_cir_acc_offset[cir_idx];
    }
    else
    {
        return (int)DWT_ERROR;
    }

    accOffset = acc_offs + sample_offs;

    // Force on the ACC clocks if we are sequenced
    dwt_or16bitoffsetreg(dw, CLK_CTRL_ID, 0x0U, CLK_CTRL_ACC_MCLK_EN_BIT_MASK | CLK_CTRL_ACC_CLK_EN_BIT_MASK);

    while( (nb_samp_out < num_samples) && ((accOffset + nb_samp_out) <= ACC_BUFFER_MAX_LEN) )
    {
        if( (uint16_t)(num_samples - nb_samp_out) >= CHUNK_CIR_NB_SAMP )
        {
            samp_to_read = CHUNK_CIR_NB_SAMP;
        }
        else
        {
            samp_to_read = num_samples - nb_samp_out;
        }

        /* Program the indirect offset registers A for specified offset to ACC */
        dwt_write32bitreg(dw, INDIRECT_ADDR_A_ID, (ACC_MEM_ID >> 16UL));
        dwt_write32bitreg(dw, ADDR_OFFSET_A_ID, (uint32_t)accOffset + (uint32_t)nb_samp_out);

        /* Indirectly read data from the IC to the buffer */
        /* 1 extra byte unused, then 3 bytes per real part, 3 byte per imaginary part,
        i.e. 6 bytes per complex samples to read */
        ull_readfromdevice(dw, INDIRECT_POINTER_A_ID, 0U, 1U + (6U * samp_to_read), buf_read);

        uint8_t *p_rd = buf_read + 1U; /* 1st byte shall be ignored when reading from Accumulator */

        if(mode == DWT_CIR_READ_FULL)
        {
            // Copy the read buffer into the output buffer
            for(uint16_t i = 0U; i < (6U * samp_to_read); i++) {
                *p_wr++ = *p_rd++;
            }
        }
        else
        {
            /*
                In QM33 hardware, each sample is a 24 bit number, with the upper 6 bits being the sign and lower 18 bits the value.
                We need to first transpose the 24 bits into 32 bits and then compresse to 16 bits.
                The 24 bit sample is formed in the following way:
                    S S S S S S V17 V16 V15 V14 V13 V12 V11 V10 V9 V8 V7 V6 V5 V4 V3 V2 V1 V0
                The final 16 bit sample will depend on the reading modes as follows:
                    - DWT_CIR_READ_LO: S V14 V13 V12 V11 V10 V9 V8 V7 V6 V5 V4 V3 V2 V1 V0
                    - DWT_CIR_READ_MI: S V15 V14 V13 V12 V11 V10 V9 V8 V7 V6 V5 V4 V3 V2 V1
                    - DWT_CIR_READ_HI: S V16 V15 V14 V13 V12 V11 V10 V9 V8 V7 V6 V5 V4 V3 V2
            */
            uint32_t current_sample_24bit;
            uint32_t current_sample_32bit;
            uint32_t sign_extended_32bit;
            int32_t current_sample_signed;

            for(uint16_t k = 0U; k < (2U * samp_to_read); k ++)
            {
                // Get the full 24bit sample
                current_sample_24bit = (uint32_t)p_rd[ 0 ] + ((uint32_t)p_rd[ 1 ] << 8UL) + ((uint32_t)p_rd[ 2 ] << 16UL);

                // Check the sign
                if((current_sample_24bit & DWT_CIR_SIGN_24BIT_EXTEND_32BIT_MASK) != 0x0UL)
                {
                    sign_extended_32bit = DWT_CIR_SIGN_24BIT_EXTEND_32BIT_MASK;
                }
                else
                {
                    sign_extended_32bit = 0UL;
                }

                // Extend the sign in the 32bit sample
                current_sample_32bit = (current_sample_24bit & DWT_CIR_VALUE_NO_SIGN_18BIT_MASK) | sign_extended_32bit;

                // Shift as follow:
                // Reduced 32-bit complex samples: bits [15:0] for real/imag parts if DWT_CIR_READ_LO, no shift
                // Reduced 32-bit complex samples: bits [16:1] for real/imag parts if DWT_CIR_READ_MID, shift by 1
                // Reduced 32-bit complex samples: bits [17:2] for real/imag parts if DWT_CIR_READ_HI, shift by 2
                if(mode == DWT_CIR_READ_MID)
                {
                    current_sample_32bit = (current_sample_32bit >> 1UL) | sign_extended_32bit; // Keep sign extension
                }else if(mode == DWT_CIR_READ_HI)
                {
                    current_sample_32bit = (current_sample_32bit >> 2UL) | sign_extended_32bit; // Keep sign extension
                }
                else
                {
                    // Do nothing
                }

                current_sample_signed = (int32_t)current_sample_32bit;

                /* Check for saturation */
                if(current_sample_signed > 32767)
                {
                    current_sample_signed = 32767;
                }
                else if(current_sample_signed < -32768)
                {
                    current_sample_signed = -32768;
                }
                else
                {
                    // Do nothing
                }

                // Convert into 16 bits and output the data
                p_wr_s16[(2U * nb_samp_out) + k ] = (int16_t)current_sample_signed;
                p_rd += 3U; /*  CIR samples real/imag are read as 24 bits long from HW CIR */
            }
        }

        nb_samp_out += samp_to_read;
    }
    // Revert clocks back
    dwt_and16bitoffsetreg(dw, CLK_CTRL_ID, 0x0U, (uint16_t) ~(CLK_CTRL_ACC_MCLK_EN_BIT_MASK | CLK_CTRL_ACC_CLK_EN_BIT_MASK));
    return (int)DWT_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the crystal offset (relating to the frequency offset of the far UWB device compared to this one)
 *        @note The returned signed 16-bit number should be divided by 2^26 to get ppm offset.
 *              A positive value means the local (RX) clock is running slower than that of the remote (TX) device.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return The offset value, signed 16-bit number. (s[-15:-26])
 *
 */
static int16_t ull_readclockoffset(dwchip_t *dw)
{
    uint16_t regval;
    int16_t retval;

    switch ((dwt_dbl_buff_conf_e)LOCAL_DATA(dw)->dblbuffon)
    // if the flag is non zero - we are either accessing RX_BUFFER_0 or RX_BUFFER_1
    {
    case DBL_BUFF_ACCESS_BUFFER_1:
        //!!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
        regval = dwt_read16bitoffsetreg(dw, INDIRECT_POINTER_B_ID, (uint16_t)(BUF1_CIA_DIAG_0 - BUF1_RX_FINFO));
        break;
    case DBL_BUFF_ACCESS_BUFFER_0:
        regval = dwt_read16bitoffsetreg(dw, BUF0_CIA_DIAG_0, 0U);
        break;
    default:
        regval = dwt_read16bitoffsetreg(dw, CIA_DIAG_0_ID, 0U);
        break;
    }

    regval &= CIA_DIAG_0_COE_PPM_BIT_MASK;
    // Bit 12 is sign, make the number to be sign extended if this bit is '1'
    if ((regval & INT13_SIGN_BIT_MASK) != 0U)
    {
        regval = INT13_SIGN_POWN - regval; // get the absolute value
        retval = -(int16_t)regval;
    }
    else
    {
        retval = (int16_t)regval;
    }

    return retval;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the RX carrier integrator value (relating to the frequency offset of the TX node)
 *        @note This is a 21-bit signed quantity, the function sign extends the most significant bit, which is bit #20
 *        (numbering from bit zero) to return a 32-bit signed integer value.
 *        A positive value means the local (RX) clock is running slower than that of the remote (TX) device.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return The carrier integrator value, signed 32-bit number.
 */
static int32_t ull_readcarrierintegrator(dwchip_t *dw)
{
    uint32_t regval = 0U;
    int32_t retval;
    uint8_t buffer[DRX_CARRIER_INT_LEN];

    /* Read 3 bytes into buffer (21-bit quantity) */
    ull_readfromdevice(dw, DRX_DIAG3_ID, 0U, DRX_CARRIER_INT_LEN, buffer);

    // arrange the three bytes into an unsigned integer value
    regval = ((uint32_t)buffer[2] << 16UL) + ((uint32_t)buffer[1] << 8UL) + (uint32_t)buffer[0];

    if ((regval & INT21_SIGN_BIT_MASK) != 0UL)
    {
        regval = INT21_SIGN_POWN - regval; // Get the absolute value
        retval = -(int32_t)regval;
    }
    else
    {
        retval = (int32_t)regval;
    }

    return retval; // cast unsigned value to signed quantity.
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function reads the STS signal quality index
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] rxStsQualityIndex: The signed STS quality index value (int16_t).
 *
 * @return value >=0 for good and < 0 if bad STS quality.
 *
 * @note For the 64 MHz PRF if value is >= @ref STSQUAL_THRESH_64_SH15 (60%) of the STS length then we can assume good STS reception.
 *       Otherwise the STS timestamp may not be accurate.
 */
static int32_t ull_readstsquality(dwchip_t *dw, int16_t *rxStsQualityIndex)
{
    uint16_t preambleCount;
    int16_t preambleCount_signed;

    // read STS preamble count value
    preambleCount = dwt_read16bitoffsetreg(dw, STS_STS_ID, 0U) & STS_STS_ACC_QUAL_BIT_MASK;

    if ((preambleCount & STS_ACC_CP_QUAL_SIGNTST) != 0U)
    {
        preambleCount = STS_ACC_CP_QUAL_SIGNTOP - preambleCount;
        preambleCount_signed = -(int16_t)preambleCount;
    }
    else
    {
        preambleCount_signed = (int16_t)preambleCount;
    }

    *rxStsQualityIndex = preambleCount_signed;

    // determine if the STS Rx quality is good or bad (return >=0 for good and < 0 if bad)
    return ((int32_t)preambleCount_signed - (int32_t)LOCAL_DATA(dw)->ststhreshold);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function reads the STS status
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] stsStatus: The (uint16_t) STS status value. 9 bits of this buffer are populated with various STS statuses. The
 *                    remaining 7 bits are ignored.
 * @param[in] sts_num: 0 for 1st STS, 1 for 2nd STS (2nd is only valid when PDOA Mode 3 is used)
 *
 * @return value 0 for good/valid STS < 0 if bad STS quality.
 */
static int32_t ull_readstsstatus(dwchip_t *dw, uint16_t *stsStatus, int32_t sts_num)
{
    uint32_t reg_offset;
    dwt_error_e ret = DWT_SUCCESS;
    uint32_t stsStatusRegAdd = (sts_num == 1) ? BUF0_STS1_STAT : BUF0_STS_STAT;
    uint32_t stsStatusRegAddN = (sts_num == 1) ? STS1_TOA_HI_ID : STS_TOA_HI_ID;

    switch ((dwt_dbl_buff_conf_e)LOCAL_DATA(dw)->dblbuffon)
    // check if in double buffer mode and if so which buffer host is currently accessing
    {
    case DBL_BUFF_ACCESS_BUFFER_1:
        //!!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
        reg_offset = (stsStatusRegAdd - BUF0_RX_FINFO + 2UL) >> 7UL;
        *stsStatus = dwt_read16bitoffsetreg(dw, INDIRECT_POINTER_B_ID, (uint16_t)reg_offset);
        break;
    case DBL_BUFF_ACCESS_BUFFER_0:
        *stsStatus = (dwt_read16bitoffsetreg(dw, stsStatusRegAdd, 2U) >> 7U);
        break;
    default:
        *stsStatus = (dwt_read16bitoffsetreg(dw, stsStatusRegAddN, 2U) >> 7U);
        break;
    }

    // determine if the STS is ok
    if (*stsStatus != 0U)
    {
        ret = DWT_ERROR;
    }

    return (int32_t)ret;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function reads the RX signal quality diagnostic data
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] diagnostics: Diagnostic structure (@ref dwt_rxdiag_t) pointer,
 *                         this will contain the diagnostic data read from the UWB IC
 *
 * @return None
 */
static void ull_readdiagnostics(dwchip_t *dw, dwt_rxdiag_t *diagnostics)
{
    uint16_t xtal_offset_calc, pdoa_calc;
    int16_t pdoa_calc_signed;
    uint32_t offset_0xd = STS_DIAG_3_LEN + STS_DIAG_3_ID - IP_TOA_LO_ID; // there are 0x6C bytes in 0xC0000 base before we enter 0xD0000
    uint16_t ip_length_min = IP_TOA_LO_IP_TOA_BIT_LEN + (IP_TOA_LO_LEN * 2U);
    uint32_t offset_buff = BUF0_RX_FINFO;
    // address from 0xC0000 to 0xD0068 (108*2 bytes) - when using normal mode, or 232 length for max logging when in Double Buffer mode
    uint8_t temp[DB_MAX_DIAG_SIZE];
    // minimal diagnostics - 40 bytes

    switch ((dwt_dbl_buff_conf_e)LOCAL_DATA(dw)->dblbuffon)
    // check if in double buffer mode and if so which buffer host is currently accessing
    {
    /* Intentional fall through */
    case DBL_BUFF_ACCESS_BUFFER_1:
    case DBL_BUFF_ACCESS_BUFFER_0:
        if (LOCAL_DATA(dw)->dblbuffon == (uint8_t)DBL_BUFF_ACCESS_BUFFER_1)
        {
            /* Program the indirect offset registers B for specified offset to swinging set buffer B */
            //!!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
            /* Indirectly read data from the IC to the buffer */
            if ((LOCAL_DATA(dw)->cia_diagnostic & (uint8_t)DW_CIA_DIAG_LOG_MAX) != 0U)
            {
                ull_readfromdevice(dw, INDIRECT_POINTER_B_ID, 0U, DB_MAX_DIAG_SIZE, temp);
            }
            else if ((LOCAL_DATA(dw)->cia_diagnostic & (uint8_t)DW_CIA_DIAG_LOG_MID) != 0U)
            {
                ull_readfromdevice(dw, INDIRECT_POINTER_B_ID, 0U, DB_MID_DIAG_SIZE, temp);
            }
            else
            {
                ull_readfromdevice(dw, INDIRECT_POINTER_B_ID, 0U, DB_MIN_DIAG_SIZE, temp);
            }
        }
        else
        {
            if ((LOCAL_DATA(dw)->cia_diagnostic & (uint8_t)DW_CIA_DIAG_LOG_MAX) != 0U)
            {
                ull_readfromdevice(dw, offset_buff, 0U, DB_MAX_DIAG_SIZE, temp);
            }
            else if ((LOCAL_DATA(dw)->cia_diagnostic & (uint8_t)DW_CIA_DIAG_LOG_MID) != 0U)
            {
                ull_readfromdevice(dw, offset_buff, 0U, DB_MID_DIAG_SIZE, temp);
            }
            else
            {
                ull_readfromdevice(dw, offset_buff, 0U, DB_MIN_DIAG_SIZE, temp);
            }
        }

        for (uint16_t i = 0U; i < (CIA_I_RX_TIME_LEN + 1U); i++)
        {
            diagnostics->tdoa[i] = temp[i + (BUF0_TDOA - BUF0_RX_FINFO)]; // timestamp difference of the 2 STS RX timestamps
        }

        // Estimated xtal offset of remote device
        xtal_offset_calc = ((((uint16_t)temp[BUF0_CIA_DIAG_0 - BUF0_RX_FINFO + 1UL] << (uint16_t)8U) |
                                       (uint16_t)temp[BUF0_CIA_DIAG_0 - BUF0_RX_FINFO]) & 0x1FFFU);
        diagnostics->xtalOffset = (int16_t)xtal_offset_calc;

        // phase difference of the 2 STS POAs (signed in [1:-11])
        pdoa_calc =  ((((uint16_t)temp[BUF0_PDOA - BUF0_RX_FINFO + 3UL] << 8U) |
                                 (uint16_t)temp[BUF0_PDOA - BUF0_RX_FINFO + 2UL]) & 0x3FFFU);
        if ((pdoa_calc & INT14_SIGN_BIT_MASK) != 0U)
        {
            pdoa_calc = INT14_SIGN_POWN - pdoa_calc; // Get the absolute value
            pdoa_calc_signed = -(int16_t)pdoa_calc;
        }
        else
        {
            pdoa_calc_signed = (int16_t)pdoa_calc;
        }

        diagnostics->pdoa = pdoa_calc_signed;

        // Number accumulated symbols [11:0] for Ipatov sequence
        diagnostics->ipatovAccumCount = ((((uint16_t)temp[BUF0_IP_DIAG_12 - BUF0_RX_FINFO + 1UL] << 8U) |
                                           (uint16_t)temp[BUF0_IP_DIAG_12 - BUF0_RX_FINFO]) & 0xFFFU);

        if ((LOCAL_DATA(dw)->cia_diagnostic & (uint8_t)DW_CIA_DIAG_LOG_MIN) != 0U)
        {
            break;
        }

        for (uint32_t i = 0UL; i < CIA_I_RX_TIME_LEN; i++)
        {
            // RX timestamp from Ipatov sequence
            diagnostics->ipatovRxTime[i] = temp[i + BUF0_IP_TS - BUF0_RX_FINFO];
            // RX timestamp from STS
            diagnostics->stsRxTime[i] = temp[i + BUF0_STS_TS - BUF0_RX_FINFO];
            // RX timestamp from STS1
            diagnostics->sts2RxTime[i] = temp[i + BUF0_STS1_TS - BUF0_RX_FINFO];
        }
        // RX status info for Ipatov sequence
        diagnostics->ipatovRxStatus = temp[BUF0_RES2 - BUF0_RX_FINFO + CIA_I_STAT_OFFSET];
        // Phase of arrival as computed from the Ipatov CIR (signed rad*2-12)
        diagnostics->ipatovPOA = (((uint16_t)temp[BUF0_RES2 - BUF0_RX_FINFO + 2UL] << 8U) |
                                   (uint16_t)temp[BUF0_RES2 - BUF0_RX_FINFO + 1UL]);

        // RX status info for STS
        diagnostics->stsRxStatus = ((((uint16_t)temp[BUF0_STS_STAT - BUF0_RX_FINFO + CIA_C_STAT_OFFSET + 1UL] << 8U) +
                                      (uint16_t)temp[BUF0_STS_STAT - BUF0_RX_FINFO + CIA_C_STAT_OFFSET]) >> 7U);
        // Phase of arrival as computed from the STS 1 CIR (signed rad*2-12)
        diagnostics->stsPOA = (((uint16_t)temp[BUF0_STS_TS - BUF0_RX_FINFO + 2UL] << 8U) |
                                (uint16_t)temp[BUF0_STS_TS - BUF0_RX_FINFO + 1UL]);

        // RX status info for STS1
        diagnostics->sts2RxStatus = ((((uint16_t)temp[BUF0_STS1_STAT - BUF0_RX_FINFO + CIA_C_STAT_OFFSET + 1UL] << 8U) +
                                       (uint16_t)temp[BUF0_STS1_STAT - BUF0_RX_FINFO + CIA_C_STAT_OFFSET]) >> 7U);
        // Phase of arrival as computed from the STS 1 CIR (signed rad*2-12)
        diagnostics->sts2POA = (((uint16_t)temp[BUF0_STS1_TS - BUF0_RX_FINFO + 2UL] << 8U) |
                                 (uint16_t)temp[BUF0_STS1_TS - BUF0_RX_FINFO + 1UL]);

        if ((LOCAL_DATA(dw)->cia_diagnostic & (uint8_t)DW_CIA_DIAG_LOG_MID) != 0U)
        {
            break;
        }

        diagnostics->ciaDiag1 = ((((uint32_t)temp[BUF0_CIA_DIAG_1 - BUF0_RX_FINFO + 3UL] << 24UL) |
                                  ((uint32_t)temp[BUF0_CIA_DIAG_1 - BUF0_RX_FINFO + 2UL] << 16UL) |
                                  ((uint32_t)temp[BUF0_CIA_DIAG_1 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                   (uint32_t)temp[BUF0_CIA_DIAG_1 - BUF0_RX_FINFO])
                                  & 0x1FFFFFFFUL);

        // IP
        // index [30:21] and amplitude [20:0] of peak sample in Ipatov sequence CIR
        diagnostics->ipatovPeak = ((((uint32_t)temp[BUF0_IP_DIAG_0 - BUF0_RX_FINFO + 3UL] << 24UL) |
                                    ((uint32_t)temp[BUF0_IP_DIAG_0 - BUF0_RX_FINFO + 2UL] << 16UL) |
                                    ((uint32_t)temp[BUF0_IP_DIAG_0 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                     (uint32_t)temp[BUF0_IP_DIAG_0 - BUF0_RX_FINFO])
                                    & 0x7FFFFFFFUL);
        // channel area allows estimation [16:0] of channel power for the Ipatov sequence
        diagnostics->ipatovPower = ((((uint32_t)temp[BUF0_IP_DIAG_1 - BUF0_RX_FINFO + 3UL] << 24UL) |
                                     ((uint32_t)temp[BUF0_IP_DIAG_1 - BUF0_RX_FINFO + 2UL] << 16UL) |
                                     ((uint32_t)temp[BUF0_IP_DIAG_1 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                      (uint32_t)temp[BUF0_IP_DIAG_1 - BUF0_RX_FINFO])
                                     & 0x1FFFFUL);
        // F1 for Ipatov sequence [21:0]
        diagnostics->ipatovF1 = ((((uint32_t)temp[BUF0_IP_DIAG_2 - BUF0_RX_FINFO + 3UL] << 24UL) |
                                  ((uint32_t)temp[BUF0_IP_DIAG_2 - BUF0_RX_FINFO + 2UL] << 16UL) |
                                  ((uint32_t)temp[BUF0_IP_DIAG_2 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                   (uint32_t)temp[BUF0_IP_DIAG_2 - BUF0_RX_FINFO])
                                  & 0x3FFFFFUL);
        // F2 for Ipatov sequence [21:0]
        diagnostics->ipatovF2 = ((((uint32_t)temp[BUF0_IP_DIAG_3 - BUF0_RX_FINFO + 3UL] << 24UL) |
                                  ((uint32_t)temp[BUF0_IP_DIAG_3 - BUF0_RX_FINFO + 2UL] << 16UL) |
                                  ((uint32_t)temp[BUF0_IP_DIAG_3 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                   (uint32_t)temp[BUF0_IP_DIAG_3 - BUF0_RX_FINFO])
                                  & 0x3FFFFFUL);
        // F3 for Ipatov sequence [21:0]
        diagnostics->ipatovF3 = ((((uint32_t)temp[BUF0_IP_DIAG_4 - BUF0_RX_FINFO + 3UL] << 24UL) |
                                 ((uint32_t)temp[BUF0_IP_DIAG_4 - BUF0_RX_FINFO + 2UL] << 16UL) |
                                 ((uint32_t)temp[BUF0_IP_DIAG_4 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                  (uint32_t)temp[BUF0_IP_DIAG_4 - BUF0_RX_FINFO])
                                 & 0x3FFFFFUL);
        // First path index [15:0] for Ipatov sequence
        diagnostics->ipatovFpIndex = (((uint16_t)temp[BUF0_IP_DIAG_8 - BUF0_RX_FINFO + 1UL] << 8U) |
                                       (uint16_t)temp[BUF0_IP_DIAG_8 - BUF0_RX_FINFO]);

        // CP 1
        // index [29:21] and amplitude [20:0] of peak sample in STS CIR
        diagnostics->stsPeak = ((((uint32_t)temp[BUF0_STS_DIAG_0 - BUF0_RX_FINFO + 3UL] << 24UL) |
                                 ((uint32_t)temp[BUF0_STS_DIAG_0 - BUF0_RX_FINFO + 2UL] << 16UL) |
                                 ((uint32_t)temp[BUF0_STS_DIAG_0 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                  (uint32_t)temp[BUF0_STS_DIAG_0 - BUF0_RX_FINFO])
                                 & 0x3FFFFFFFUL);
        // channel area allows estimation of channel power for the STS
        diagnostics->stsPower = (((uint32_t)temp[BUF0_STS_DIAG_1 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                  (uint32_t)temp[BUF0_STS_DIAG_1 - BUF0_RX_FINFO]);

        // F1 for STS [21:0]
        diagnostics->stsF1 = ((((uint32_t)temp[BUF0_STS_DIAG_2 - BUF0_RX_FINFO + 3UL] << 24UL) |
                               ((uint32_t)temp[BUF0_STS_DIAG_2 - BUF0_RX_FINFO + 2UL] << 16UL) |
                               ((uint32_t)temp[BUF0_STS_DIAG_2 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                (uint32_t)temp[BUF0_STS_DIAG_2 - BUF0_RX_FINFO])
                               & 0x3FFFFFUL);
        // F2 for STS [21:0]
        diagnostics->stsF2 = ((((uint32_t)temp[BUF0_STS_DIAG_3 - BUF0_RX_FINFO + 3UL] << 24UL) |
                               ((uint32_t)temp[BUF0_STS_DIAG_3 - BUF0_RX_FINFO + 2UL] << 16UL) |
                               ((uint32_t)temp[BUF0_STS_DIAG_3 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                (uint32_t)temp[BUF0_STS_DIAG_3 - BUF0_RX_FINFO])
                               & 0x3FFFFFUL);
        // F3 for STS [21:0]
        diagnostics->stsF3 = ((((uint32_t)temp[BUF0_STS_DIAG_4 - BUF0_RX_FINFO + 3UL] << 24UL) |
                               ((uint32_t)temp[BUF0_STS_DIAG_4 - BUF0_RX_FINFO + 2UL] << 16UL) |
                               ((uint32_t)temp[BUF0_STS_DIAG_4 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                (uint32_t)temp[BUF0_STS_DIAG_4 - BUF0_RX_FINFO])
                               & 0x3FFFFFUL);
        // First path index [14:0] for STS
        diagnostics->stsFpIndex = ((((uint16_t)temp[BUF0_STS_DIAG_8 - BUF0_RX_FINFO + 1UL] << 8U) |
                                     (uint16_t)temp[BUF0_STS_DIAG_8 - BUF0_RX_FINFO]) & 0x7FFFU);
        // Number accumulated symbols [11:0] for STS
        diagnostics->stsAccumCount = ((((uint16_t)temp[BUF0_STS_DIAG_12 - BUF0_RX_FINFO + 1UL] << 8U) |
                                        (uint16_t)temp[BUF0_STS_DIAG_12 - BUF0_RX_FINFO]) & 0xFFFU);

        // CP 2
        // index [29:21] and amplitude [20:0] of peak sample in STS CIR
        diagnostics->sts2Peak = ((((uint32_t)temp[BUF0_STS1_DIAG_0 - BUF0_RX_FINFO + 3UL] << 24UL) |
                                  ((uint32_t)temp[BUF0_STS1_DIAG_0 - BUF0_RX_FINFO + 2UL] << 16UL) |
                                  ((uint32_t)temp[BUF0_STS1_DIAG_0 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                   (uint32_t)temp[BUF0_STS1_DIAG_0 - BUF0_RX_FINFO])
                                 & 0x3FFFFFFFUL);
        // channel area allows estimation of channel power for the STS
        diagnostics->sts2Power = (((uint32_t)temp[BUF0_STS1_DIAG_1 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                   (uint32_t)temp[BUF0_STS1_DIAG_1 - BUF0_RX_FINFO]);
        // F1 for STS [21:0]
        diagnostics->sts2F1 = ((((uint32_t)temp[BUF0_STS1_DIAG_2 - BUF0_RX_FINFO + 3UL] << 24UL) |
                                ((uint32_t)temp[BUF0_STS1_DIAG_2 - BUF0_RX_FINFO + 2UL] << 16UL) |
                                ((uint32_t)temp[BUF0_STS1_DIAG_2 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                 (uint32_t)temp[BUF0_STS1_DIAG_2 - BUF0_RX_FINFO])
                                & 0x3FFFFFUL);
        // F2 for STS [21:0]
        diagnostics->sts2F2 = ((((uint32_t)temp[BUF0_STS1_DIAG_3 - BUF0_RX_FINFO + 3UL] << 24UL) |
                                ((uint32_t)temp[BUF0_STS1_DIAG_3 - BUF0_RX_FINFO + 2UL] << 16UL) |
                                ((uint32_t)temp[BUF0_STS1_DIAG_3 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                 (uint32_t)temp[BUF0_STS1_DIAG_3 - BUF0_RX_FINFO])
                                & 0x3FFFFFUL);
        // F3 for STS [21:0]
        diagnostics->sts2F3 = ((((uint32_t)temp[BUF0_STS1_DIAG_4 - BUF0_RX_FINFO + 3UL] << 24UL) |
                                ((uint32_t)temp[BUF0_STS1_DIAG_4 - BUF0_RX_FINFO + 2UL] << 16UL) |
                                ((uint32_t)temp[BUF0_STS1_DIAG_4 - BUF0_RX_FINFO + 1UL] << 8UL) |
                                 (uint32_t)temp[BUF0_STS1_DIAG_4 - BUF0_RX_FINFO])
                                & 0x3FFFFFUL);
        // First path index [14:0] for STS
        diagnostics->sts2FpIndex = ((((uint16_t)temp[BUF0_STS1_DIAG_8 - BUF0_RX_FINFO + 1UL] << 8U) |
                                      (uint16_t)temp[BUF0_STS1_DIAG_8 - BUF0_RX_FINFO]) & 0x7FFFU);
        // Number accumulated symbols [11:0] for STS
        diagnostics->sts2AccumCount = ((((uint16_t)temp[BUF0_STS1_DIAG_12 - BUF0_RX_FINFO + 1UL] << 8U) |
                                         (uint16_t)temp[BUF0_STS1_DIAG_12 - BUF0_RX_FINFO]) & 0xFFFU);

        break;

    default: // double buffer is off

        if ((LOCAL_DATA(dw)->cia_diagnostic & (uint8_t)DW_CIA_DIAG_LOG_ALL) != 0U)
        {
            ull_readfromdevice(dw, IP_TOA_LO_ID, 0U, (uint16_t)offset_0xd, temp);        // read form 0xC0000 space  (108 bytes)
            ull_readfromdevice(dw, STS_DIAG_4_ID, 0U, (uint16_t)offset_0xd, &temp[offset_0xd]); // read from 0xD0000 space  (108 bytes)
        }
        else // even if other CIA_DIAG logging levels are set (e.g. MAX, MID or MIN) as double buffer is not used, we only log as if only MIN is set
        {
            ull_readfromdevice(dw, IP_TOA_LO_ID, 0U, ip_length_min, temp);
        }

        for (uint32_t i = 0UL; i < CIA_I_RX_TIME_LEN; i++)
        {
            diagnostics->ipatovRxTime[i] = temp[i];                               // RX timestamp from Ipatov sequence
            diagnostics->stsRxTime[i] = temp[i + STS_TOA_LO_ID - IP_TOA_LO_ID];   // RX timestamp from STS
            diagnostics->sts2RxTime[i] = temp[i + STS1_TOA_LO_ID - IP_TOA_LO_ID]; // RX timestamp from STS1
            diagnostics->tdoa[i] = temp[i + CIA_TDOA_0_ID - IP_TOA_LO_ID];        // timestamp difference of the 2 STS RX timestamps
        }
        diagnostics->tdoa[5] = temp[5UL + CIA_TDOA_0_ID - IP_TOA_LO_ID];

        // RX status info for Ipatov sequence
        diagnostics->ipatovRxStatus = temp[IP_TOA_HI_ID - IP_TOA_LO_ID + CIA_I_STAT_OFFSET];
        // Phase of arrival as computed from the Ipatov CIR (signed rad*2-12)
        diagnostics->ipatovPOA = (((uint16_t)temp[IP_TOA_HI_ID - IP_TOA_LO_ID + 2UL] << 8U) |
                                   (uint16_t)temp[IP_TOA_HI_ID - IP_TOA_LO_ID + 1UL]);

        // RX status info for STS
        diagnostics->stsRxStatus = ((((uint16_t)temp[STS_TOA_HI_ID - IP_TOA_LO_ID + CIA_C_STAT_OFFSET + 1UL] << 8U) |
                                      (uint16_t)temp[STS_TOA_HI_ID - IP_TOA_LO_ID + CIA_C_STAT_OFFSET]) >> 7U);
        // Phase of arrival as computed from the STS 1 CIR (signed rad*2-12)
        diagnostics->stsPOA = (((uint16_t)temp[STS_TOA_HI_ID - IP_TOA_LO_ID + 2UL] << 8U) |
                                (uint16_t)temp[STS_TOA_HI_ID - IP_TOA_LO_ID + 1UL]);

        // RX status info for STS
        diagnostics->sts2RxStatus = ((((uint16_t)temp[STS1_TOA_HI_ID - IP_TOA_LO_ID + CIA_C_STAT_OFFSET + 1UL] << 8U) |
                                       (uint16_t)temp[STS_TOA_HI_ID - IP_TOA_LO_ID + CIA_C_STAT_OFFSET]) >> 7U);
        // Phase of arrival as computed from the STS 1 CIR (signed rad*2-12)
        diagnostics->sts2POA = (((uint16_t)temp[STS1_TOA_HI_ID - IP_TOA_LO_ID + 2UL] << 8U) |
                                 (uint16_t)temp[STS1_TOA_HI_ID - IP_TOA_LO_ID + 1UL]);

        // phase difference of the 2 STS POAs (signed in [1:-11])
        pdoa_calc = ((((uint16_t)temp[CIA_TDOA_1_PDOA_ID - IP_TOA_LO_ID + 3UL] << 8U) |
                       (uint16_t)temp[CIA_TDOA_1_PDOA_ID - IP_TOA_LO_ID + 2UL]) & 0x3FFFU);
        if ((pdoa_calc & INT14_SIGN_BIT_MASK) != 0U)
        {
            pdoa_calc = INT14_SIGN_POWN - pdoa_calc; // Get the absolute value
            pdoa_calc_signed = -(int16_t)pdoa_calc;
        }
        else
        {
            pdoa_calc_signed = (int16_t)pdoa_calc;
        }

        diagnostics->pdoa = pdoa_calc_signed;

        // Estimated xtal offset of remote device
        xtal_offset_calc = ((((uint16_t)temp[CIA_DIAG_0_ID - IP_TOA_LO_ID + 1UL] << 8U) |
                              (uint16_t)temp[CIA_DIAG_0_ID - IP_TOA_LO_ID]) & 0x1FFFU);
        diagnostics->xtalOffset = (int16_t)xtal_offset_calc;

        // Diagnostics common to both sequences
        diagnostics->ciaDiag1 = ((((uint32_t)temp[CIA_DIAG_1_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                                  ((uint32_t)temp[CIA_DIAG_1_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                                  ((uint32_t)temp[CIA_DIAG_1_ID - IP_TOA_LO_ID + 1UL] << 8UL) |
                                   (uint32_t)temp[CIA_DIAG_1_ID - IP_TOA_LO_ID])
                                  & 0x1FFFFFFFUL);

        if ((LOCAL_DATA(dw)->cia_diagnostic & (uint8_t)DW_CIA_DIAG_LOG_ALL) == 0U)
        {
            break; // break here is only logging minimal diagnostics
        }

        // IP
        // index [30:21] and amplitude [20:0] of peak sample in Ipatov sequence CIR
        diagnostics->ipatovPeak = ((((uint32_t)temp[IP_DIAG_0_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                                    ((uint32_t)temp[IP_DIAG_0_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                                    ((uint32_t)temp[IP_DIAG_0_ID - IP_TOA_LO_ID + 1UL] << 8UL) |
                                     (uint32_t)temp[IP_DIAG_0_ID - IP_TOA_LO_ID])
                                    & 0x7FFFFFFFUL);
        // channel area allows estimation [16:0] of channel power for the Ipatov sequence
        diagnostics->ipatovPower = ((((uint32_t)temp[IP_DIAG_1_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                                     ((uint32_t)temp[IP_DIAG_1_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                                     ((uint32_t)temp[IP_DIAG_1_ID - IP_TOA_LO_ID + 1UL] << 8UL) |
                                      (uint32_t)temp[IP_DIAG_1_ID - IP_TOA_LO_ID])
                                     & 0x1FFFFUL);
        // F1 for Ipatov sequence [21:0]
        diagnostics->ipatovF1 = ((((uint32_t)temp[IP_DIAG_2_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                                  ((uint32_t)temp[IP_DIAG_2_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                                  ((uint32_t)temp[IP_DIAG_2_ID - IP_TOA_LO_ID + 1UL] << 8UL) |
                                   (uint32_t)temp[IP_DIAG_2_ID - IP_TOA_LO_ID])
                                  & 0x3FFFFFUL);
        // F2 for Ipatov sequence [21:0]
        diagnostics->ipatovF2 = ((((uint32_t)temp[IP_DIAG_3_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                                  ((uint32_t)temp[IP_DIAG_3_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                                  ((uint32_t)temp[IP_DIAG_3_ID - IP_TOA_LO_ID + 1UL] << 8UL) |
                                   (uint32_t)temp[IP_DIAG_3_ID - IP_TOA_LO_ID])
                                  & 0x3FFFFFUL);
        diagnostics->ipatovF3 = ((((uint32_t)temp[IP_DIAG_4_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                                  ((uint32_t)temp[IP_DIAG_4_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                                  ((uint32_t)temp[IP_DIAG_4_ID - IP_TOA_LO_ID + 1UL] << 8UL) |
                                   (uint32_t)temp[IP_DIAG_4_ID - IP_TOA_LO_ID])
                                  & 0x3FFFFFUL); // F3 for Ipatov sequence [21:0]
        // First path index [15:0] for Ipatov sequence
        diagnostics->ipatovFpIndex = (((uint16_t)temp[IP_DIAG_8_ID - IP_TOA_LO_ID + 1UL] << 8U) |
                                       (uint16_t)temp[IP_DIAG_8_ID - IP_TOA_LO_ID]);
        // Number accumulated symbols [11:0] for Ipatov sequence
        diagnostics->ipatovAccumCount = ((((uint16_t)temp[IP_DIAG_12_ID - IP_TOA_LO_ID + 1UL] << 8U) |
                                           (uint16_t)temp[IP_DIAG_12_ID - IP_TOA_LO_ID]) & 0xFFFU);

        // STS 1
        diagnostics->stsPeak = ((((uint32_t)temp[STS_DIAG_0_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                                 ((uint32_t)temp[STS_DIAG_0_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                                 ((uint32_t)temp[STS_DIAG_0_ID - IP_TOA_LO_ID + 1UL] << 8UL) |
                                  (uint32_t)temp[STS_DIAG_0_ID - IP_TOA_LO_ID])
                                 & 0x3FFFFFFFUL); // index [29:21] and amplitude [20:0] of peak sample in STS CIR
        // channel area allows estimation of channel power for the STS
        diagnostics->stsPower = (((uint32_t)temp[STS_DIAG_1_ID - IP_TOA_LO_ID + 1UL] << 8UL) |
                                  (uint32_t)temp[STS_DIAG_1_ID - IP_TOA_LO_ID]);
        // F1 for STS [21:0]
        diagnostics->stsF1 = ((((uint32_t)temp[STS_DIAG_2_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                               ((uint32_t)temp[STS_DIAG_2_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                               ((uint32_t)temp[STS_DIAG_2_ID - IP_TOA_LO_ID + 1UL] << 8UL) |
                                (uint32_t)temp[STS_DIAG_2_ID - IP_TOA_LO_ID])
                               & 0x3FFFFFUL);
        // F2 for STS [21:0]
        diagnostics->stsF2 = ((((uint32_t)temp[STS_DIAG_3_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                               ((uint32_t)temp[STS_DIAG_3_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                               ((uint32_t)temp[STS_DIAG_3_ID - IP_TOA_LO_ID + 1UL] << 8UL) |
                                (uint32_t)temp[STS_DIAG_3_ID - IP_TOA_LO_ID])
                               & 0x3FFFFFUL);
        // offset_0xd - there are 0x6C bytes in 0xC0000 base before we enter 0xD0000
        // F3 for STS [21:0]
        diagnostics->stsF3 = ((((uint32_t)temp[STS_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd + 3UL] << 24UL) |
                               ((uint32_t)temp[STS_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd + 2UL] << 16UL) |
                               ((uint32_t)temp[STS_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8UL) |
                                (uint32_t)temp[STS_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd])
                               & 0x3FFFFFUL);
        // First path index [14:0] for STS
        diagnostics->stsFpIndex = ((((uint16_t)temp[STS_DIAG_8_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8U) |
                                     (uint16_t)temp[STS_DIAG_8_ID - STS_DIAG_4_ID + offset_0xd]) & 0x7FFFU);
        // Number accumulated symbols [11:0] for STS
        diagnostics->stsAccumCount = ((((uint16_t)temp[STS_DIAG_12_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8U) |
                                        (uint16_t)temp[STS_DIAG_12_ID - STS_DIAG_4_ID + offset_0xd]) & 0xFFFU);

        // STS 2
        diagnostics->sts2Peak = ((((uint32_t)temp[STS1_DIAG_0_ID - STS_DIAG_4_ID + offset_0xd + 3UL] << 24UL) |
                                  ((uint32_t)temp[STS1_DIAG_0_ID - STS_DIAG_4_ID + offset_0xd + 2UL] << 16UL) |
                                  ((uint32_t)temp[STS1_DIAG_0_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8UL) |
                                   (uint32_t)temp[STS1_DIAG_0_ID - STS_DIAG_4_ID + offset_0xd])
                                  & 0x3FFFFFFFUL); // index [29:21] and amplitude [20:0] of peak sample in STS CIR
        // channel area allows estimation of channel power for the STS
        diagnostics->sts2Power = (((uint32_t)temp[STS1_DIAG_1_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8UL) |
                                   (uint32_t)temp[STS1_DIAG_1_ID - STS_DIAG_4_ID + offset_0xd]);
        // F1 for STS [21:0]
        diagnostics->sts2F1 = ((((uint32_t)temp[STS1_DIAG_2_ID - STS_DIAG_4_ID + offset_0xd + 3UL] << 24UL) |
                                ((uint32_t)temp[STS1_DIAG_2_ID - STS_DIAG_4_ID + offset_0xd + 2UL] << 16UL) |
                                ((uint32_t)temp[STS1_DIAG_2_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8UL) |
                                 (uint32_t)temp[STS1_DIAG_2_ID - STS_DIAG_4_ID + offset_0xd])
                                & 0x3FFFFFUL);
        // F2 for STS [21:0]
        diagnostics->sts2F2 = ((((uint32_t)temp[STS1_DIAG_3_ID - STS_DIAG_4_ID + offset_0xd + 3UL] << 24UL) |
                                ((uint32_t)temp[STS1_DIAG_3_ID - STS_DIAG_4_ID + offset_0xd + 2UL] << 16UL) |
                                ((uint32_t)temp[STS1_DIAG_3_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8UL) |
                                 (uint32_t)temp[STS1_DIAG_3_ID - STS_DIAG_4_ID + offset_0xd])
                                & 0x3FFFFFUL);
        // F3 for STS [21:0]
        diagnostics->sts2F3 = ((((uint32_t)temp[STS1_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd + 3UL] << 24UL) |
                                ((uint32_t)temp[STS1_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd + 2UL] << 16UL) |
                                ((uint32_t)temp[STS1_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8UL) |
                                 (uint32_t)temp[STS1_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd])
                                & 0x3FFFFFUL);
        // First path index [14:0] for STS
        diagnostics->sts2FpIndex = ((((uint16_t)temp[STS1_DIAG_8_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8U) |
                                      (uint16_t)temp[STS1_DIAG_8_ID - STS_DIAG_4_ID + offset_0xd]) & 0x7FFFU);
        // Number accumulated symbols [11:0] for STS
        diagnostics->sts2AccumCount = ((((uint16_t)temp[STS1_DIAG_12_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8U) |
                                         (uint16_t)temp[STS1_DIAG_12_ID - STS_DIAG_4_ID + offset_0xd]) & 0xFFFU);
        break;
    }
}

/*! ---------------------------------------------------------------------------------------------------
* @brief This function reads the CIA diagnostics for an individual CIR/accumulator
*
* @param[in] dw: DW3000 chip descriptor handler.
* @param[out] diag: Pointer to a CIR diagnostics structure for a particular CIR/accumulator
* @param[in] acc_idx: CIR index (see @ref dwt_acc_idx_e)
*
* @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error (if input parameters not correct)
*/
static int ull_readdiagnostics_acc(dwchip_t *dw, dwt_cirdiags_t *cir_diag, dwt_acc_idx_e acc_idx)
{
    uint8_t temp[DB_MAX_DIAG_SIZE];
    uint32_t offset_0xd = STS_DIAG_3_LEN + STS_DIAG_3_ID - IP_TOA_LO_ID; // there are 0x6C bytes in 0xC0000 base before we enter 0xD0000
    uint16_t ip_length_min = IP_TOA_LO_IP_TOA_BIT_LEN + (IP_TOA_LO_LEN * 2U);
    dwt_error_e retVal = DWT_SUCCESS;

    if ((LOCAL_DATA(dw)->cia_diagnostic & (uint8_t)DW_CIA_DIAG_LOG_ALL) != 0U)
    {
        ull_readfromdevice(dw, IP_TOA_LO_ID, 0U, (uint16_t)offset_0xd, temp);               // read form 0xC0000 space  (108 bytes)
        ull_readfromdevice(dw, STS_DIAG_4_ID, 0U, (uint16_t)offset_0xd, &temp[offset_0xd]); // read from 0xD0000 space  (108 bytes)
    }
    else // even if other CIA_DIAG logging levels are set (e.g. MAX, MID or MIN) as double buffer is not used, we only log as if only MIN is set
    {
        ull_readfromdevice(dw, IP_TOA_LO_ID, 0U, ip_length_min, temp);
    }

    if (acc_idx > DWT_ACC_IDX_STS1_M)
    {
        retVal = DWT_ERROR;
    }
    else
    {
        if (acc_idx == DWT_ACC_IDX_IP_M)
        {
            // Index [30:21] and amplitude [20:0] of peak sample in Ipatov sequence CIR
            uint32_t reg_val = ((uint32_t)temp[IP_DIAG_0_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                        ((uint32_t)temp[IP_DIAG_0_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                        ((uint32_t)temp[IP_DIAG_0_ID - IP_TOA_LO_ID + 1UL] << 8UL)  |
                        (uint32_t)temp[IP_DIAG_0_ID - IP_TOA_LO_ID];
            cir_diag->peakAmp = reg_val & IP_DIAG_0_PEAKAMP_BIT_MASK;
            cir_diag->peakIndex = (uint16_t)((reg_val & IP_DIAG_0_PEAKLOC_BIT_MASK) >> IP_DIAG_0_PEAKLOC_BIT_OFFSET);
            // Channel area allows estimation [16:0] of channel power for the Ipatov sequence
            cir_diag->power = ((((uint32_t)temp[IP_DIAG_1_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                                ((uint32_t)temp[IP_DIAG_1_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                                ((uint32_t)temp[IP_DIAG_1_ID - IP_TOA_LO_ID + 1UL] << 8UL)  |
                                 (uint32_t)temp[IP_DIAG_1_ID - IP_TOA_LO_ID])
                                & 0x1FFFFUL);
            // F1 for Ipatov sequence [21:0]
            cir_diag->F1 = ((((uint32_t)temp[IP_DIAG_2_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                             ((uint32_t)temp[IP_DIAG_2_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                             ((uint32_t)temp[IP_DIAG_2_ID - IP_TOA_LO_ID + 1UL] << 8UL)  |
                              (uint32_t)temp[IP_DIAG_2_ID - IP_TOA_LO_ID])
                             & 0x3FFFFFUL);
            // F2 for Ipatov sequence [21:0]
            cir_diag->F2 = ((((uint32_t)temp[IP_DIAG_3_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                             ((uint32_t)temp[IP_DIAG_3_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                             ((uint32_t)temp[IP_DIAG_3_ID - IP_TOA_LO_ID + 1UL] << 8UL)  |
                              (uint32_t)temp[IP_DIAG_3_ID - IP_TOA_LO_ID])
                             & 0x3FFFFFUL);
            // F3 for Ipatov sequence [21:0]
            cir_diag->F3 = ((((uint32_t)temp[IP_DIAG_4_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                             ((uint32_t)temp[IP_DIAG_4_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                             ((uint32_t)temp[IP_DIAG_4_ID - IP_TOA_LO_ID + 1UL] << 8UL)  |
                              (uint32_t)temp[IP_DIAG_4_ID - IP_TOA_LO_ID])
                             & 0x3FFFFFUL);
            // First path index [15:0] for Ipatov sequence
            cir_diag->FpIndex = (((uint16_t)temp[IP_DIAG_8_ID - IP_TOA_LO_ID + 1U] << 8U) |
                                  (uint16_t)temp[IP_DIAG_8_ID - IP_TOA_LO_ID]);
            // Number accumulated symbols [11:0] for Ipatov sequence
            cir_diag->accumCount = ((((uint16_t)temp[IP_DIAG_12_ID - IP_TOA_LO_ID + 1U] << 8U) |
                                      (uint16_t)temp[IP_DIAG_12_ID - IP_TOA_LO_ID]) & 0xFFFU);
            // Early first path index [15:0] for Ipatov sequence
            cir_diag->EFpIndex = (((uint16_t)temp[IP_DIAG_9_ID - IP_TOA_LO_ID + 1U] << 8U) |
                                  (uint16_t)temp[IP_DIAG_9_ID - IP_TOA_LO_ID]);
            // Early first path confidence level [19:16] for Ipatov sequence
            cir_diag->EFpConfLevel = (uint8_t)((temp[IP_DIAG_9_ID - IP_TOA_LO_ID + 2U] >> 4) & 0x0FU);
            // First path threshold [21:0] for Ipatov sequence
            cir_diag->FpThreshold = ((((uint32_t)temp[IP_DIAG_11_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                             ((uint32_t)temp[IP_DIAG_11_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                             ((uint32_t)temp[IP_DIAG_11_ID - IP_TOA_LO_ID + 1UL] << 8UL)  |
                              (uint32_t)temp[IP_DIAG_11_ID - IP_TOA_LO_ID])
                             & 0x3FFFFFUL);
        }
        else if (acc_idx == DWT_ACC_IDX_STS0_M)
        {
            // STS1 index [29:21] and amplitude [20:0] of peak sample in STS CIR
            uint32_t reg_val = ((uint32_t)temp[STS_DIAG_0_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                        ((uint32_t)temp[STS_DIAG_0_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                        ((uint32_t)temp[STS_DIAG_0_ID - IP_TOA_LO_ID + 1UL] << 8UL)  |
                        (uint32_t)temp[STS_DIAG_0_ID - IP_TOA_LO_ID];
            cir_diag->peakAmp = reg_val & STS_DIAG_0_PEAKAMP_BIT_MASK;
            cir_diag->peakIndex = (uint16_t)((reg_val & STS_DIAG_0_PEAKLOC_BIT_MASK) >> STS_DIAG_0_PEAKLOC_BIT_OFFSET);
            // Channel area allows estimation of channel power for the STS1
            cir_diag->power = (((uint32_t)temp[STS_DIAG_1_ID - IP_TOA_LO_ID + 1UL] << 8UL) |
                                (uint32_t)temp[STS_DIAG_1_ID - IP_TOA_LO_ID]);
            // F1 for STS1 [21:0]
            cir_diag->F1 = ((((uint32_t)temp[STS_DIAG_2_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                             ((uint32_t)temp[STS_DIAG_2_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                             ((uint32_t)temp[STS_DIAG_2_ID - IP_TOA_LO_ID + 1UL] << 8UL)  |
                              (uint32_t)temp[STS_DIAG_2_ID - IP_TOA_LO_ID])
                             & 0x3FFFFFUL);
            // F2 for STS1 [21:0]
            cir_diag->F2 = ((((uint32_t)temp[STS_DIAG_3_ID - IP_TOA_LO_ID + 3UL] << 24UL) |
                             ((uint32_t)temp[STS_DIAG_3_ID - IP_TOA_LO_ID + 2UL] << 16UL) |
                             ((uint32_t)temp[STS_DIAG_3_ID - IP_TOA_LO_ID + 1UL] << 8UL)  |
                              (uint32_t)temp[STS_DIAG_3_ID - IP_TOA_LO_ID])
                              & 0x3FFFFFUL);
            // F3 for STS1 [21:0]
            cir_diag->F3 = ((((uint32_t)temp[STS_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd + 3UL] << 24UL) |
                             ((uint32_t)temp[STS_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd + 2UL] << 16UL) |
                             ((uint32_t)temp[STS_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8UL)  |
                              (uint32_t)temp[STS_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd])
                             & 0x3FFFFFUL);
            // First path index [14:0] for STS1
            cir_diag->FpIndex = ((((uint16_t)temp[STS_DIAG_8_ID - STS_DIAG_4_ID + offset_0xd + 1U] << 8U) |
                                  (uint16_t)temp[STS_DIAG_8_ID - STS_DIAG_4_ID + offset_0xd])
                                 & 0x7FFFU);
            // Number accumulated symbols [11:0] for STS1
            cir_diag->accumCount = ((((uint16_t)temp[STS_DIAG_12_ID - STS_DIAG_4_ID + offset_0xd + 1U] << 8U) |
                                      (uint16_t)temp[STS_DIAG_12_ID - STS_DIAG_4_ID + offset_0xd])
                                     & 0xFFFU);
            // Early first path index [14:0] for STS1
            cir_diag->EFpIndex =  ((((uint16_t)temp[STS_DIAG_9_ID - STS_DIAG_4_ID + offset_0xd + 1U] << 8U) |
                                  (uint16_t)temp[STS_DIAG_9_ID - STS_DIAG_4_ID + offset_0xd])
                                 & 0x7FFFU);
            // Early first path confidence level [19:16] for STS1
            cir_diag->EFpConfLevel = (uint8_t)((temp[STS_DIAG_9_ID - STS_DIAG_4_ID + offset_0xd + 2U] >> 4) & 0x0FU);
            // First path threshold [21:0] for STS1
            cir_diag->FpThreshold = ((((uint32_t)temp[STS_DIAG_11_ID - STS_DIAG_4_ID + offset_0xd + 3UL] << 24UL) |
                             ((uint32_t)temp[STS_DIAG_11_ID - STS_DIAG_4_ID + offset_0xd + 2UL] << 16UL) |
                             ((uint32_t)temp[STS_DIAG_11_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8UL)  |
                              (uint32_t)temp[STS_DIAG_11_ID - STS_DIAG_4_ID + offset_0xd])
                             & 0x3FFFFFUL);
        }
        else if (acc_idx == DWT_ACC_IDX_STS1_M)
        {
            // STS2 index [29:21] and amplitude [20:0] of peak sample in STS CIR
            uint32_t reg_val = ((uint32_t)temp[STS1_DIAG_0_ID - STS_DIAG_4_ID + offset_0xd + 3UL] << 24UL) |
                        ((uint32_t)temp[STS1_DIAG_0_ID - STS_DIAG_4_ID + offset_0xd + 2UL] << 16UL) |
                        ((uint32_t)temp[STS1_DIAG_0_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8UL)  |
                        (uint32_t)temp[STS1_DIAG_0_ID - STS_DIAG_4_ID + offset_0xd];
            cir_diag->peakAmp = reg_val & STS_DIAG_0_PEAKAMP_BIT_MASK;
            cir_diag->peakIndex = (uint16_t)((reg_val & STS_DIAG_0_PEAKLOC_BIT_MASK) >> STS_DIAG_0_PEAKLOC_BIT_OFFSET);
            // Channel area allows estimation of channel power for the STS2
            cir_diag->power = (((uint32_t)temp[STS1_DIAG_1_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8UL) |
                                (uint32_t)temp[STS1_DIAG_1_ID - STS_DIAG_4_ID + offset_0xd]);
            // F1 for STS2 [21:0]
            cir_diag->F1 = ((((uint32_t)temp[STS1_DIAG_2_ID - STS_DIAG_4_ID + offset_0xd + 3UL] << 24UL) |
                             ((uint32_t)temp[STS1_DIAG_2_ID - STS_DIAG_4_ID + offset_0xd + 2UL] << 16UL) |
                             ((uint32_t)temp[STS1_DIAG_2_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8UL)  |
                              (uint32_t)temp[STS1_DIAG_2_ID - STS_DIAG_4_ID + offset_0xd])
                             & 0x3FFFFFUL);
            // F2 for STS2 [21:0]
            cir_diag->F2 = ((((uint32_t)temp[STS1_DIAG_3_ID - STS_DIAG_4_ID + offset_0xd + 3UL] << 24UL) |
                             ((uint32_t)temp[STS1_DIAG_3_ID - STS_DIAG_4_ID + offset_0xd + 2UL] << 16UL) |
                             ((uint32_t)temp[STS1_DIAG_3_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8UL)  |
                              (uint32_t)temp[STS1_DIAG_3_ID - STS_DIAG_4_ID + offset_0xd])
                             & 0x3FFFFFUL);
            // F3 for STS2 [21:0]
            cir_diag->F3 = ((((uint32_t)temp[STS1_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd + 3UL] << 24UL) |
                             ((uint32_t)temp[STS1_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd + 2UL] << 16UL) |
                             ((uint32_t)temp[STS1_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8UL)  |
                              (uint32_t)temp[STS1_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd])
                             & 0x3FFFFFUL);
            // First path index [14:0] for STS2
            cir_diag->FpIndex = ((((uint16_t)temp[STS1_DIAG_8_ID - STS_DIAG_4_ID + offset_0xd + 1U] << 8U) |
                                  (uint16_t)temp[STS1_DIAG_8_ID - STS_DIAG_4_ID + offset_0xd])
                                 & 0x7FFFU);
            // Number accumulated symbols [11:0] for STS2
            cir_diag->accumCount = ((((uint16_t)temp[STS1_DIAG_12_ID - STS_DIAG_4_ID + offset_0xd + 1U] << 8U) |
                                      (uint16_t)temp[STS1_DIAG_12_ID - STS_DIAG_4_ID + offset_0xd])
                                     & 0xFFFU);
            // Early first path index [14:0] for STS2
            cir_diag->EFpIndex =  ((((uint16_t)temp[STS1_DIAG_9_ID - STS_DIAG_4_ID + offset_0xd + 1U] << 8U) |
                                  (uint16_t)temp[STS1_DIAG_9_ID - STS_DIAG_4_ID + offset_0xd])
                                 & 0x7FFFU);
            // Early first path confidence level [19:16] for STS2
            cir_diag->EFpConfLevel = (uint8_t)((temp[STS1_DIAG_9_ID - STS_DIAG_4_ID + offset_0xd + 2U] >> 4) & 0x0FU);
            // First path threshold [21:0] for STS2
            cir_diag->FpThreshold = ((((uint32_t)temp[STS1_DIAG_11_ID - STS_DIAG_4_ID + offset_0xd + 3UL] << 24UL) |
                             ((uint32_t)temp[STS1_DIAG_11_ID - STS_DIAG_4_ID + offset_0xd + 2UL] << 16UL) |
                             ((uint32_t)temp[STS1_DIAG_11_ID - STS_DIAG_4_ID + offset_0xd + 1UL] << 8UL)  |
                              (uint32_t)temp[STS1_DIAG_11_ID - STS_DIAG_4_ID + offset_0xd])
                             & 0x3FFFFFUL);
        }
        else
        {
            /* Nothing to do */
        }
    }

    return (int)retVal;
}

/*! ---------------------------------------------------------------------------------------------------
* @brief This API will return the RSSI  - UWB channel power
*        This API must be called only after initializing and configuring the driver
*
* @param[in] dw: DW3000 chip descriptor handler and receving some Rx data packets.
* @param[in] diag: Pointer to a CIR diagnostics structure for a particular CIR/accumulator
* @param[in] acc_idx: CIR index (see @ref dwt_acc_idx_e)
* @param[out] signal_strength: Channel power signal strength in q8.8 format (int16_t).
*
* @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
*/
static int ull_calculate_rssi(struct dwchip_s *dw, const dwt_cirdiags_t *diag, dwt_acc_idx_e acc_idx, int16_t *signal_strength)
{
    int r_code = (int)DWT_ERROR;
    if ((NULL != diag) && (NULL != signal_strength))
    {
        uint8_t dgc_decision = ull_get_dgcdecision(dw);
        bool is_sts = acc_idx !=  DWT_ACC_IDX_IP_M;
        uint32_t rx_pcode_u32 = dwt_read32bitoffsetreg(dw, CHAN_CTRL_ID, 0U) & CHAN_CTRL_RX_PCODE_BIT_MASK >> CHAN_CTRL_RX_PCODE_BIT_OFFSET;
        uint8_t rx_pcode = (uint8_t)rx_pcode_u32;
        *signal_strength = rsl_calculate_signal_power(
            (int32_t)(diag->power), RSL_QUANTIZATION_FACTOR, diag->accumCount, dgc_decision, rx_pcode, is_sts
        );
        r_code = (int)DWT_SUCCESS;
    }
    return r_code;
}

/*! ---------------------------------------------------------------------------------------------------
* @brief This API will return the first path signal power
*        This API must be called only after initializing and configuring the driver
*        and receving some RX data packets.
*
* @param[in] dw: DW3000 chip descriptor handler.
* @param[in] diag: Pointer to a CIR diagnostics structure for a particular CIR/accumulator
* @param[in] acc_idx: CIR index (see @ref dwt_acc_idx_e)
* @param[out] signal_strength: First path signal strength in q8.8 format (int16_t).
*
* @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
*/
static int ull_calculate_first_path_power(struct dwchip_s *dw, const dwt_cirdiags_t *diag, dwt_acc_idx_e acc_idx, int16_t *signal_strength)
{
    int r_code = (int)DWT_ERROR;
    if ((NULL != diag) && (NULL != signal_strength))
    {
        uint8_t dgc_decision = ull_get_dgcdecision(dw);
        bool is_sts = acc_idx !=  DWT_ACC_IDX_IP_M;
        uint8_t rx_pcode = (uint8_t)((dwt_read32bitoffsetreg(dw, CHAN_CTRL_ID, 0U) & CHAN_CTRL_RX_PCODE_BIT_MASK) >> CHAN_CTRL_RX_PCODE_BIT_OFFSET);
        *signal_strength = rsl_calculate_first_path_power(
            diag->F1, diag->F2, diag->F3, diag->accumCount, dgc_decision, rx_pcode, is_sts
        );
        r_code = (int)DWT_SUCCESS;
    }
    return r_code;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the DGC_DECISION index when RX_TUNING (DGC) is enabled, this value is used to adjust the
 *        RX level and first path FP level estimation
 *        See also dwt_calculate_rssi(), dwt_calculate_first_path_power()
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @return The index value to be used in RX level and FP level formulas
 */
static uint8_t ull_get_dgcdecision(dwchip_t *dw)
{
    return ((dwt_read8bitoffsetreg(dw, DGC_DBG_ID, 3U) & 0x70U) >> 4U);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the TX timestamp (adjusted with the programmed antenna delay)
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] timestamp: A pointer to a 5-byte buffer which will store the read TX timestamp time
 *
 * @return  None
 */
static void ull_readtxtimestamp(dwchip_t *dw, uint8_t *timestamp)
{
    ull_readfromdevice(dw, (uint32_t)TX_TIME_LO_ID, 0U, TX_TIME_TX_STAMP_LEN, timestamp); // Read bytes directly into buffer
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the high 32-bits of the TX timestamp (adjusted with the programmed antenna delay)
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return High 32-bits of 40-bit TX timestamp
 */
static uint32_t ull_readtxtimestamphi32(dwchip_t *dw)
{
    return dwt_read32bitoffsetreg(dw, TX_TIME_LO_ID, 1U); // Offset is 1 to get the 4 upper bytes out of 5
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the low 32-bits of the TX timestamp (adjusted with the programmed antenna delay)
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return Low 32-bits of 40-bit TX timestamp
 */
static uint32_t ull_readtxtimestamplo32(dwchip_t *dw)
{
    return dwt_read32bitreg(dw, TX_TIME_LO_ID); // Read TX TIME as a 32-bit register to get the 4 lower bytes out of 5
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the PDOA result (Phase Difference On Arrival).
 *        It is the phase difference between either the Ipatov and STS phase-of-arrival (POA), or
 *        the two STS POAs, depending on the PDOA mode of operation.
 *
 * @note To convert to degrees: float pdoa_deg = ((float)pdoa / (1 << 11)) * 180 / M_PI
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return The PDOA result signed 16-bit number, ([1:-11] radian units)
 */
static int16_t ull_readpdoa(dwchip_t *dw)
{
    uint16_t pdoa;
    int16_t pdoa_signed;

    switch ((dwt_dbl_buff_conf_e)LOCAL_DATA(dw)->dblbuffon)
    // check if in double buffer mode and if so which buffer host is currently accessing
    {
    case DBL_BUFF_ACCESS_BUFFER_1:
        //!!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
        pdoa = dwt_read16bitoffsetreg(dw, INDIRECT_POINTER_B_ID, (uint16_t)(BUF1_PDOA - BUF1_RX_FINFO) + 2U) &
               (uint16_t)(CIA_TDOA_1_PDOA_PDOA_BIT_MASK >> 16UL);
        break;
    case DBL_BUFF_ACCESS_BUFFER_0:
        pdoa = dwt_read16bitoffsetreg(dw, BUF0_PDOA, 2U) & (uint16_t)(CIA_TDOA_1_PDOA_PDOA_BIT_MASK >> 16UL);
        break;
    default:
        pdoa = dwt_read16bitoffsetreg(dw, CIA_TDOA_1_PDOA_ID, 2U) &
               (uint16_t)(CIA_TDOA_1_PDOA_PDOA_BIT_MASK >> 16UL); // phase difference of the 2 POAs
        break;
    }

    if ((pdoa & INT14_SIGN_BIT_MASK) != 0U)
    {
        pdoa = INT14_SIGN_POWN - pdoa; // get absolute value
        pdoa_signed = -(int16_t)pdoa;
    }
    else
    {
        pdoa_signed = (int16_t)pdoa;
    }

    return pdoa_signed;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function is used to read the TDOA (Time Difference On Arrival). The TDOA value that is read from the
 * register is 41-bits in length. However, 6 bytes (or 48-bits) are read from the register. The remaining 7-bits at
 * the 'top' of the 6 bytes that are not part of the TDOA value should be set to zero and should not interfere with
 * rest of the 41-bit value. However, there is no harm in masking the returned value.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] tdoa: time difference of arrival - buffer of 6 bytes that will be filled with TDOA value by calling this function
 *
 * @return  None
 */
static void ull_readtdoa(dwchip_t *dw, uint8_t *tdoa)
{
    // timestamp difference of the 2 cipher RX timestamps
    ull_readfromdevice(dw, CIA_TDOA_0_ID, 0U, CIA_TDOA_LEN, tdoa);
    tdoa[5] &= 0x01U; // TDOA value is 41 bits long. You will need to read 6 bytes and mask the highest byte with 0x01
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the RX timestamp (adjusted time of arrival)
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] timestamp: A pointer to a 5-byte buffer which will store the read RX timestamp.
 *                   The timestamp buffer will contain the value after the function call
 *
 * @return  None
 */

static void ull_readrxtimestamp(dwchip_t *dw, uint8_t *timestamp)
{
    switch ((dwt_dbl_buff_conf_e)LOCAL_DATA(dw)->dblbuffon)
    // check if in double buffer mode and if so which buffer host is currently accessing
    {
    case DBL_BUFF_ACCESS_BUFFER_1:
        //!!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
        ull_readfromdevice(dw, INDIRECT_POINTER_B_ID, (uint16_t)(BUF1_RX_TIME - BUF1_RX_FINFO), RX_TIME_RX_STAMP_LEN, timestamp);
        break;
    case DBL_BUFF_ACCESS_BUFFER_0:
        ull_readfromdevice(dw, BUF0_RX_TIME, 0U, RX_TIME_RX_STAMP_LEN, timestamp);
        break;
    default:
        ull_readfromdevice(dw, (uint32_t)RX_TIME_0_ID, 0U, RX_TIME_RX_STAMP_LEN, timestamp); // Get the adjusted time of arrival
        break;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the RX timestamp (unadjusted time of arrival)
 * @note The RX raw timestamp is read into the 5-byte array and the lowest byte is always 0x0.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] timestamp: A pointer to a 5-byte buffer which will store the read RX timestamp.
 *                   The timestamp buffer will contain the value after the function call
 *
 * @return  None
 */

static void ull_readrxtimestampunadj(dwchip_t *dw, uint8_t *timestamp)
{
    timestamp[0] = 0U;
    ull_readfromdevice(dw, RX_TIME_RAW_ID, 0U, RX_TIME_RX_STAMP_LEN - 1U, &timestamp[1]);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the RX timestamp (adjusted time of arrival) w.r.t. Ipatov CIR
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] timestamp: A pointer to a 5-byte buffer which will store the read RX timestamp.
 *                   The timestamp buffer will contain the value after the function call
 *
 * @return  None
 */
static void ull_readrxtimestamp_ipatov(dwchip_t *dw, uint8_t *timestamp)
{
    switch ((dwt_dbl_buff_conf_e)LOCAL_DATA(dw)->dblbuffon)
    // check if in double buffer mode and if so which buffer host is currently accessing
    {
    case DBL_BUFF_ACCESS_BUFFER_1:
        //!!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
        ull_readfromdevice(dw, INDIRECT_POINTER_B_ID, (uint16_t)(BUF1_IP_TS - BUF1_RX_FINFO), CIA_I_RX_TIME_LEN, timestamp);
        break;
    case DBL_BUFF_ACCESS_BUFFER_0:
        ull_readfromdevice(dw, BUF0_IP_TS, 0U, CIA_I_RX_TIME_LEN, timestamp);
        break;
    default:
        ull_readfromdevice(dw, IP_TOA_LO_ID, 0U, CIA_I_RX_TIME_LEN, timestamp); // Get the adjusted time of arrival w.r.t. Ipatov CIR
        break;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the RX timestamp (adjusted time of arrival) w.r.t. STS CIR
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] timestamp: A pointer to a 5-byte buffer which will store the read RX timestamp.
 *                   The timestamp buffer will contain the value after the function call
 *
 * @return  None
 */

static void ull_readrxtimestamp_sts(dwchip_t *dw, uint8_t *timestamp)
{
    switch ((dwt_dbl_buff_conf_e)LOCAL_DATA(dw)->dblbuffon)
    // check if in double buffer mode and if so which buffer host is currently accessing
    {
    case DBL_BUFF_ACCESS_BUFFER_1:
        //!!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
        ull_readfromdevice(dw, INDIRECT_POINTER_B_ID, (uint16_t)(BUF1_STS_TS - BUF1_RX_FINFO), CIA_C_RX_TIME_LEN, timestamp);
        break;
    case DBL_BUFF_ACCESS_BUFFER_0:
        ull_readfromdevice(dw, BUF0_STS_TS, 0U, CIA_C_RX_TIME_LEN, timestamp);
        break;
    default:
        ull_readfromdevice(dw, STS_TOA_LO_ID, 0U, CIA_C_RX_TIME_LEN, timestamp); // Get the adjusted time of arrival w.r.t. STS CIR
        break;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the high 32-bits of the RX timestamp.
 *        The adjusted timestamp is read (adjusted with the programmed antenna delay)
 *
 * @note This should not be used when RX double buffer mode is enabled. Following APIs to read RX timestamp should be
 * used:  dwt_readrxtimestamp_ipatov() or dwt_readrxtimestamp_sts() or dwt_readrxtimestamp()
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return High 32-bits of RX timestamp
 */
static uint32_t ull_readrxtimestamphi32(dwchip_t *dw)
{
    return dwt_read32bitoffsetreg(dw, RX_TIME_0_ID, 1U); // Offset is 1 to get the 4 upper bytes out of 5 byte tiemstamp
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the low 32-bits of the RX timestamp.
 *        The adjusted timestamp is read (adjusted with the programmed antenna delay)
 *
 * @note This should not be used when RX double buffer mode is enabled. Following APIs to read RX timestamp should be
 * used:  dwt_readrxtimestamp_ipatov() or dwt_readrxtimestamp_sts() or dwt_readrxtimestamp()
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return Low 32-bits of RX timestamp
 */
static uint32_t ull_readrxtimestamplo32(dwchip_t *dw)
{
    return dwt_read32bitreg(dw, RX_TIME_0_ID); // Read RX TIME as a 32-bit register to get the 4 lower bytes out of 5 byte timestamp
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the high 32-bits of the UWB IC system time
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @return High 32-bits of system time timestamp
 */
static uint32_t ull_readsystimehi32(dwchip_t *dw)
{
    return dwt_read32bitreg(dw, SYS_TIME_ID);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the UWB IC system time
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] timestamp: A pointer to a 4-byte buffer which will store the read system time.
 *                   The timestamp buffer will contain the value after the function call
 *
 * @return  None
 */
static void ull_readsystime(dwchip_t *dw, uint8_t *timestamp)
{
    ull_readfromdevice(dw, (uint32_t)SYS_TIME_ID, 0U, SYS_TIME_LEN, timestamp);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to enable the frame filtering. The default option is to
 * accept any data and ACK frames with correct destination address.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] enabletype: A bitmask which enables/disables the frame filtering and configures 802.15.4 type
 *       DWT_FF_ENABLE_802_15_4      0x2             - use 802.15.4 filtering rules
 *       DWT_FF_DISABLE              0x0             - disable FF
 * @param[in] filtermode: A bitmask which configures the frame filtering options according to
 *       DWT_FF_BEACON_EN            0x001           - beacon frames allowed
 *       DWT_FF_DATA_EN              0x002           - data frames allowed
 *       DWT_FF_ACK_EN               0x004           - ack frames allowed
 *       DWT_FF_MAC_EN               0x008           - mac control frames allowed
 *       DWT_FF_RSVD_EN              0x010           - reserved frame types allowed
 *       DWT_FF_MULTI_EN             0x020           - multipurpose frames allowed
 *       DWT_FF_FRAG_EN              0x040           - fragmented frame types allowed
 *       DWT_FF_EXTEND_EN            0x080           - extended frame types allowed
 *       DWT_FF_COORD_EN             0x100           - behave as coordinator (can receive frames with no dest address (PAN ID has to match))
 *       DWT_FF_IMPBRCAST_EN         0x200           - allow MAC implicit broadcast
 *
 * @return  None
 */
static void ull_configureframefilter(dwchip_t *dw, uint16_t enabletype, uint16_t filtermode)
{
    if (enabletype == (uint16_t)DWT_FF_ENABLE_802_15_4)
    {
        dwt_or8bitoffsetreg(dw, SYS_CFG_ID, 0U, SYS_CFG_FFEN_BIT_MASK);
        dwt_write16bitoffsetreg(dw, ADR_FILT_CFG_ID, 0U, filtermode);
    }
    else
    {
        // Disable frame filter
        dwt_and8bitoffsetreg(dw, SYS_CFG_ID, 0U, (uint8_t)~(SYS_CFG_FFEN_BIT_MASK));
        // Clear the configuration
        dwt_write16bitoffsetreg(dw, ADR_FILT_CFG_ID, 0U, 0x0U);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to set the PAN ID
 *        Used when frame filtering is enabled with dwt_configureframefilter()
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] panID: This is the network PAN ID
 *
 * @return  None
 */
static void ull_setpanid(dwchip_t *dw, uint16_t panID)
{
    // PAN ID is high 16-bits of register
    dwt_write16bitoffsetreg(dw, PANADR_ID, PANADR_PAN_ID_BYTE_OFFSET, panID);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to set 16-bit (short) address
 *        Used when frame filtering is enabled with dwt_configureframefilter()
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] shortAddress: This sets the 16-bit short address
 *
 * @return  None
 */
static void ull_setaddress16(dwchip_t *dw, uint16_t shortAddress)
{
    // Short address into low 16 bits
    dwt_write16bitoffsetreg(dw, PANADR_ID, PANADR_SHORTADDR_BIT_OFFSET, shortAddress);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to set the EUI 64-bit (long) address
 *        Used when frame filtering is enabled with dwt_configureframefilter()
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] eui64: This is the pointer to a buffer that contains the 64-bit address
 *
 * @return  None
 */
static void ull_seteui(dwchip_t *dw, uint8_t *eui64)
{
    ull_writetodevice(dw, EUI_64_LO_ID, 0U, 0x8U, eui64);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to get the EUI 64-bit from the DW IC
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] eui64: This is the pointer to a buffer that contains the 64-bit address
 *
 * @return  None
 */
static void ull_geteui(dwchip_t *dw, uint8_t *eui64)
{
    ull_readfromdevice(dw, EUI_64_LO_ID, 0U, 0x8U, eui64);
}

 /*! ------------------------------------------------------------------------------------------------------------------
 * @brief This call enables the auto-ACK feature. If the responseDelayTime (parameter) is 0, the ACK will be sent a.s.a.p.,
 * otherwise it will be sent with a programmed delay (in symbols), max is 255.
 * @note Needs to have frame filtering enabled as well.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] responseDelayTime: If non-zero the ACK is sent after this delay, max is 255.
 * @param[in] enable: Enables or disables the auto-ACK feature
 *
 * @return None
 */
static void ull_enableautoack(dwchip_t *dw, uint8_t responseDelayTime, int32_t enable)
{
    // Set auto ACK reply delay
    dwt_write8bitoffsetreg(dw, ACK_RESP_ID, 3U, responseDelayTime); // In symbols

    // Enable AUTO ACK
    if (enable != 0)
    {
        dwt_or32bitoffsetreg(dw, SYS_CFG_ID, 0U, (uint32_t)SYS_CFG_AUTO_ACK_BIT_MASK | SYS_CFG_FAST_AAT_EN_BIT_MASK); // set the AUTO_ACK bit
    }
    else
    {
        dwt_and16bitoffsetreg(dw, SYS_CFG_ID, 0U, (uint16_t)(~SYS_CFG_AUTO_ACK_BIT_MASK)); // clear the AUTO_ACK bit
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function is used to write a 16-bit address to a desired Low-Energy device (LE) address.
 * It is used for the frame pending to function when the correct bits are set in the frame filtering configuration
 * via the dwt_configureframefilter(). See dwt_configureframefilter() for more details.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] addr: The address value to be written to the selected LE register
 * @param[in] leIndex: Low-Energy device (LE) address to write to, see @ref dwt_le_addresses_e
 *
 * @return None
 */
static void ull_configure_le_address(dwchip_t *dw, uint16_t addr, int32_t leIndex)
{
    switch (leIndex)
    {
    case 0:
        dwt_write16bitoffsetreg(dw, LE_PEND_01_ID, 0U, addr);
        break;
    case 1:
        dwt_write16bitoffsetreg(dw, LE_PEND_01_ID, 2U, addr);
        break;
    case 2:
        dwt_write16bitoffsetreg(dw, LE_PEND_23_ID, 0U, addr);
        break;
    case 3:
        dwt_write16bitoffsetreg(dw, LE_PEND_23_ID, 2U, addr);
        break;
    default:
        /* Invalid index */
        break;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read from AON memory
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] aon_address: This is the address of the memory location to read
 *
 * @return 8-bits read from given AON memory address
 */
static uint8_t ull_aon_read(dwchip_t *dw, uint16_t aon_address)
{
    dwt_write16bitoffsetreg(dw, AON_ADDR_ID, 0x0U, aon_address); // Set short AON address for read
    dwt_write8bitoffsetreg(dw, AON_CTRL_ID, 0x0U, (AON_CTRL_DCA_ENAB_BIT_MASK | AON_CTRL_DCA_READ_EN_BIT_MASK));
    dwt_write8bitoffsetreg(dw, AON_CTRL_ID, 0x0U, 0x0U);   // Clear all enabled bits
    return dwt_read8bitoffsetreg(dw, AON_RDATA_ID, 0x0U); // Return the data that was read
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to write to AON memory
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] aon_address: This is the address of the memory location to write
 * @param[in] aon_write_data: This is the data to write
 *
 * @return  None
 *
 */
static void ull_aon_write(dwchip_t *dw, uint16_t aon_address, uint8_t aon_write_data)
{
    uint8_t temp = 0U;

    if (aon_address >= 0x100U)
    {
        temp = AON_CTRL_DCA_WRITE_HI_EN_BIT_MASK;
    }
    dwt_write16bitoffsetreg(dw, AON_ADDR_ID, 0x0U, (uint8_t)aon_address);                                                // Set AON address for write
    dwt_write8bitoffsetreg(dw, AON_WDATA_ID, 0x0U, aon_write_data);                                                      // Set write data
    dwt_write8bitoffsetreg(dw, AON_CTRL_ID, 0x0U, (temp | AON_CTRL_DCA_ENAB_BIT_MASK | AON_CTRL_DCA_WRITE_EN_BIT_MASK)); // Enable write
    dwt_write8bitoffsetreg(dw, AON_CTRL_ID, 0x0U, 0x0U);                                                                 // Clear all enabled bits
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the OTP data from given address into provided array
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] address: This is the OTP address to read from
 * @param[out] array: This is the pointer to the array into which to read the data
 * @param[in] length: This is the number of 32-bit words to read (array needs to be at least this length)
 *
 * @return  None
 */
static void ull_otpread(dwchip_t *dw, uint16_t address, uint32_t *array, uint8_t length)
{
    for (uint8_t i = 0U; i < length; i++)
    {
        array[i] = dwt_otpreadword32(dw, address + (uint16_t)i);
    }

    return;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function reads 32-bit words from the OTP memory.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] address: The address to read from
 *
 * @return The read data
 */
static uint32_t dwt_otpreadword32(dwchip_t *dw, uint16_t address)
{
    uint32_t ret_data = 0UL;

    // Set manual access mode
    dwt_write16bitoffsetreg(dw, OTP_CFG_ID, 0U, OTP_CFG_OTP_MAN_CTR_EN_BIT_MASK);
    // set the address
    dwt_write16bitoffsetreg(dw, OTP_ADDR_ID, 0U, address);
    // Assert the read strobe
    dwt_write16bitoffsetreg(dw, OTP_CFG_ID, 0U, OTP_CFG_OTP_READ_BIT_MASK);
    // attempt a read from OTP address
    ret_data = dwt_read32bitoffsetreg(dw, OTP_RDATA_ID, 0U);

    // Return the 32bit of read data
    return ret_data;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief For each value to send to OTP bloc, following two register writes are required as shown below
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] val: 16-bit value to write to the OTP block
 *
 * @return None
 */
static void dwt_otp_write_wdata_id_reg(dwchip_t *dw, int16_t val)
{
    /* Pull the CS high to enable user interface for programming */
    /* 'val' is ignored in this instance by the OTP block */
    dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, 0x0200U | (uint16_t)val);
    /* Send the relevant command to the OTP block */
    dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, 0x0000U | (uint16_t)val);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Static function to program the OTP memory.
 * @note the address is only 11-bits long.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param data[in]: Data to write to given address
 * @param address[in]: Address to write the data to
 *
 * @return  None
 */
static void dwt_otpprogword32(dwchip_t *dw, uint32_t data, uint16_t address)
{
    // uint32_t rd_buf;
    uint16_t wr_buf[4];
    // uint8_t otp_done;

    // Read current register value
    uint32_t ldo_tune = dwt_read32bitoffsetreg(dw, LDO_TUNE_HI_ID, 0U);
    // Set VDDHV_TX LDO to max
    dwt_or32bitoffsetreg(dw, LDO_TUNE_HI_ID, 0U, LDO_TUNE_HI_LDO_HVAUX_TUNE_BIT_MASK);

    // configure mode for programming
    dwt_write16bitoffsetreg(dw, OTP_CFG_ID, 0U, 0x10U | OTP_CFG_OTP_WRITE_MR_BIT_MASK);

    // Select fast programming
    dwt_otp_write_wdata_id_reg(dw, 0x0025);

    // Apply instruction to write the address
    dwt_otp_write_wdata_id_reg(dw, 0x0002);
    dwt_otp_write_wdata_id_reg(dw, 0x01fc);

    // Now sending the OTP address data (2 bytes)
    wr_buf[0] = 0x0100U | (address & 0xffU);
    dwt_otp_write_wdata_id_reg(dw, (int16_t)wr_buf[0]);

    // Write data (upper byte of address)
    dwt_otp_write_wdata_id_reg(dw, 0x0100);

    // Clean up
    dwt_otp_write_wdata_id_reg(dw, 0x0000);

    // Apply instruction  to write data
    dwt_otp_write_wdata_id_reg(dw, 0x0002);
    dwt_otp_write_wdata_id_reg(dw, 0x01c0);

    // Write the data
    wr_buf[0] = 0x100U | (uint16_t)((data >> 24UL) & 0xffUL);
    wr_buf[1] = 0x100U | (uint16_t)((data >> 16UL) & 0xffUL);
    wr_buf[2] = 0x100U | (uint16_t)((data >> 8UL) & 0xffUL);
    wr_buf[3] = 0x100U | (uint16_t)(data & 0xffUL);
    dwt_otp_write_wdata_id_reg(dw, (int16_t)wr_buf[3]);
    dwt_otp_write_wdata_id_reg(dw, (int16_t)wr_buf[2]);
    dwt_otp_write_wdata_id_reg(dw, (int16_t)wr_buf[1]);
    dwt_otp_write_wdata_id_reg(dw, (int16_t)wr_buf[0]);

    // Clean up
    dwt_otp_write_wdata_id_reg(dw, 0x0000);

    // Enter prog mode
    dwt_otp_write_wdata_id_reg(dw, 0x003a);
    dwt_otp_write_wdata_id_reg(dw, 0x01ff);
    dwt_otp_write_wdata_id_reg(dw, 0x010a);
    // Clean up
    dwt_otp_write_wdata_id_reg(dw, 0x0000);

     // Enable state/status output
     // dwt_otp_write_wdata_id_reg(0x003a);
     // dwt_otp_write_wdata_id_reg(0x01bf);
     // dwt_otp_write_wdata_id_reg(0x0100);

    // Start prog mode
    dwt_otp_write_wdata_id_reg(dw, 0x003a);
    dwt_otp_write_wdata_id_reg(dw, 0x0101);
    dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, 0x0002U); // Different to previous one
    dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, 0x0000U);

    /*
     read status after program command.
     The for loop will exit once the status indicates programming is complete or if it reaches the max 1000 iterations.
     1000 is more than sufficient for max OTP programming delay and max supported DW3000 SPI rate.
     Instead a delay of 2ms (as commented out below) can be used.
     Burn time is about 1.76ms
     */

    /*uint16_t i;

     for (i = 0U; i < 1000U; i++)
     {
     rd_buf = dwt_read32bitoffsetreg(dw, OTP_STATUS_ID, 0U);

     if (!(rd_buf & OTP_STATUS_OTP_PROG_DONE_BIT_MASK))
     {
     #ifdef DWT_DEBUG_PRINT
     printf("otp status %x\n", rd_buf);
     #endif
     break;
     }
     }*/

    deca_sleep(2U); // Uncomment this command if you don't want to use the loop above. It will take more time than the loop above.

    // Stop prog mode
    dwt_otp_write_wdata_id_reg(dw, 0x003a);
    dwt_otp_write_wdata_id_reg(dw, 0x0102);
    dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, 0x0002U); // Different to previous one
    dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, 0x0000U);

    // configure mode for reading
    dwt_write16bitoffsetreg(dw, OTP_CFG_ID, 0U, 0x0000U);

    // Restore LDO tune register
    dwt_write32bitoffsetreg(dw, LDO_TUNE_HI_ID, 0U, ldo_tune);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to program 32-bit value into the devices OTP memory.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] value: This is the 32-bit value to be programmed into OTP
 * @param[in] address: This is the 16-bit OTP address into which the 32-bit value is programmed
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
 */
static int32_t ull_otpwriteandverify(dwchip_t *dw, uint32_t value, uint16_t address)
{
    dwt_error_e retVal = DWT_ERROR;
    // program the word
    dwt_otpprogword32(dw, value, address);

    // check it is programmed correctly
    if (dwt_otpreadword32(dw, address) == value)
    {
        retVal = DWT_SUCCESS;
    }

    return (int32_t)retVal;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to program 32-bit value into the device's OTP memory, it will not validate the word was written correctly
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] value: This is the 32-bit value to be programmed into OTP
 * @param[in] address: This is the 16-bit OTP address into which the 32-bit value is programmed
 *
 * @return @ref DWT_SUCCESS
 */
static int32_t ull_otpwrite(dwchip_t *dw, uint32_t value, uint16_t address)
{
    // program the word
    dwt_otpprogword32(dw, value, address);

    return (int32_t)DWT_SUCCESS;
}

/*!
 * @brief This function puts the device into deep sleep or sleep. dwt_configuresleep() should be called first
 * to configure the sleep and on-wake/wake-up parameters
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] idle_rc: If this is set to @ref DWT_DW_IDLE_RC, the auto INIT2IDLE bit will be cleared prior to going to sleep
 *                  thus after wakeup device will stay in the IDLE_RC state (PLL will not be enabled).
 *
 * @note With DW3000 devices, it is recommended to use @ref DWT_DW_IDLE_RC rather than @ref DWT_DW_IDLE to speed up wake up time.
 *
 * @return None
 */
static void ull_entersleep(dwchip_t *dw, int32_t idle_rc)
{
    // OTP low power mode
    ull_dis_otp_ips(dw, 1);

    // clear auto INIT2IDLE bit if required
    if (idle_rc == (int32_t)DWT_DW_IDLE_RC)
    {
        dwt_and8bitoffsetreg(dw, SEQ_CTRL_ID, 0x1U, (uint8_t)(~(SEQ_CTRL_AINIT2IDLE_BIT_MASK >> 8U)));
    }

    // Copy config to AON - upload the new configuration
    dwt_write8bitoffsetreg(dw, AON_CTRL_ID, 0U, 0U);
    dwt_write8bitoffsetreg(dw, AON_CTRL_ID, 0U, AON_CTRL_ARRAY_SAVE_BIT_MASK);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Calibrates the local oscillator (LP OSC) as its frequency can vary between 15 kHz and 34 kHz depending on temp and voltage
 *
 * @note This function needs to be run before dwt_configuresleepcnt(), so that we know what the counter units are.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return The number of XTAL cycles per low-power oscillator cycle. LP OSC frequency = 38.4 MHz/return value
 */
static uint16_t ull_calibratesleepcnt(dwchip_t *dw)
{
    uint16_t temp = 0U;
    uint8_t temp2 = 0U;

    // Enable VDDPLL for reference clock
    dwt_or8bitoffsetreg(dw, LDO_CTRL_ID, 0U, LDO_CTRL_LDO_VDDPLL_EN_BIT_MASK);
    // Clear any previous cal settings
    temp2 = ull_aon_read(dw, (uint16_t)AON_SLPCNT_CAL_CTRL) & 0xE0U;
    ull_aon_write(dw, (uint16_t)AON_SLPCNT_CAL_CTRL, temp2);
    // Run cal
    ull_aon_write(dw, (uint16_t)AON_SLPCNT_CAL_CTRL, temp2 | 0x04U);
    deca_sleep(2U); // need to wait for at least 1 LP OSC period at slowest frequency of 15kHz =~ 66 us
    // Read the Cal value from AON
    temp = ull_aon_read(dw, (uint16_t)AON_SLPCNT_CAL_LO);
    temp |= ((uint16_t)ull_aon_read(dw, (uint16_t)AON_SLPCNT_CAL_HI) << 8U);
    // Clear cal
    ull_aon_write(dw, (uint16_t)AON_SLPCNT_CAL_CTRL, temp2);
    // Disable VDDPLL for reference clock
    dwt_and8bitoffsetreg(dw, LDO_CTRL_ID, 0U, (uint8_t)(~LDO_CTRL_LDO_VDDPLL_EN_BIT_MASK));
    return temp;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Sets the sleep counter to new value, this function programs the high 16-bits of the 28-bit counter
 *
 * @note This function needs to be run before dwt_configuresleep(), also the SPI frequency has to be < 3MHz
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] sleepcnt: This it value of the sleep counter to program
 *
 * @return  None
 */
static void ull_configuresleepcnt(dwchip_t *dw, uint16_t sleepcnt)
{
    ull_aon_write(dw, (uint16_t)AON_SLPCNT_LO, (uint8_t)sleepcnt);
    ull_aon_write(dw, (uint16_t)AON_SLPCNT_HI, (uint8_t)(sleepcnt >> 8U));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief configures the device for both DEEP_SLEEP and SLEEP modes, and on-wake mode
 * i.e. before entering the sleep, the device should be programmed for TX or RX, then upon "waking up" the TX/RX settings
 * will be preserved and the device can immediately perform the desired action TX/RX
 *
 * @note e.g. Tag operation - after deep sleep, the device needs to just load the TX buffer and send the frame
 * @note For optimal performance the PGF calibration on wakeup should not be used. The API will force it to off.
 *
 *
 *      mode: on-wake parameters, @ref dwt_on_wake_param_e
 *      DWT_PGFCAL       0x0800  - Should not be used for optimal performance
 *      DWT_GOTORX       0x0200
 *      DWT_GOTOIDLE     0x0100
 *      DWT_SEL_OPS      0x0040 | 0x0080
 *      DWT_LOADOPS      0x0020
 *      DWT_LOADLDO      0x0010
 *      DWT_LOADDGC      0x0008
 *      DWT_LOADBIAS     0x0004
 *      DWT_RUNSAR       0x0002
 *      DWT_CONFIG       0x0001 - Download the AON array into the HIF (configuration download)
 *
 *      wake: wake up parameters, @ref dwt_wkup_param_e
 *      DWT_SLP_CNT_RPT  0x40 - sleep counter loop after expiration
 *      DWT_PRESRVE_SLP  0x20 - allows for SLEEP_EN bit to be "preserved", although it will self-clear on wake up
 *      DWT_WAKE_WK      0x10 - wake up on WAKEUP PIN
 *      DWT_WAKE_CS      0x8 - wake up on chip select
 *      DWT_BR_DET       0x4 - enable brownout detector during sleep/deep sleep
 *      DWT_SLEEP        0x2 - enable sleep
 *      DWT_SLP_EN       0x1 - enable sleep/deep sleep functionality
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] mode: Configures on-wake parameters
 * @param[in] wake: Configures wake up parameters
 *
 * @return None
 */
static void ull_configuresleep(dwchip_t *dw, uint16_t mode, uint8_t wake)
{
    uint8_t temp2;
    // set LP OSC trim value to increase freq. to max
    ull_aon_write(dw, (uint16_t)AON_LPOSC_TRIM, 0U);

    // reduce the internal WAKEUP delays to min to minimise wakeup time/reduce current consumption
    /* @note If using slow starting crystals > 1ms. Then this
     * should not be used, as the device will issue SPI_RDY before crystal is stable.
     * Should the host then issue start TX the PLL may not lock and the TX packet will not be sent.
     */
    temp2 = ull_aon_read(dw, (uint16_t)AON_SLPCNT_CAL_CTRL) & 0x1FU; //<<< comment out if slow crystals
    ull_aon_write(dw, (uint16_t)AON_SLPCNT_CAL_CTRL, temp2);        //<<< comment out if slow crystals

    // Add predefined sleep settings before writing the mode
    LOCAL_DATA(dw)->sleep_mode |= mode;

    // Don't run PGF calibration on wakeup
    LOCAL_DATA(dw)->sleep_mode &= ~(uint16_t)DWT_PGFCAL;

    dwt_write16bitoffsetreg(dw, AON_DIG_CFG_ID, 0U, LOCAL_DATA(dw)->sleep_mode);

    dwt_write8bitoffsetreg(dw, ANA_CFG_ID, 0U, wake); // bit 0 - SLEEP_EN, bit 1 - DEEP_SLEEP=0/SLEEP=1, bit 3 wake on CS
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function clears the AON configuration in DW3000
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return  None
 */
static void ull_clearaonconfig(dwchip_t *dw)
{
    // Clear any AON auto download bits (as reset will trigger AON download)
    dwt_write16bitoffsetreg(dw, AON_DIG_CFG_ID, 0U, 0x00U);
    // Clear the wake-up configuration
    dwt_write8bitoffsetreg(dw, ANA_CFG_ID, 0U, 0x00U);
    // Upload the new configuration
    // Copy config to AON - upload the new configuration
    dwt_write8bitoffsetreg(dw, AON_CTRL_ID, 0U, 0U);
    dwt_write8bitoffsetreg(dw, AON_CTRL_ID, 0U, AON_CTRL_ARRAY_SAVE_BIT_MASK);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Sets or clears the auto TX to sleep bit. This means that after a frame transmission
 * the device will enter sleep or deep sleep mode. The dwt_configuresleep() function
 * needs to be called before this to configure the on-wake settings
 *
 * @note The IRQ line has to be low/inactive (i.e. no pending events) otherwise device will not enter sleep until the interrupt has been cleared.
 *       This API has been deprecated, might be removed in a future major release.
 *       Consider using the dwt_entersleepafter() function instead.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] enable: Set to 1 to configure the device to enter sleep or deep sleep after TX, 0 disables the configuration
 *
 * @return None
 */
static void ull_entersleepaftertx(dwchip_t *dw, int32_t enable)
{
    // OTP low power mode
    ull_dis_otp_ips(dw, 1);

    // Set the auto TX -> sleep bit
    if (enable != 0)
    {
        dwt_or16bitoffsetreg(dw, SEQ_CTRL_ID, 0U, SEQ_CTRL_ATX2SLP_BIT_MASK);
    }
    else
    {
        dwt_and16bitoffsetreg(dw, SEQ_CTRL_ID, 0U, (uint16_t)(~SEQ_CTRL_ATX2SLP_BIT_MASK));
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Sets or clears the auto TX and/or RX to sleep bits.
 *
 * This makes the device automatically enter deep sleep or sleep mode after a frame transmission and/or reception.
 * dwt_configuresleep() needs to be called before this to configure the sleep and on-wake/wake-up parameters.
 *
 * @note The IRQ line has to be low/inactive (i.e. no pending events) otherwise device will not enter sleep until the interrupt has been cleared.
 *
 * input parameters
 * @param dw - DW3000 chip descriptor handler.
 * @param event_mask: bitmask to go to sleep after:
 *  - DWT_TX_COMPLETE to configure the device to enter sleep or deep sleep after TX
 *  - DWT_RX_COMPLETE to configure the device to enter sleep or deep sleep after RX
 *
 * output parameters
 *
 * no return value
 */
static void ull_entersleepafter(dwchip_t *dw, int32_t event_mask)
{
    uint16_t seq_ctrl_or = 0U;
    uint16_t seq_ctrl_and = 0xFFFFU;

    // OTP low power mode
    ull_dis_otp_ips(dw, 1);

    if (((uint32_t)event_mask & (uint32_t)DWT_TX_COMPLETE) != 0UL)
    {
        seq_ctrl_or |= SEQ_CTRL_ATX2SLP_BIT_MASK;
    }
    else
    {
        seq_ctrl_and &= ~(uint16_t)SEQ_CTRL_ATX2SLP_BIT_MASK;
    }

    if (((uint32_t)event_mask & (uint32_t)DWT_RX_COMPLETE) != 0UL)
    {
        seq_ctrl_or |= SEQ_CTRL_ARX2SLP_BIT_MASK;
    }
    else
    {
        seq_ctrl_and &= ~(uint16_t)SEQ_CTRL_ARX2SLP_BIT_MASK;
    }

    dwt_modify16bitoffsetreg(dw, SEQ_CTRL_ID, 0U, seq_ctrl_and, seq_ctrl_or);
}

#ifdef WIN32
/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Function to wake up the device from sleep mode using the SPI read,
 * the device will wake up on chip select line going low if the line is held low for at least 500us.
 * To define the length depending on the time one wants to hold
 * the chip select line low, use the following formula:
 *
 *      length (bytes) = time (s) * byte_rate (Hz)
 *
 * where fastest byte_rate is spi_rate (Hz) / 8 if the SPI is sending the bytes back-to-back.
 * To save time and power, a system designer could determine byte_rate value more precisely.
 *
 * @note Alternatively the device can be waken up with WAKE_UP pin if configured for that operation
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] buff: This is a pointer to the dummy buffer which will be used in the SPI read transaction used for the WAKE UP of the device
 * @param[in] length: This is the length of the dummy buffer
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
 */
static int32_t ull_spicswakeup(dwchip_t *dw, uint8_t *buff, uint16_t length)
{
    dwt_error_e retVal = DWT_SUCCESS;

    if ((dwt_error_e)ull_check_dev_id(dw) != DWT_SUCCESS)
    {
        // Need to keep chip select line low for at least 500us
        ull_readfromdevice(dw, 0x0UL, 0x0U, length, buff); // Do a long read to wake up the chip (hold the chip select low)
        // Need 5ms for XTAL to start and stabilize (could wait for PLL lock IRQ status bit !!!)
        // @note Polling of the STATUS register is not possible unless frequency is < 3MHz
        deca_sleep(5U);
    }
    else
    {
        // DEBUG - check if still in sleep mode
        if ((dwt_error_e)ull_check_dev_id(dw) != DWT_SUCCESS)
        {
            retVal = DWT_ERROR;
        }
    }

    return (int32_t)retVal;
}
#endif // WIN32

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This reads the device ID and checks if it is the right one, i.e., if the driver matches the device.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
 */
static int32_t ull_check_dev_id(dwchip_t *dw)
{
    uint32_t dev_id;
    dwt_error_e ret = DWT_ERROR;

    dev_id = dwt_read32bitreg(dw, DEV_ID_ID);

    if ((dw->dwt_driver->devid & dw->dwt_driver->devmatch) == (dev_id & dw->dwt_driver->devmatch))
    {
        ret = DWT_SUCCESS;
    }

    return (int32_t)ret;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function enables CIA diagnostic data. When turned on the following registers will be logged:
 * IP_TOA_LO, IP_TOA_HI, STS_TOA_LO, STS_TOA_HI, STS1_TOA_LO, STS1_TOA_HI, CIA_TDOA_0, CIA_TDOA_1_PDOA, CIA_DIAG_0, CIA_DIAG_1
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] enable_mask: Configures the CIA diagnostic logging in normal/single and double buffer (DB) modes.
 *                @ref DW_CIA_DIAG_LOG_MAX (0x8)   - CIA to copy to swinging set a maximum set of diagnostic registers in DB mode.
 *                @ref DW_CIA_DIAG_LOG_MID (0x4)   - CIA to copy to swinging set a medium set of diagnostic registers in DB mode.
 *                @ref DW_CIA_DIAG_LOG_MIN (0x2)   - CIA to copy to swinging set a minimal set of diagnostic registers in DB mode.
 *                @ref DW_CIA_DIAG_LOG_ALL (0x1)   - CIA to log all diagnostic registers.
 *                @ref DW_CIA_DIAG_LOG_OFF (0x0)   - CIA to log reduced set of diagnostic registers.
 *
 *
 * @return None
 */
static void ull_configciadiag(dwchip_t *dw, uint8_t enable_mask)
{
    if ((enable_mask & (uint8_t)DW_CIA_DIAG_LOG_ALL) != 0U)
    {
        dwt_and8bitoffsetreg(dw, CIA_CONF_ID, 2U, (uint8_t)(~(CIA_DIAGNOSTIC_OFF)));
    }
    else
    {
        dwt_or8bitoffsetreg(dw, CIA_CONF_ID, 2U, CIA_DIAGNOSTIC_OFF);
    }

    LOCAL_DATA(dw)->cia_diagnostic = enable_mask;

    //set at least minimum diagnostic for double buffer mode otherwise data is not logged
    if ((LOCAL_DATA(dw)->cia_diagnostic >> 1U) == 0U)
    {
        dwt_write8bitoffsetreg(dw, RDB_DIAG_MODE_ID, 0U, (uint8_t)DW_CIA_DIAG_LOG_MIN >> 1U);
        LOCAL_DATA(dw)->cia_diagnostic |= (uint8_t)DW_CIA_DIAG_LOG_MIN;
    }
    else
    {
        dwt_write8bitoffsetreg(dw, RDB_DIAG_MODE_ID, 0U, enable_mask >> 1U);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This call signals to the chip that the specific RX buff is free (available) for writing the data of the the next
 *        received frame. It will also update the dblbuffon flag/status to the next buffer.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return  None
 */
static void ull_signal_rx_buff_free(dwchip_t *dw)
{
    dwt_writefastCMD(dw, CMD_DB_TOGGLE);

    // update the status
    if (LOCAL_DATA(dw)->dblbuffon == (uint8_t)DBL_BUFF_ACCESS_BUFFER_1)
    {
        LOCAL_DATA(dw)->dblbuffon = (uint8_t)DBL_BUFF_ACCESS_BUFFER_0; // next buffer is RX_BUFFER_0
    }
    else
    {
        LOCAL_DATA(dw)->dblbuffon = (uint8_t)DBL_BUFF_ACCESS_BUFFER_1; // next buffer is RX_BUFFER_1
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This call enables the double receive buffer mode
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] dbl_buff_state: Enum variable for enabling/disabling double buffering mode (see @ref dwt_dbl_buff_state_e)
 * @param[in] dbl_buff_mode: Enum variable for receiver re-enable (see @ref dwt_dbl_buff_mode_e)
 *
 * @return  None
 */
static void ull_setdblrxbuffmode(dwchip_t *dw, dwt_dbl_buff_state_e dbl_buff_state, dwt_dbl_buff_mode_e dbl_buff_mode)
{
    uint32_t or_val = 0U, and_val = UINT32_MAX;

    if (dbl_buff_state == DBL_BUF_STATE_EN)
    {
        and_val = ~(uint32_t)(SYS_CFG_DIS_DRXB_BIT_MASK);
        LOCAL_DATA(dw)->dblbuffon = (uint8_t)DBL_BUFF_ACCESS_BUFFER_0; // the host will access RX_BUFFER_0 initially (on 1st reception after enable)
        // Updating indirect address here to save time setting it inside the interrupt(in order to read BUF1_RX_FINFO)..
        // Pay attention that after sleep, this register needs to be set again.
        dwt_write32bitreg(dw, INDIRECT_ADDR_B_ID, (BUF1_RX_FINFO >> 16UL));
        dwt_write32bitreg(dw, ADDR_OFFSET_B_ID, BUF1_RX_FINFO & 0xFFFFUL);
    }
    else
    {
        or_val = SYS_CFG_DIS_DRXB_BIT_MASK;
        LOCAL_DATA(dw)->dblbuffon = (uint8_t)DBL_BUFF_OFF;
    }

    if (dbl_buff_mode == DBL_BUF_MODE_AUTO)
    {
        or_val |= SYS_CFG_RXAUTR_BIT_MASK;
    }
    else
    {
        and_val &= (~SYS_CFG_RXAUTR_BIT_MASK); // Clear the needed bit
    }

    dwt_and_or32bitoffsetreg(dw, SYS_CFG_ID, 0U, and_val, or_val);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This sets the receiver turn on delay time after a transmission of a frame
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] rxDelayTime: The delay is in UWB microseconds, 20-bit value.
 *
 * @return None
 */
static void ull_setrxaftertxdelay(dwchip_t *dw, uint32_t rxDelayTime)
{
    uint32_t val = dwt_read32bitreg(dw, ACK_RESP_ID); // Read ACK_RESP_T_ID register

    val &= (~ACK_RESP_W4R_TIM_BIT_MASK); // Clear the timer (19:0)

    val |= (rxDelayTime & ACK_RESP_W4R_TIM_BIT_MASK); // In UWB microseconds (e.g. turn the receiver on 20uus after TX)

    dwt_write32bitoffsetreg(dw, ACK_RESP_ID, 0U, val);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function checks if the IRQ line is active, this can used instead of interrupt handler
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return value is 1 if the SYS_STATUS_IRQS bit is set and 0 otherwise
 */
static uint8_t ull_checkirq(dwchip_t *dw)
{
    /* Reading the lower byte only is enough for this operation */
    return (dwt_read8bitoffsetreg(dw, SYS_STATUS_ID, 0U) & SYS_STATUS_IRQS_BIT_MASK);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function checks if the UWB radio is in IDLE_RC state
 *
 * @note
 * @parblock The DW3XXX states are described in the User Manual. On power up, or following a reset the device will progress from INIT_RC to IDLE_RC.
 * Once the device is in IDLE_RC SPI rate ca be increased to more than 7 MHz. The device will automatically proceed from INIT_RC to IDLE_RC
 * and both INIT_RC and SPI_RDY event flags will be set, once device is in IDLE_RC.
 *
 * It is recommended that host waits for SPI_RDY event, which will also generate interrupt once device is ready after reset/power on.
 * If the host cannot use interrupt as a way to check device is ready for SPI comms, then we recommend the host waits for 2 ms and reads this function,
 * which checks if the device is in IDLE_RC state by reading the SYS_STATUS register and checking for the IDLE_RC event to be set.
 * If host initiates SPI transaction with the device prior to it being ready, the SPI transaction may be incorrectly decoded by the device and
 * the device may be misconfigured. Reading registers over SPI prior to device being ready may return garbage on the MISO, which may confuse the host application.
 * @endparblock
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @return value is 1 if the IDLE_RC bit is set and 0 otherwise
 */
static uint8_t ull_checkidlerc(dwchip_t *dw)
{
    /* Poll DW IC until IDLE_RC event set. This means that DW IC is in IDLE_RC state and ready */
    uint32_t reg = ((uint32_t)dwt_read16bitoffsetreg(dw, SYS_STATUS_ID, 2U) << 16UL);
    return ((reg & (SYS_STATUS_RCINIT_BIT_MASK)) == (SYS_STATUS_RCINIT_BIT_MASK)) ? 1U : 0U;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function clears/resets cbData structure
 *
 * @param[in] cbData: Pointer to dwt_cb_data_t *cbData to clear
 *
 * @return None
 */
static void ull_clear_cbData(dwt_cb_data_t *cbData)
{
    cbData->datalength = 0U;
    cbData->rx_flags = 0U;
    cbData->status = 0UL;
    cbData->status_hi = 0U;
    cbData->dw = NULL;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is the DW3000's general Interrupt Service Routine. It will process/report the following events:
 *          - RXFR + no data mode (through cbRxOk callback, but set datalength to 0)
 *          - RXFCG (through cbRxOk callback)
 *          - TXFRS (through cbTxDone callback)
 *          - RXRFTO/RXPTO (through cbRxTo callback)
 *          - RXPHE/RXFCE/RXRFSL/RXSFDTO/AFFREJ/LDEERR/LCSSERR (through cbRxTo cbRxErr)
 *          -
 *        For all events, corresponding interrupts are cleared and necessary resets are performed. In addition, in the RXFCG case,
 *        received frame information and frame control are read before calling the callback. If double buffering is activated, it
 *        will also toggle between reception buffers once the reception callback processing has ended.
 *
 *        ! This version of the ISR supports double buffering but does not support automatic RX re-enabling!
 *
 * @note  In PC based system using (Cheetah or ARM) USB to SPI converter there can be no interrupts, however we still need something
 *        to take the place of it and operate in a polled way. In an embedded system this function should be configured to be triggered
 *        on any of the interrupts described above.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return  None
 */
static void ull_isr(dwchip_t *dw)
{
    // Read Fast Status register
    uint8_t fstat = dwt_read8bitoffsetreg(dw, FINT_STAT_ID, 0U);
    uint32_t status = dwt_read32bitreg(dw, SYS_STATUS_ID) & ~SYS_STATUS_IRQS_BIT_MASK; // Read status register low 32bits
    dwt_write32bitreg(dw, SYS_STATUS_ID, status); // Clear all status bits
    uint8_t statusDB = 0U;
    bool rx_ok_event;
    bool rxfce_error_event_no_payload;
    bool rx_fr_dis_fce;

    ull_clear_cbData(&LOCAL_DATA(dw)->cbData);
    LOCAL_DATA(dw)->cbData.dw = dw;

    LOCAL_DATA(dw)->cbData.status = status;

    if ((LOCAL_DATA(dw)->stsconfig & (uint8_t)DWT_STS_MODE_ND) == (uint8_t)DWT_STS_MODE_ND) // cannot use FSTAT when in no data mode...
    {
        if ((status & SYS_STATUS_RXFR_BIT_MASK) != 0UL)
        {
            fstat |= FINT_STAT_RXOK_BIT_MASK;
        }
    }

    if ((status & SYS_STATUS_CIADONE_BIT_MASK) != 0UL)
    {
        LOCAL_DATA(dw)->cbData.rx_flags |= (uint8_t)DWT_CB_DATA_RX_FLAG_CIA;
    }

    // Handle System panic confirmation event
    // AES_ERR|SPICRCERR|BRNOUT|SPI_UNF|SPI_OVR|CMD_ERR|SPI_COLLISION|PLLHILO
    if ((fstat & FINT_STAT_SYS_PANIC_BIT_MASK) != 0U)
    {
        LOCAL_DATA(dw)->cbData.status_hi = dwt_read16bitoffsetreg(dw, SYS_STATUS_HI_ID, 0U);
        dwt_write16bitoffsetreg(dw, SYS_STATUS_HI_ID, 0U, LOCAL_DATA(dw)->cbData.status_hi); // Clear all error event bit

        // Handle SPI CRC error event, which was due to an SPI write CRC error
        // Handle SPI error events (if this has happened, the last SPI transaction has not completed correctly, the device should be reset)
        if (((LOCAL_DATA(dw)->spicrc != DWT_SPI_CRC_MODE_NO) && ((LOCAL_DATA(dw)->cbData.status & SYS_STATUS_SPICRCE_BIT_MASK) != 0UL)) ||
            ((LOCAL_DATA(dw)->cbData.status_hi & (SYS_STATUS_HI_SPIERR_BIT_MASK | SYS_STATUS_HI_SPI_UNF_BIT_MASK | SYS_STATUS_HI_SPI_OVF_BIT_MASK)) != 0U))
        {
            // Call the corresponding callback if present
            if (dw->callbacks.cbSPIErr != NULL)
            {
                dw->callbacks.cbSPIErr(&LOCAL_DATA(dw)->cbData);
            }
        }

        // Handle Fast CMD errors event, the means the last CMD did not execute (e.g. it was given while device was already executing previous)
        if ((LOCAL_DATA(dw)->cbData.status_hi & SYS_STATUS_HI_CMD_ERR_BIT_MASK) != 0U)
        {
            // Call the corresponding callback if present
            /*if(LOCAL_DATA(dw)->cbCMDErr != NULL)
             {
             LOCAL_DATA(dw)->cbCMDErr(&LOCAL_DATA(dw)->cbData);
             }*/
        }

        // AES_ERR, BRNOUT, PLLHILO not handled here ...
    }

    // Handle TX frame sent confirmation event
    if ((fstat & FINT_STAT_TXOK_BIT_MASK) != 0U)
    {
        // Resetting to default PLL bias trim value if previously changed
        ull_setpllbiastrim(dw, DWT_DEF_PLLBIASTRIM);

        // Call the corresponding callback if present
        if (dw->callbacks.cbTxDone != NULL)
        {
            dw->callbacks.cbTxDone(&LOCAL_DATA(dw)->cbData);
        }
    }

    // SPI ready and IDLE_RC bit gets set when device powers on, or on wake up
    if ((fstat & FINT_STAT_SYS_EVENT_BIT_MASK) != 0U)
    {
        // Call the corresponding callback if present
        if (dw->callbacks.cbSPIRdy != NULL)
        {
            dw->callbacks.cbSPIRdy(&LOCAL_DATA(dw)->cbData);
        }

        // VTDET, GPIO, not handled here ...
    }

    rx_ok_event = ((fstat & FINT_STAT_RXOK_BIT_MASK) != 0U);
    rxfce_error_event_no_payload = ((status & SYS_STATUS_RXFCE_BIT_MASK) != 0U) && (((uint8_t)dw->isrFlags & (uint8_t)DWT_LEN0_RXGOOD) != 0U);
    rx_fr_dis_fce = (((status & SYS_STATUS_RXFR_BIT_MASK) != 0UL) && (LOCAL_DATA(dw)->sys_cfg_dis_fce_bit_flag == 1U));
    // Handle RX OK event, and RX FCE error event generated because the received frame has no payload - frame length read below,
    // and frame reception when RX FCS check is disabled with DIS_FCE
    if (rx_ok_event || rxfce_error_event_no_payload || rx_fr_dis_fce)
    {
        LOCAL_DATA(dw)->cbData.rx_flags = 0U;

        if(rxfce_error_event_no_payload)
        {
            uint16_t datalength = ull_getframelength(dw, &LOCAL_DATA(dw)->cbData.rx_flags); // read data length
            if(datalength != 0U)
            {
                rxfce_error_event_no_payload = false; //clear the flag
            }
        }

        if ((LOCAL_DATA(dw)->dblbuffon) != 0U) // if in double buffer mode
        {
            statusDB = dwt_read8bitoffsetreg(dw, RDB_STATUS_ID, 0U);

            // If accessing the second buffer (RX_BUFFER_B then read second nibble of the DB status reg)
            if (LOCAL_DATA(dw)->dblbuffon == (uint8_t)DBL_BUFF_ACCESS_BUFFER_1)
            {
                statusDB >>= 4U;
            }
            // setting the relevant bits in the main status register according to DB status register
            if ((statusDB & RDB_STATUS_RXFCG0_BIT_MASK) != 0U)
            {
                status |= SYS_STATUS_RXFCG_BIT_MASK;
            }
            if ((statusDB & RDB_STATUS_RXFR0_BIT_MASK) != 0U)
            {
                status |= SYS_STATUS_RXFR_BIT_MASK;
            }
            if ((statusDB & RDB_STATUS_CIADONE0_BIT_MASK) != 0U)
            {
                status |= SYS_STATUS_CIADONE_BIT_MASK;
            }
        }

        // update the status based on the DB RX events
        LOCAL_DATA(dw)->cbData.status = status;
        // clear LDE error (as we do not want to go back into cbRxErr)
        if ((status & SYS_STATUS_CIAERR_BIT_MASK) != 0UL)
        {
            LOCAL_DATA(dw)->cbData.rx_flags |= (uint8_t)DWT_CB_DATA_RX_FLAG_CER;
        }
        else
        {
            if ((status & SYS_STATUS_CIADONE_BIT_MASK) != 0UL)
            {
                LOCAL_DATA(dw)->cbData.rx_flags |= (uint8_t)DWT_CB_DATA_RX_FLAG_CIA;
            }
        }

        if ((status & SYS_STATUS_CPERR_BIT_MASK) != 0UL)
        {
            LOCAL_DATA(dw)->cbData.rx_flags |= (uint8_t)DWT_CB_DATA_RX_FLAG_CPER;
        }

         // When RXFCE Error is due to frame with no payload OR when using No Data STS mode we do not get RXFCG but RXFR
        if (rxfce_error_event_no_payload || (((status & SYS_STATUS_RXFR_BIT_MASK) != 0UL) &&
            ((LOCAL_DATA(dw)->stsconfig & (uint8_t)DWT_STS_MODE_ND) == (uint8_t)DWT_STS_MODE_ND)))
        {
            LOCAL_DATA(dw)->cbData.rx_flags |= (uint8_t)DWT_CB_DATA_RX_FLAG_ND;
            LOCAL_DATA(dw)->cbData.datalength = 0U;
        }
        else
        {
            // Handle RX good frame event (if FCS check is disabled in the receiver the frame length should be provided to the host
            // to do own CRC check if needed)
            if (((status & SYS_STATUS_RXFCG_BIT_MASK) != 0UL) || rx_fr_dis_fce)
            {
                (void)ull_getframelength(dw, &LOCAL_DATA(dw)->cbData.rx_flags);
            }
        }

        //!!! NOTE: In some circumstances, device can decode frame length as 0, because the PHR will have uncorrectable errors.
        // The PHE uses a 6-bit SECDED parity check (Single Error Correct / Double Error Detect), this can be used to correct 1 bit error or detect 2 bit error,
        // otherwise, e.g. any more than one or two bits in error and it might report an error, or it might not. If PHR has more errors device can
        // incorrectly think that the frame length is 0. This will produce an RXPHD event. But, as when FCS is used the mininum frame length is 3:
        // one byte data and two byte FCS, we cannot have a frame with 0 bytes.
        // Thus here we add a check and signal PHE instead of RX_OK, as this should have been detected as PHE.
        // Do not treat the case of RXFCE event with no payload as an error, as it is a valid case.
        if (!rxfce_error_event_no_payload && LOCAL_DATA(dw)->cbData.datalength == 0U && ((LOCAL_DATA(dw)->stsconfig & (uint8_t)DWT_STS_MODE_ND) != (uint8_t)DWT_STS_MODE_ND))
        {
            LOCAL_DATA(dw)->cbData.status &= ~((uint32_t)DWT_INT_RXFCG_BIT_MASK | (uint32_t)DWT_INT_RXPHD_BIT_MASK); //clear PHD and FCG if set in the callback data structure
            LOCAL_DATA(dw)->cbData.status |= (uint32_t)DWT_INT_RXPHE_BIT_MASK; //add PHE

            // Call the corresponding callback if present
            if (dw->callbacks.cbRxErr != NULL)
            {
                dw->callbacks.cbRxErr(&LOCAL_DATA(dw)->cbData);
            }

            LOCAL_DATA(dw)->cbData.rx_flags = 0U;
        }
        else //RX OK
        {
            // Call the corresponding callback if present
            if (dw->callbacks.cbRxOk != NULL)
            {
                dw->callbacks.cbRxOk(&LOCAL_DATA(dw)->cbData);
            }

        }

        if ((LOCAL_DATA(dw)->dblbuffon) != 0U) // check if in double buffer mode and if so which buffer host is currently accessing
        {
            // Free up the current buffer - let the device know that it can receive into this buffer again
            ull_signal_rx_buff_free(dw);
        }

        LOCAL_DATA(dw)->cbData.rx_flags = 0U;
    }

    // RXFCE&~DISFCE|RXPHE|RXFSL|ARFE|RXSTO|RXOVRR. Real errored frame received, so ignore FCE if disabled
    // Also ignore error if RXFCE error was due to frame with no payload
    // Handle RX errors events
    if (!rxfce_error_event_no_payload && (fstat & FINT_STAT_RXERR_BIT_MASK) != 0U)
    {

        // Call the corresponding callback if present
        if (dw->callbacks.cbRxErr != NULL)
        {
            dw->callbacks.cbRxErr(&LOCAL_DATA(dw)->cbData);
        }

        LOCAL_DATA(dw)->cbData.rx_flags = 0U;
    }

    // Handle RX Timeout event (PTO and FWTO)
    if ((fstat & FINT_STAT_RXTO_BIT_MASK) != 0U)
    {
        // Call the corresponding callback if present
        if (dw->callbacks.cbRxTo != NULL)
        {
            dw->callbacks.cbRxTo(&LOCAL_DATA(dw)->cbData);
        }

        LOCAL_DATA(dw)->cbData.rx_flags = 0U;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function enables the specified events to trigger an interrupt.
 * The following events can be found in SYS_ENABLE_LO and SYS_ENABLE_HI registers.
 *
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] bitmask_lo: Sets the events in SYS_ENABLE_LO_ID register which will generate interrupt
 * @param[in] bitmask_hi: Sets the events in SYS_ENABLE_HI_ID register which will generate interrupt
 * @param[in] INT_options: If set to @ref DWT_ENABLE_INT additional interrupts as selected in the bitmask are enabled
 *                       If set to @ref DWT_ENABLE_INT_ONLY the interrupts in the bitmask are forced to selected state
 *                       i.e., the mask is written to the register directly.
 *                       Otherwise (if set to @ref DWT_DISABLE_INT) clear the interrupts as selected in the bitmask
 *
 * @return  None
 */
static void ull_setinterrupt(dwchip_t *dw, uint32_t bitmask_lo, uint32_t bitmask_hi, dwt_INT_options_e INT_options)
{
    decaIrqStatus_t stat;

    // Need to beware of interrupts occurring in the middle of following read modify write cycle
    stat = decamutexon();

    if (INT_options == DWT_ENABLE_INT_ONLY)
    {
        dwt_write32bitreg(dw, SYS_ENABLE_LO_ID, bitmask_lo); // New value
        dwt_write32bitreg(dw, SYS_ENABLE_HI_ID, bitmask_hi); // New value
    }
    else
    {
        if (INT_options == DWT_ENABLE_INT)
        {
            dwt_or32bitoffsetreg(dw, SYS_ENABLE_LO_ID, 0U, bitmask_lo); // Set the bits
            dwt_or32bitoffsetreg(dw, SYS_ENABLE_HI_ID, 0U, bitmask_hi); // Set the bits
        }
        else
        {
            dwt_and32bitoffsetreg(dw, SYS_ENABLE_LO_ID, 0U, (uint32_t)(~bitmask_lo)); // Clear the bits
            dwt_and32bitoffsetreg(dw, SYS_ENABLE_HI_ID, 0U, (uint32_t)(~bitmask_hi)); // Clear the bits
        }
    }

    /*Clearing any existing events which can give rise to interrupts*/
    dwt_write32bitreg(dw, SYS_STATUS_ID, dwt_read32bitreg(dw, SYS_ENABLE_LO_ID));
    dwt_write32bitreg(dw, SYS_STATUS_HI_ID, dwt_read32bitreg(dw, SYS_ENABLE_HI_ID));

    decamutexoff(stat);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to set up TX/RX GPIOs which could be used to control LEDs
 * @note Not completely IC dependent, also needs a board with LEDS fitted on right I/O lines.
 *       This function enables GPIOs 2 and 3 which are connected to D1 and D2 on the DW3000 CSP shield / LED2 and LED3
 *       on the DWM3000EVB and QM33120MEVB, which are connected to LED3 and LED4 on EVB1000
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] mode: This is a bit field interpreted as follows:
 *          - bit 0: 1 to enable LEDs, 0 to disable them
 *          - bit 1: 1 to make LEDs blink once on init. Only valid if bit 0 is set (enable LEDs)
 *          - bit 2 to 7: reserved
 *
 * @return None
 */
static void ull_setleds(dwchip_t *dw, uint8_t mode)
{
    uint32_t reg;
    if ((mode & (uint8_t)DWT_LEDS_ENABLE) != 0U)
    {
        // Set up MFIO for LED output.
        dwt_modify32bitoffsetreg(dw, GPIO_MODE_ID, 0U, ~(uint32_t)(GPIO_MODE_MSGP3_MODE_BIT_MASK | GPIO_MODE_MSGP2_MODE_BIT_MASK), ((uint32_t)GPIO_PIN2_RXLED | (uint32_t)GPIO_PIN3_TXLED));

        // Enable LP Oscillator to run from counter and turn on de-bounce clock.
        dwt_or32bitoffsetreg(dw, CLK_CTRL_ID, 0U, (CLK_CTRL_GPIO_DCLK_EN_BIT_MASK | CLK_CTRL_LP_CLK_EN_BIT_MASK));

        // Enable LEDs to blink and set default blink time.
        reg = LED_CTRL_BLINK_EN_BIT_MASK | (uint32_t)DWT_LEDS_BLINK_TIME_DEF;
        // Make LEDs blink once if requested.
        if ((mode & (uint8_t)DWT_LEDS_INIT_BLINK) != 0U)
        {
            reg |= LED_CTRL_FORCE_TRIGGER_BIT_MASK;
        }
        dwt_write32bitreg(dw, LED_CTRL_ID, reg);
        // Clear force blink bits if needed.
        if ((mode & (uint8_t)DWT_LEDS_INIT_BLINK) != 0U)
        {
            reg &= (~LED_CTRL_FORCE_TRIGGER_BIT_MASK);
            dwt_write32bitreg(dw, LED_CTRL_ID, reg);
        }
    }
    else
    {
        // Clear the GPIO bits that are used for LED control.
        dwt_and32bitoffsetreg(dw, GPIO_MODE_ID, 0U, ~(uint32_t)(GPIO_MODE_MSGP2_MODE_BIT_MASK | GPIO_MODE_MSGP3_MODE_BIT_MASK));
        dwt_and16bitoffsetreg(dw, LED_CTRL_ID, 0U, (uint16_t)~LED_CTRL_BLINK_EN_BIT_MASK);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Function to enable/disable clocks to particular digital blocks/system
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] clocks: Set of clocks to enable/disable
 *
 * @return  None
 */
static void ull_force_clocks(dwchip_t *dw, int32_t clocks)
{
    if (clocks == FORCE_CLK_SYS_TX)
    {
        uint16_t regvalue0 = CLK_CTRL_TX_BUF_CLK_ON_BIT_MASK | CLK_CTRL_RX_BUF_CLK_ON_BIT_MASK;

        // SYS_CLK_SEL = PLL
        regvalue0 |= (FORCE_SYSCLK_PLL) << CLK_CTRL_SYS_CLK_SEL_BIT_OFFSET;

        // TX_CLK_SEL = ON
        regvalue0 |= (FORCE_CLK_PLL) << CLK_CTRL_TX_CLK_SEL_BIT_OFFSET;

        dwt_write16bitoffsetreg(dw, CLK_CTRL_ID, 0x0U, regvalue0);
    }

    if (clocks == FORCE_CLK_AUTO)
    {
        // Restore auto clock mode
        // we only need to restore the low 16 bits as they are the only ones to change as a result of  FORCE_CLK_SYS_TX
        dwt_write16bitoffsetreg(dw, CLK_CTRL_ID, 0x0U, (uint16_t)DWT_AUTO_CLKS);
    }

} // end ull_force_clocks()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API function configures the reference time used for relative timing of delayed sending and reception.
 * The value is at a 8ns resolution.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param reftime: The reference time (which together with DX_TIME or TX timestamp or RX timestamp time is used to define a
 * transmission time or delayed RX on time)
 *
 * @return  None
 */
static void ull_setreferencetrxtime(dwchip_t *dw, uint32_t reftime)
{
    dwt_write32bitoffsetreg(dw, DREF_TIME_ID, 0U, reftime); // Note: bit 0 of this register is ignored
} // end ull_setreferencetrxtime()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API function configures the delayed transmit time or the delayed RX on time
 * The value is at a 8ns resolution.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param starttime: The TX/RX start time (the 32-bits should be the high 32-bits of the system time at which to send the message,
 * or at which to turn on the receiver)
 *
 * @return  None
 */
static void ull_setdelayedtrxtime(dwchip_t *dw, uint32_t starttime)
{
    dwt_write32bitoffsetreg(dw, DX_TIME_ID, 0U, starttime); // Note: bit 0 of this register is ignored
} // end ull_setdelayedtrxtime()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function corrects for the addition of TX antenna delay when performing delayed TX/RX w.r.t. TX/RX timestamp.
 *
 */
static void dwt_adjust_delaytime(dwchip_t *dw, int32_t tx_rx)
{
    if (tx_rx != 0)
    {
        /** because TX delay is specified from previous TX timestamp,
         *  the relative delay is from a RAW TX time, without antenna delay adjustment
         *  thus we need to subtract the TX antenna delay here as TX timestamp has this automatically added after TX event.
         */
        uint32_t tx_delay = dwt_read32bitoffsetreg(dw, DX_TIME_ID, 0U);
        tx_delay -= dwt_read8bitoffsetreg(dw, TX_ANTD_ID, 1U);
        dwt_write32bitoffsetreg(dw, DX_TIME_ID, 0U, tx_delay);
    }
    else
    {
        /** because TX delay is specified from previous RX timestamp,
         *  the relative delay is from a RAW RX time, without antenna delay adjustment
         *  thus we need to subtract the RX antenna delay here as RX timestamp has this automatically added after RX event.
         */
        uint32_t tx_delay = dwt_read32bitoffsetreg(dw, DX_TIME_ID, 0U);
        tx_delay -= dwt_read8bitoffsetreg(dw, CIA_CONF_ID, 1U);
        dwt_write32bitoffsetreg(dw, DX_TIME_ID, 0U, tx_delay);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This call initiates the transmission, input parameter indicates which TX mode is used see below
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] mode: It is a bit mask made up of @ref dwt_starttx_mode_e definitions.
 *               If mode = DWT_START_TX_IMMEDIATE - immediate TX (no response expected)
 *               if mode = DWT_START_TX_DELAYED   - delayed TX (no response expected) at specified time (time in DX_TIME register)
 *               if mode = DWT_START_TX_DLY_REF   - delayed TX (no response expected) at specified time
 *                                                  (time in DREF_TIME register + any time in DX_TIME register)
 *               if mode = DWT_START_TX_DLY_RS    - delayed TX (no response expected) at specified time
 *                                                  (time in RX_TIME_0 register + any time in DX_TIME register)
 *               if mode = DWT_START_TX_DLY_TS    - delayed TX (no response expected) at specified time
 *                                                  (time in TX_TIME_LO register + any time in DX_TIME register)
 *               if mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED - immediate TX (response expected,
 *                                                                          so the receiver will be automatically
 *                                                                          turned on after TX is done)
 *               if mode = DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED - delayed TX (response expected,
 *                                                                        so the receiver will be automatically
 *                                                                        turned on after TX is done)
 *               if mode = DWT_START_TX_CCA        - Send the frame if no preamble detected within PTO time
 *               if mode = DWT_START_TX_CCA  | DWT_RESPONSE_EXPECTED - Send the frame if no preamble detected
 *                                                                     within PTO time and then enable RX output parameters
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error (e.g., a delayed transmission will be cancelled if the delayed time has passed)
 */
static int32_t ull_starttx(dwchip_t *dw, uint8_t mode)
{
    dwt_error_e retval = DWT_SUCCESS;

    if (((mode & (uint8_t)DWT_START_TX_DELAYED) | (mode & (uint8_t)DWT_START_TX_DLY_REF) |
         (mode & (uint8_t)DWT_START_TX_DLY_RS) | (mode & (uint8_t)DWT_START_TX_DLY_TS)) != 0U)
    {
        if ((mode & (uint8_t)DWT_START_TX_DELAYED) != 0U) // delayed TX
        {
            if ((mode & (uint8_t)DWT_RESPONSE_EXPECTED) != 0U)
            {
                dwt_writefastCMD(dw, CMD_DTX_W4R);
            }
            else
            {
                dwt_writefastCMD(dw, CMD_DTX);
            }
        }
        else if ((mode & (uint8_t)DWT_START_TX_DLY_RS) != 0U) // delayed TX WRT RX timestamp
        {
            dwt_adjust_delaytime(dw, 0);

            if ((mode & (uint8_t)DWT_RESPONSE_EXPECTED) != 0U)
            {
                dwt_writefastCMD(dw, CMD_DTX_RS_W4R);
            }
            else
            {
                dwt_writefastCMD(dw, CMD_DTX_RS);
            }
        }
        else if ((mode & (uint8_t)DWT_START_TX_DLY_TS) != 0U) // delayed TX WRT TX timestamp
        {
            dwt_adjust_delaytime(dw, 1);

            if ((mode & (uint8_t)DWT_RESPONSE_EXPECTED) != 0U)
            {
                dwt_writefastCMD(dw, CMD_DTX_TS_W4R);
            }
            else
            {
                dwt_writefastCMD(dw, CMD_DTX_TS);
            }
        }
        else // delayed TX WRT reference time
        {
            if ((mode & (uint8_t)DWT_RESPONSE_EXPECTED) != 0U)
            {
                dwt_writefastCMD(dw, CMD_DTX_REF_W4R);
            }
            else
            {
                dwt_writefastCMD(dw, CMD_DTX_REF);
            }
        }

        uint8_t checkTxOK = dwt_read8bitoffsetreg(dw, SYS_STATUS_ID, 3U);        // Read at offset 3 to get the upper 2 bytes out of 5
        if ((checkTxOK & (uint8_t)(SYS_STATUS_HPDWARN_BIT_MASK >> 24UL)) == 0U) // Transmit Delayed Send set over Half a Period away.
        {
            uint32_t sys_state = dwt_read32bitreg(dw, SYS_STATE_LO_ID);
            if (sys_state == DW_SYS_STATE_TXERR)
            {
                dwt_writefastCMD(dw, CMD_TXRXOFF);
                retval = DWT_ERROR; // Failed !
            }
            else
            {
                retval = DWT_SUCCESS; // All okay
            }
        }
        else
        {
            dwt_writefastCMD(dw, CMD_TXRXOFF);
            retval = DWT_ERROR; // Failed !

            // optionally could return error, and still send the frame at indicated time
            // then if the application want to cancel the sending this can be done in a separate command.
        }
    }
    else if ((mode & (uint8_t)DWT_START_TX_CCA) != 0U)
    {
        if ((mode & (uint8_t)DWT_RESPONSE_EXPECTED) != 0U)
        {
            dwt_writefastCMD(dw, CMD_CCA_TX_W4R);
        }
        else
        {
            dwt_writefastCMD(dw, CMD_CCA_TX);
        }
    }
    else
    {
        if ((mode & (uint8_t)DWT_RESPONSE_EXPECTED) != 0U)
        {
            dwt_writefastCMD(dw, CMD_TX_W4R);
        }
        else
        {
            dwt_writefastCMD(dw, CMD_TX);
        }
    }

    return (int32_t)retval;

} // end ull_starttx()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to turn off the transceiver
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return  None
 */
static void ull_forcetrxoff(dwchip_t *dw)
{
    // check if in TX or RX state before forcing device into IDLE state
    if (!(dwt_read8bitoffsetreg(dw, SYS_STATE_LO_ID, 2U) <= DW_SYS_STATE_IDLE))
    {
        decaIrqStatus_t stat;

        // Need to beware of interrupts occurring in the middle of following command cycle
        // We can disable the radio, but before the status is cleared an interrupt can be set (e.g. the
        // event has just happened before the radio was disabled)
        // thus we need to disable interrupt during this operation
        stat = decamutexon();

        dwt_writefastCMD(dw, CMD_TXRXOFF);

        // Enable/restore interrupts again...
        decamutexoff(stat);
    }
    // else device is already in IDLE or IDLE_RC ... must not force to IDLE if in IDLE_RC
} // end ull_forcetrxoff()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This enables/disables and configure SNIFF mode.
 *
 * SNIFF mode is a low-power reception mode where the receiver is sequenced on and off instead of being on all the time.
 * The time spent in each state (on/off) is specified through the parameters below.
 * See DW3000 User Manual section 4.5 "Low-Power SNIFF mode" for more details.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] enable: Set to 1 to enable SNIFF mode, 0 to disable. When 0, all other parameters are not taken into account.
 * @param[in] timeOn: Duration of receiver ON phase, expressed in multiples of PAC size. The counter automatically adds 1 PAC
 *                 size to the value set. Min value that can be set is 1 (i.e. an ON time of 2 PAC size), max value is 15.
 * @param[in] timeOff: Duration of receiver OFF phase, expressed in multiples of 128/125 us (~1 us). Max value is 255.
 *
 * output parameters
 *
 * no return value
 */
static void ull_setsniffmode(dwchip_t *dw, int32_t enable, uint8_t timeOn, uint8_t timeOff)
{
    if (enable != 0)
    {
        /* Configure ON/OFF times and enable PLL2 on/off sequencing by SNIFF mode. */
        uint16_t sniff_reg = (((uint16_t)timeOff << 8U) | timeOn) & (RX_SNIFF_SNIFF_OFF_BIT_MASK | RX_SNIFF_SNIFF_ON_BIT_MASK);
        dwt_write16bitoffsetreg(dw, RX_SNIFF_ID, 0U, sniff_reg);
    }
    else
    {
        /* Clear ON/OFF times and disable PLL2 on/off sequencing by SNIFF mode. */
        dwt_write16bitoffsetreg(dw, RX_SNIFF_ID, 0U, 0x0000U);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This call turns on the receiver, can be immediate or delayed (depending on the mode parameter). In the case of a
 * "late" error the receiver will only be turned on immediately unless the @ref DWT_IDLE_ON_DLY_ERR is not set.
 * The receiver will stay turned on, listening to any messages until
 * it either receives a good frame, an error (CRC, PHY header, Reed Solomon) or  it times out (SFD, Preamble or Frame).
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] mode: This can be one of the following values (@ref dwt_startrx_mode_e):
 *
 * DWT_START_RX_IMMEDIATE      0x00    Enable the receiver immediately
 * DWT_START_RX_DELAYED        0x01    Set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
 * DWT_IDLE_ON_DLY_ERR         0x02    If delayed RX failed due to "late" error then if this
 *                                     flag is set the RX will not be re-enabled immediately, and device will be in IDLE when function exits
 * DWT_START_RX_DLY_REF        0x04    Enable the receiver at specified time (time in DREF_TIME register + any time in DX_TIME register)
 * DWT_START_RX_DLY_RS         0x08    Enable the receiver at specified time (time in RX_TIME_0 register + any time in DX_TIME register)
 * DWT_START_RX_DLY_TS         0x10    Enable the receiver at specified time (time in TX_TIME_LO register + any time in DX_TIME register)
 *
 * e.g.,
 * (DWT_START_RX_DELAYED | DWT_IDLE_ON_DLY_ERR) 0x03 used to disable re-enabling of receiver if delayed RX failed due to "late" error
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error (e.g., a delayed receive enable will be too far in the future, the delayed time has passed)
 */
static int32_t ull_rxenable(dwchip_t *dw, int32_t mode)
{
    uint8_t temp1;
    dwt_error_e retval = DWT_SUCCESS;
    
    // Bias_trim always set to DWT_DEF_PLLBIASTRIM when RX enabled
    ull_setpllbiastrim(dw, DWT_DEF_PLLBIASTRIM);

    if (mode == (int32_t)DWT_START_RX_IMMEDIATE)
    {
        dwt_writefastCMD(dw, CMD_RX);
    }
    else // delayed RX
    {
        switch ((uint32_t)mode & ~(uint32_t)DWT_IDLE_ON_DLY_ERR)
        {
        case (uint32_t)DWT_START_RX_DELAYED:
            dwt_writefastCMD(dw, CMD_DRX);
            break;
        case (uint32_t)DWT_START_RX_DLY_REF:
            dwt_writefastCMD(dw, CMD_DRX_REF);
            break;
        case (uint32_t)DWT_START_RX_DLY_RS:
        {
            dwt_adjust_delaytime(dw, 0);

            dwt_writefastCMD(dw, CMD_DRX_RS);
        }
        break;
        case (uint32_t)DWT_START_RX_DLY_TS:
        {
            dwt_adjust_delaytime(dw, 1);

            dwt_writefastCMD(dw, CMD_DRX_TS);
        }
        break;
        default:
        {
            retval = DWT_ERROR; // return error
            break;
        }
        }

        if (retval != DWT_ERROR) {
            temp1 = dwt_read8bitoffsetreg(dw, SYS_STATUS_ID, 3U);    // Read 1 byte at offset 3 to get the 4th byte out of 5
            if ((temp1 & (SYS_STATUS_HPDWARN_BIT_MASK >> 24UL)) != 0U) // if delay has passed do immediate RX on unless DWT_IDLE_ON_DLY_ERR is true
            {
                dwt_writefastCMD(dw, CMD_TXRXOFF);

                if (((uint32_t)mode & (uint32_t)DWT_IDLE_ON_DLY_ERR) == 0UL) // if DWT_IDLE_ON_DLY_ERR not set then re-enable receiver
                {
                    dwt_writefastCMD(dw, CMD_RX);
                }
                retval = DWT_ERROR; // return warning indication
            }
        }
    }

    return (int32_t)retval;
} // end dwt_rxenable()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This call enables RX timeout (SYS_STATUS_RFTO event)
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] time: How long the receiver remains on from the RX enable command
 *               The time parameter used here is in 1.0256 us (512/499.2MHz) units
 *               If set to 0 the timeout is disabled.
 *
 * @return  None
 */
static void ull_setrxtimeout(dwchip_t *dw, uint32_t on_time)
{
    if (on_time > 0UL)
    {
        dwt_write32bitoffsetreg(dw, RX_FWTO_ID, 0U, on_time);

        dwt_or16bitoffsetreg(dw, SYS_CFG_ID, 0U, SYS_CFG_RXWTOE_BIT_MASK); // set the RX FWTO bit
    }
    else
    {
        dwt_and16bitoffsetreg(dw, SYS_CFG_ID, 0U, (uint16_t)(~SYS_CFG_RXWTOE_BIT_MASK)); // clear the RX FWTO bit
    }
} // end ull_setrxtimeout()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This call enables preamble timeout (SYS_STATUS_RXPTO event)
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param  timeout - Preamble detection timeout, expressed in multiples of PAC size. The counter automatically adds 1 PAC
 *                   size to the value set.
 *                   Value 0 will disable the preamble timeout.
 *                   Value X >= 1 sets a timeout equal to (X + 1) * PAC
 *
 * @return  None
 */
static void ull_setpreambledetecttimeout(dwchip_t *dw, uint16_t timeout)
{
    dwt_write16bitoffsetreg(dw, DTUNE1_ID, 0U, timeout);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to enable/disable the event counter in the IC
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] enable: 1 enables (and reset), 0 disables the event counters
 *
 * @return  None
 */
static void ull_configeventcounters(dwchip_t *dw, int32_t enable)
{
    // Need to clear and disable, can't just clear
    dwt_write8bitoffsetreg(dw, EVC_CTRL_ID, 0x0U, EVC_CTRL_EVC_CLR_BIT_MASK);

    if (enable != 0)
    {
        dwt_write8bitoffsetreg(dw, EVC_CTRL_ID, 0x0U, EVC_CTRL_EVC_EN_BIT_MASK); // Enable
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the event counters in the IC
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] counters: Pointer to the @ref dwt_deviceentcnts_t structure which will hold the read data
 *
 * @return  None
 */
static void ull_readeventcounters(dwchip_t *dw, dwt_deviceentcnts_t *counters)
{
    uint32_t temp;

    temp = dwt_read32bitoffsetreg(dw, EVC_COUNT0_ID, 0U); // Read sync loss (27-16), PHE (11-0)
    counters->PHE = (uint16_t)(temp & 0xFFFUL);
    counters->RSL = (uint16_t)((temp >> 16UL) & 0xFFFU);

    temp = dwt_read32bitoffsetreg(dw, EVC_COUNT1_ID, 0U); // Read CRC bad (27-16), CRC good (11-0)
    counters->CRCG = (uint16_t)(temp & 0xFFFUL);
    counters->CRCB = (uint16_t)((temp >> 16UL) & 0xFFFU);

    temp = dwt_read32bitoffsetreg(dw, EVC_COUNT2_ID, 0U); // Overruns (23-16), address errors (7-0)
    counters->ARFE = (uint8_t)temp;
    counters->OVER = (uint8_t)(temp >> 16UL);

    temp = dwt_read32bitoffsetreg(dw, EVC_COUNT3_ID, 0U); // Read PTO (27-16), SFDTO (11-0)
    counters->PTO = (uint16_t)((temp >> 16UL) & 0xFFFU);
    counters->SFDTO = (uint16_t)(temp & 0xFFFUL);

    temp = dwt_read32bitoffsetreg(dw, EVC_COUNT4_ID, 0U); // Read TXFRAME (27-16), RX TO (7-0)
    counters->TXF = (uint16_t)((temp >> 16UL) & 0xFFFU);
    counters->RTO = (uint8_t)temp;

    temp = dwt_read32bitoffsetreg(dw, EVC_COUNT5_ID, 0U); // Read half period warning (7-0) events
    counters->HPW = (uint8_t)temp;
    counters->CRCE = (uint8_t)(temp >> 16UL); // SPI CRC errors (23-16) warning events

    temp = dwt_read32bitoffsetreg(dw, EVC_COUNT6_ID, 0U); // Preamble rejections (11-0) events
    counters->PREJ = (uint16_t)(temp & 0xFFFUL);

    counters->SFDD = 0U; // SFD detections (27-16) events ... ONLY valid in DW3720

    counters->STSE = dwt_read8bitoffsetreg(dw, EVC_COUNT7_ID, 0U); // STS error (7-0) events
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function resets the DW3000
 *
 * @note SPI rate must be <= 7MHz before a call to this function as the device will use FOSC/4 as part of internal reset
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return  None
 */
static void ull_softreset(dwchip_t *dw)
{
    // clear any AON configurations (this will leave the device at FOSC/4, thus we need low SPI rate)
    ull_clearaonconfig(dw);

    // make sure the new AON array config has been set
    deca_sleep(1U);

    // need to make sure clock is not PLL as the PLL will be switched off as part of reset
    dwt_or8bitoffsetreg(dw, CLK_CTRL_ID, 0U, FORCE_SYSCLK_FOSC);

    // Reset HIF, TX, RX and PMSC
    dwt_write8bitoffsetreg(dw, SOFT_RST_ID, 0U, (uint8_t)DWT_RESET_ALL);

    // DW3000 needs 1.5ms to initialise after reset
    deca_sleep(1U);

    dwt_localstruct_init(LOCAL_DATA(dw));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to adjust the crystal frequency
 *
 * @note dwt_initialise() must be called prior to this function in order to initialise the local data
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] value: Crystal trim value (in range 0x0 to 0x3F) 64 steps  (~1.65ppm per step)
 *
 * @return  None
 */
static void ull_setxtaltrim(dwchip_t *dw, uint8_t value)
{
    value &= XTAL_TRIM_BIT_MASK;
    LOCAL_DATA(dw)->init_xtrim = value;
    dwt_write8bitoffsetreg(dw, XTAL_ID, 0U, value);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function returns the value of XTAL trim that has been applied during initialisation (dwt_initialise()). This can
 *        be either the value read in OTP memory or a default value.
 *
 * @note The value returned by this function is the initial value only! It is not updated on dwt_setxtaltrim() calls.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return The XTAL trim value set upon initialisation
 */
static uint8_t ull_getxtaltrim(dwchip_t *dw)
{
    return LOCAL_DATA(dw)->init_xtrim;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will disable TX LDOs and allow TX blocks to be manually turned off by ull_disable_rftx_blocks()
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] switch_config: Specifies whether the switch needs to be restored to automatic control
 *
 * @return None
 */
static void ull_disable_rf_tx(dwchip_t *dw, uint8_t switch_config)
{
    // Turn off TX LDOs
    dwt_write32bitoffsetreg(dw, LDO_CTRL_ID, 0U, 0x00000000UL);

    // Disable RF blocks for TX (configure RF_ENABLE_ID reg)
    dwt_write32bitoffsetreg(dw, RF_ENABLE_ID, 0U, 0x00000000UL);

    // Enable RF sequencing
    dwt_or8bitoffsetreg(dw, SEQ_CTRL_ID, 1U, (uint8_t)((SEQ_CTRL_AUTO_RX_SEQ_BIT_MASK | SEQ_CTRL_AUTO_TX_SEQ_BIT_MASK) >> 8UL));

    if (switch_config != 0U)
    {
        // Restore the TXRX switch to auto
        dwt_modify32bitoffsetreg(dw, RF_SWITCH_CTRL_ID, 0x0U, 0xF0FFUL, TXRXSWITCH_AUTO);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will enable TX LDOs and allow TX blocks to be manually turned on by
 *        ull_enable_rftx_blocks for a given channel.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] switch_config: Specifies whether the switch needs to be configured for TX
 * @param[in] framerepetitionrate: If 0 the TX LDOs are enabled, and RF sequencing is disabled.
 *
 * @return None
 */
static void ull_enable_rf_tx(dwchip_t *dw, uint8_t switch_control, uint32_t framerepetitionrate)
{
    if (framerepetitionrate == 0UL)
    {
        // Turn on TX LDOs
        dwt_or32bitoffsetreg(dw, LDO_CTRL_ID, 0U, (LDO_CTRL_LDO_VDDHVTX_VREF_BIT_MASK | LDO_CTRL_LDO_VDDHVTX_EN_BIT_MASK));
        dwt_or32bitoffsetreg(dw, LDO_CTRL_ID, 0U,
            (LDO_CTRL_LDO_VDDTX2_VREF_BIT_MASK | LDO_CTRL_LDO_VDDTX1_VREF_BIT_MASK | LDO_CTRL_LDO_VDDTX2_EN_BIT_MASK | LDO_CTRL_LDO_VDDTX1_EN_BIT_MASK));
        // Disable RF sequencing
        // This will remove the ~8us LDO power-up gap before each frame
        dwt_and8bitoffsetreg(dw, SEQ_CTRL_ID, 1U, (uint8_t)((~(SEQ_CTRL_AUTO_RX_SEQ_BIT_MASK | SEQ_CTRL_AUTO_TX_SEQ_BIT_MASK)) >> 8UL));
    }
    // Enable RF blocks for TX (configure RF_ENABLE_ID reg)
    dwt_or32bitoffsetreg(dw, RF_ENABLE_ID, 0U,
        (RF_ENABLE_TX_SW_EN_BIT_MASK | RF_ENABLE_TX_CH5_BIT_MASK | RF_ENABLE_TX_EN_BIT_MASK | RF_ENABLE_TX_EN_BUF_BIT_MASK | RF_ENABLE_TX_BIAS_EN_BIT_MASK));

    if (switch_control != 0U)
    {
        uint32_t switch_rf_port = dwt_read32bitoffsetreg(dw, RF_SWITCH_CTRL_ID, 0x0U) & 0xF0FFUL; //check which RF port is selected

        if(switch_rf_port == 0UL)
        {
            //if non selected, i.e., auto mode, then select RF port 1
            switch_rf_port |= 0x1000UL;
        }

        // configure the TXRX switch for TX mode, leave RF port as selected by ull_configure_rf_port()
        dwt_modify32bitoffsetreg(dw, RF_SWITCH_CTRL_ID, 0x0U, 0x00FFUL, TXRXSWITCH_TX | switch_rf_port);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will enable a repeated continuous waveform on the device
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] cw_enable: CW mode enable
 * @param[in] cw_mode_config: CW configuration mode.
 *
 * @return  None
 *
 */
static void ull_repeated_cw(dwchip_t *dw, int32_t cw_enable, int32_t cw_mode_config)
{
    // Turn off TX Seq
    ull_setfinegraintxseq(dw, 0);

    if (cw_mode_config > 0xF)
    {
        cw_mode_config = 0xF;
    }
    if ((cw_enable > 3) || (cw_enable < 1))
    {
        cw_enable = 4;
    }

    dwt_write32bitoffsetreg(dw, TX_TEST_ID, 0x0U, 0x10UL >> (uint32_t)cw_enable);
    dwt_write32bitoffsetreg(dw, PG_TEST_ID, 0x0U, (uint32_t)cw_mode_config << (((uint32_t)cw_enable - 1UL) * 4UL));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function disables repeated frames from being generated.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return  None
 */
static void ull_stop_repeated_frames(dwchip_t *dw)
{
    // Disable repeated frames
    dwt_and8bitoffsetreg(dw, TEST_CTRL0_ID, 0x0U, (uint8_t)(~TEST_CTRL0_TX_PSTM_BIT_MASK));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function enables repeated frames to be generated given a frame repetition rate.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] framerepetitionrate: Value specifying the rate at which frames will be repeated.
 *                            If the value is less than the frame duration, the frames are sent
 *                            back-to-back.
 *
 * @return  None
 */
static void ull_repeated_frames(dwchip_t *dw, uint32_t framerepetitionrate)
{
    // Enable repeated frames
    dwt_or8bitoffsetreg(dw, TEST_CTRL0_ID, 0x0U, TEST_CTRL0_TX_PSTM_BIT_MASK);

    if (framerepetitionrate < 2UL)
    {
        framerepetitionrate = 2UL;
    }
    dwt_write32bitreg(dw, DX_TIME_ID, framerepetitionrate);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function enables preamble test mode which sends a preamble pattern for duration specified by delay parameter.
 *        Once the delay expires, the device goes back to normal mode.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] delay: This is the duration of the preamble transmission in us
 * @param[in] test_txpower: This is the TX power to be applied while transmitting preamble
 *
 * @return  None
 */
static void ull_send_test_preamble(dwchip_t *dw, uint16_t delay, uint32_t test_txpower)
{
    //save current TX power so we can restore this
    uint32_t txpow = dwt_read32bitoffsetreg(dw, TX_POWER_ID, 0U);

    ull_enable_rf_tx(dw, 1, 0);
    ull_enable_rftx_blocks(dw);
    ull_force_clocks(dw, FORCE_CLK_SYS_TX);

    if(test_txpower != 0UL) {
        dwt_write32bitoffsetreg(dw, TX_POWER_ID, 0U, test_txpower);
    }

    // Test pulse transmission
    dwt_write32bitoffsetreg(dw, TX_TEST_ID, 0U, 0x0F00000FUL); //enable all PGs
    dwt_write32bitoffsetreg(dw, PG_TST_DATA_ID, 0U, 0xDDDDDDDDUL); //pattern

    deca_usleep(delay);

    dwt_write32bitoffsetreg(dw, TX_TEST_ID, 0U, 0UL);
    dwt_write32bitoffsetreg(dw, PG_TST_DATA_ID, 0U, 0UL);

    ull_force_clocks(dw, FORCE_CLK_AUTO);
    ull_disable_rftx_blocks(dw);
    ull_disable_rf_tx(dw, 1);

    // restore TX power
    dwt_write32bitoffsetreg(dw, TX_POWER_ID, 0U, txpow);

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function disables the automatic sequencing of the TX RF blocks.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return None
 */
static void ull_enable_rftx_blocks(dwchip_t *dw)
{
    dwt_or32bitoffsetreg(dw, RF_CTRL_MASK_ID, 0U,
        (RF_ENABLE_TX_SW_EN_BIT_MASK | RF_ENABLE_TX_CH5_BIT_MASK | RF_ENABLE_TX_EN_BIT_MASK | RF_ENABLE_TX_EN_BUF_BIT_MASK | RF_ENABLE_TX_BIAS_EN_BIT_MASK));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function enables the automatic sequencing of the TX RF blocks.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return  None
 */
static void ull_disable_rftx_blocks(dwchip_t *dw)
{
    dwt_write32bitoffsetreg(dw, RF_CTRL_MASK_ID, 0U, 0x00000000UL);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function sets the UWB radio to transmit continuous wave (CW) signal at specific channel frequency
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return  None
 */
static void ull_configcwmode(dwchip_t *dw)
{
    ull_enable_rf_tx(dw, 1, 0);
    ull_enable_rftx_blocks(dw);
    ull_force_clocks(dw, FORCE_CLK_SYS_TX);
    ull_repeated_cw(dw, 1, 0xF); // PulseGen Channel 1, full power
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function sets the UWB radio to continuous TX frame mode for regulatory approvals testing.
 *
 * @note dwt_configure() and dwt_configuretxrf() must be called before a call to this API
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] framerepetitionrate: This is a 32-bit value that is used to set the interval between transmissions.
 *  The minimum value is 2. The units are approximately 4 ns. (or more precisely 512/(499.2e6*256) seconds)).
 *
 * @return  None
 */
static void ull_configcontinuousframemode(dwchip_t *dw, uint32_t framerepetitionrate)
{
    // NOTE: dwt_configure and dwt_configuretxrf must be called before a call to this API
    ull_enable_rf_tx(dw, 1, framerepetitionrate);
    ull_enable_rftx_blocks(dw);
    ull_force_clocks(dw, FORCE_CLK_SYS_TX);
    ull_repeated_frames(dw, framerepetitionrate);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function stops the continuous TX frame mode of the UWB radio.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return  None
 */
static void ull_disablecontinuousframemode(dwchip_t *dw)
{
    // NOTE: dwt_configure, dwt_configuretxrf and dwt_configcontinuousframemode must be called before a call to this API
    ull_stop_repeated_frames(dw);
    ull_force_clocks(dw, FORCE_CLK_AUTO);
    ull_disable_rf_tx(dw, 1);
    ull_disable_rftx_blocks(dw);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function stops the continuous wave mode of the UWB radio.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return  None
 */
static void ull_disablecontinuouswavemode(dwchip_t *dw)
{
    ull_repeated_cw(dw, 0, 0);
    ull_force_clocks(dw, FORCE_CLK_AUTO);
    ull_disable_rf_tx(dw, 1);
    ull_disable_rftx_blocks(dw);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function reads the raw battery voltage and temperature values of the UWB IC.
 * The values read here will be the current values sampled by UWB IC AtoD converters.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @return reading  = (temp_raw<<8)|(vbat_raw)
 */
static uint16_t ull_readtempvbat(dwchip_t *dw)
{
    uint16_t wr_buf;

    wr_buf = (ull_readsar(dw, 2U, 0U) & 0xFFU) << 8U;       // Vptat
    wr_buf |= ull_readsar(dw, 1U, 0U) & 0xFFU;              // VDD1/VDDBAT

    return  wr_buf;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Reads the SAR ADC inputs in debug mode
 * The SAR ADC is an 8-bit single-ended ADC with input range of 0-400mV
 * There is an input multiplexer to sample multiple signals. The sample time is ~200ns.
 * Internal reference voltages are provided for offset and gain correction.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @param[in]  input_mux: Mux select [0-15]
 *         0  -> VDDHV_AUX (2.2V) (Clamped to 1.45V due to ROADMAP-146)
 *         1  -> VDDBAT/16 (Use attenuation = 0 for this)
 *         2  -> tsense_ptat (133 mV - 400mV)
 *         3  -> tsense_vref (245 mV)
 *
 * @param[in]  attenuation: attenuation select [0-2]
 *         0 -> No attenuation (0.4V Fullscale)
 *         1 -> 1/4 attenuation (1.6V Fullscale)
 *         2 -> 1/16 attenuation (6.4V Fullscale)
 *
 * @note The maximum ADC input is clamped to 1.45V by the input mux supply of 2.2V plus a diode.
 *
 * @returns  reading  = (temp_raw<<8)|(vbat_raw)
 */
static uint16_t ull_readsar(dwchip_t *dw, uint8_t input_mux, uint8_t attn)
{
    uint32_t ldo_ctrl_val;
    uint16_t reading;
    uint32_t att = 0UL;

    if ((attn > 0U) && (attn <= 2U))
    {
        att = ((uint32_t)attn + 0x1UL) << SAR_TEST_DIG_AUXADC_ATTN_SEL_ULV_BIT_OFFSET;
    }

    if (input_mux > 15U)
    {
        input_mux = 1U;
    }

    // Enable the TSENSE
    dwt_write8bitoffsetreg(dw, SAR_TEST_ID, 0U, (uint8_t)SAR_TEST_SAR_RDEN_BIT_MASK);

    // turn on LDOs
    ldo_ctrl_val = dwt_read32bitoffsetreg(dw, LDO_CTRL_ID, 0U);
    dwt_modify32bitoffsetreg(dw, LDO_CTRL_ID, 0U, LDO_CTRL_MASK, LDO_CTRL_LDO_VDDMS2_EN_BIT_MASK);

    // Enable attenuation
    dwt_modify32bitoffsetreg(dw, SAR_TEST_ID, 0U, ~(uint32_t)(SAR_TEST_DIG_AUXADC_ATTN_EN_ULV_BIT_MASK | SAR_TEST_DIG_AUXADC_ATTN_SEL_ULV_BIT_MASK), att);

    // Select input mux and mux override
    dwt_write32bitoffsetreg(dw, SAR_CTRL_ID, 0U, (uint32_t)(SAR_CTRL_SAR_OVR_MUX_EN_BIT_MASK | ((uint32_t)input_mux << SAR_CTRL_SAR_FORCE_SEL_BIT_OFFSET)));

    // Run SAR
    dwt_modify32bitoffsetreg(dw, SAR_CTRL_ID, 0U, ~(uint32_t)SAR_CTRL_SAR_START_BIT_MASK, SAR_CTRL_SAR_START_BIT_MASK);

    // Wait until SAR conversion is complete.
    while ((dwt_read32bitoffsetreg(dw, SAR_STATUS_ID, SAR_STATUS_SAR_DONE_BIT_OFFSET) & SAR_STATUS_SAR_DONE_BIT_MASK) == 0UL)
        {}

    // Reading SAR
    reading = dwt_read16bitoffsetreg(dw, SAR_READING_ID, 0U);

    // Clear SAR enable
    dwt_write8bitoffsetreg(dw, SAR_CTRL_ID, SAR_CTRL_SAR_START_BIT_OFFSET, 0x00U);

    // Disable the TSENSE
    dwt_write8bitoffsetreg(dw, SAR_TEST_ID, 0U, (uint8_t)0x0U << SAR_TEST_SAR_RDEN_BIT_OFFSET);

    // restore LDO control register
    dwt_write32bitoffsetreg(dw, LDO_CTRL_ID, 0U, ldo_ctrl_val);

    // Disable attenuation
    dwt_modify32bitoffsetreg(dw, SAR_TEST_ID, 0U, ~(uint32_t)(SAR_TEST_DIG_AUXADC_ATTN_EN_ULV_BIT_MASK | SAR_TEST_DIG_AUXADC_ATTN_SEL_ULV_BIT_MASK), 0UL);

    return reading;

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function takes in a raw temperature value and applies the conversion factor
 * to give true temperature. The dwt_initialise needs to be called before call to this to
 * ensure LOCAL_DATA(dw)->tempP contains the SAR_LTEMP value from OTP.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] raw_temp: This is the 8-bit raw temperature value as read by ull_readtempvbat() [15:8]
 *
 * @return temperature sensor value
 */
static float ull_convertrawtemperature(dwchip_t *dw, uint8_t raw_temp)
{
    float realtemp;

    // the User Manual formula is: Temperature (C) = ( (SAR_LTEMP - OTP_READ(Vtemp @ 22C) ) x 1.05)        // Vtemp @ 22C
    realtemp = (((float)raw_temp - (float)LOCAL_DATA(dw)->tempP) * 1.05f) + 22.0f;
    return realtemp;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function takes in a raw voltage value and applies the conversion factor
 * to give true voltage. The dwt_initialise needs to be called before call to this to
 * ensure LOCAL_DATA(dw)->vBatP contains the SAR_LVBAT value from OTP
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] raw_voltage: This is the 8-bit raw voltage value as read by ull_readtempvbat() [7:0]
 *
 * @return voltage sensor value
 */
static float ull_convertrawvoltage(dwchip_t *dw, uint8_t raw_voltage)
{
    float realvolt;

    // Bench measurements gives approximately: VDDBAT = sar_read * Vref / max_code * 16x_atten   - assume Vref @ 3.0V
    realvolt = ((((float)raw_voltage - (float)LOCAL_DATA(dw)->vBatP) * 0.4f * 16.0f / 255.0f) + 3.0f);
    // realvolt = ((float)raw_voltage * 0.4f / 255) * 16;
    return realvolt;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function reads the temperature of the UWB IC that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if @ref DWT_RUNSAR bit is set in mode parameter of dwt_configuresleep()
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return 8-bit raw battery temperature sensor value
 */
static uint8_t ull_readwakeuptemp(dwchip_t *dw)
{
    return dwt_read8bitoffsetreg(dw, SAR_READING_ID, 1U);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function reads the battery voltage of the UWB IC that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if @ref DWT_RUNSAR bit is set in mode parameter of dwt_configuresleep()
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return 8-bit raw battery voltage sensor value
 */
static uint8_t ull_readwakeupvbat(dwchip_t *dw)
{
    return dwt_read8bitoffsetreg(dw, SAR_READING_ID, 0U);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function determines the adjusted bandwidth setting (PG_DELAY bitfield setting)
 * of the UWB IC. The adjustment is a result of UWB IC's internal PG cal routine, given a target count value it will try to
 * find the PG delay which gives the closest count value to the given target count.
 * Manual sequencing of TX blocks and TX clocks need to be enabled for either channel 5 or 9.
 * This function presumes that the PLL is already on (device is in the IDLE state). Please configure the device to IDLE
 * state before calling this function, by calling dwt_configure(), dwt_setchannel() or dwt_setdwstate().
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] target_count: The PG count target to reach in order to correct the bandwidth
 *
 * @return The value that was written to the PG_DELAY register (when calibration completed, [5:0])
 */
static uint8_t ull_calcbandwidthadj(dwchip_t *dw, uint16_t target_count)
{
    // Force system clock to FOSC/4 and TX clocks on and enable RF blocks
    ull_force_clocks(dw, FORCE_CLK_SYS_TX);
    ull_enable_rf_tx(dw, 0, 0);
    ull_enable_rftx_blocks(dw);

    // Write to the PG target before kicking off PG auto-cal with given target value
    dwt_write16bitoffsetreg(dw, PG_CAL_TARGET_ID, 0x0U, target_count & PG_CAL_TARGET_TARGET_BIT_MASK);
    // Run PG count cal
    dwt_or8bitoffsetreg(dw, PGC_CTRL_ID, 0x0, (uint8_t)(PGC_CTRL_PGC_START_BIT_MASK | PGC_CTRL_PGC_AUTO_CAL_BIT_MASK));
    // Wait for calibration to complete
    while ((dwt_read8bitoffsetreg(dw, PGC_CTRL_ID, 0U) & PGC_CTRL_PGC_START_BIT_MASK) != 0U)
        {}

    // Restore clocks to AUTO and turn off TX blocks
    ull_disable_rftx_blocks(dw);
    ull_disable_rf_tx(dw, 0);
    ull_force_clocks(dw, FORCE_CLK_AUTO);

    return (dwt_read8bitoffsetreg(dw, TX_CTRL_HI_ID, 0U) & TX_CTRL_HI_TX_PG_DELAY_BIT_MASK);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function calculates the value in the pulse generator counter register (PGC_STATUS) for a given PG_DELAY
 * This is used to take a reference measurement, and the value recorded as the reference is used to adjust the
 * bandwidth of the device when the temperature changes. This function presumes that the PLL is already on (device is in the IDLE state).
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] pgdly: The PG_DELAY (max value 63) to set (to control bandwidth), and to find the corresponding count value for
 *
 * @return The count value calculated from the provided PG_DELAY value (from PGC_STATUS) - used as reference
 * for later bandwidth adjustments [11:0]
 */
static uint16_t ull_calcpgcount(dwchip_t *dw, uint8_t pgdly)
{
    uint16_t count = 0U;

    // Force system clock to FOSC/4 and TX clocks on
    ull_force_clocks(dw, FORCE_CLK_SYS_TX);
    ull_enable_rf_tx(dw, 0, 0);
    ull_enable_rftx_blocks(dw);

    dwt_write8bitoffsetreg(dw, TX_CTRL_HI_ID, TX_CTRL_HI_TX_PG_DELAY_BIT_OFFSET, pgdly & TX_CTRL_HI_TX_PG_DELAY_BIT_MASK);

    // Run count cal
    dwt_or8bitoffsetreg(dw, PGC_CTRL_ID, 0x0U, PGC_CTRL_PGC_START_BIT_MASK);
    // Wait for calibration to complete
    while ((dwt_read8bitoffsetreg(dw, PGC_CTRL_ID, 0U) & PGC_CTRL_PGC_START_BIT_MASK) != 0U)
        {}
    count = dwt_read16bitoffsetreg(dw, PGC_STATUS_ID, PGC_STATUS_PG_DELAY_COUNT_BIT_OFFSET) & PGC_STATUS_PG_DELAY_COUNT_BIT_MASK;

    ull_disable_rftx_blocks(dw);
    ull_disable_rf_tx(dw, 0U);
    ull_force_clocks(dw, FORCE_CLK_AUTO);

    return count;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Read the current value of the PLL status register (32-bits)
 * The status bits are defined as follows:
 *
 * - PLL_STATUS_LD_CODE_BIT_MASK          0x1f00U - Counter-based lock-detect status indicator
 * - PLL_STATUS_XTAL_AMP_SETTLED_BIT_MASK 0x40U   - Status flag from the XTAL indicating that the amplitude has settled
 * - PLL_STATUS_VCO_TUNE_UPDATE_BIT_MASK  0x20U   - Flag to indicate that the COARSE_TUNE codes have been updated by cal and are ready to read
 * - PLL_STATUS_PLL_OVRFLOW_BIT_MASK      0x10U   - PLL calibration flag indicating all VCO_TUNE values have been cycled through
 * - PLL_STATUS_PLL_HI_FLAG_BIT_MASK      0x8U    - VCO voltage too high indicator (active-high)
 * - PLL_STATUS_PLL_LO_FLAG_N_BIT_MASK    0x4U    - VCO voltage too low indicator (active-low)
 * - PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK    0x2U    - PLL lock flag
 * - PLL_STATUS_CPC_CAL_DONE_BIT_MASK     0x1U    - PLL cal done and PLL locked
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return A value containing the value of the PLL status register (only bits [14:0] are valid)
 */
static uint32_t ull_readpllstatus(dwchip_t *dw)
{
    return dwt_read32bitoffsetreg(dw, PLL_STATUS_ID, 0x0U);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will re-calibrate and re-lock the PLL. If the cal/lock is successful @ref DWT_SUCCESS
 * will be returned otherwise @ref DWT_ERROR will be returned
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
 */
static int32_t ull_pll_cal(dwchip_t *dw)
{
    (void)ull_setdwstate(dw, (int32_t)DWT_DW_IDLE_RC);
    return ull_setdwstate(dw, (int32_t)DWT_DW_IDLE);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function is used to control what RF port to use for TX/RX.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] port_control: Enum value for enabling or disabling manual control and primary antenna, see @ref dwt_rf_port_ctrl_e
 *
 * @return None
 */
static void ull_configure_rf_port(dwchip_t *dw, dwt_rf_port_ctrl_e port_control)
{
    uint32_t set_bits_val = 0;
    uint32_t p_ctrl = (uint32_t) port_control;
    const uint32_t bit_mask = ~(RF_SWITCH_CTRL_ANT_SW_PDOA_PORT_BIT_MASK |
				RF_SWITCH_CTRL_ANTSWCTRL_BIT_MASK |
				RF_SWITCH_CTRL_ANTSWEN_BIT_MASK
				);
    if (p_ctrl < (uint32_t)DWT_RF_PORT_AUTO_1_2) {
	/* Manual control. */
	set_bits_val = ((1UL << RF_SWITCH_CTRL_ANTSWEN_BIT_OFFSET) |
	                ((p_ctrl) << RF_SWITCH_CTRL_ANTSWCTRL_BIT_OFFSET));
    }
    else {
	/* Automatic PDoA switch. */
	set_bits_val = ((p_ctrl - (uint32_t)DWT_RF_PORT_AUTO_1_2) << RF_SWITCH_CTRL_ANT_SW_PDOA_PORT_BIT_OFFSET);
    }

    dwt_modify32bitoffsetreg(dw, RF_SWITCH_CTRL_ID, 0, bit_mask, set_bits_val);
}


/********************************************************************************************************************/
/*                                                AES BLOCK                                                         */
/********************************************************************************************************************/

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function provides the API for the configuration of the AES block before its first usage.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] pCfg: Pointer to the configuration structure, which contains the AES configuration data, see @ref dwt_aes_config_t
 *
 * @return  None
 */
static void ull_configure_aes(dwchip_t *dw, const dwt_aes_config_t *pCfg)
{
    uint32_t tmp;

    tmp = (uint32_t)pCfg->mode;
    tmp |= ((uint32_t)pCfg->key_size) << AES_CFG_KEY_SIZE_BIT_OFFSET;
    tmp |= ((uint32_t)pCfg->key_addr) << AES_CFG_KEY_ADDR_BIT_OFFSET;
    tmp |= ((uint32_t)pCfg->key_load) << AES_CFG_KEY_LOAD_BIT_OFFSET;
    tmp |= ((uint32_t)pCfg->key_src) << AES_CFG_KEY_SRC_BIT_OFFSET;
    tmp |= ((uint32_t)pCfg->mic) << AES_CFG_TAG_SIZE_BIT_OFFSET;
    tmp |= ((uint32_t)pCfg->aes_core_type) << AES_CFG_CORE_SEL_BIT_OFFSET;
    tmp |= ((uint32_t)pCfg->aes_key_otp_type) << AES_CFG_KEY_OTP_BIT_OFFSET;

    dwt_write16bitoffsetreg(dw, AES_CFG_ID, 0U, (uint16_t)tmp);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This gets the mic size in bytes and convert it to value to write in AES_CFG
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] mic_size_in_bytes: mic size in bytes.
 *
 * @return  @ref dwt_mic_size_e value
 */

static dwt_mic_size_e ull_mic_size_from_bytes(dwchip_t *dw, uint8_t mic_size_in_bytes)
{
    (void)dw;
    uint8_t mic_size = (uint8_t)MIC_0;

    if (mic_size_in_bytes != 0U)
    {
        mic_size = ((mic_size_in_bytes >> 1U) - 1U);
    }
    return (dwt_mic_size_e)mic_size;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function provides the API for the configuration of the AES key before the first usage.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] key: Pointer to the key which will be programmed to the AES_KEY register, see @ref dwt_aes_key_t
 *                 Note the AES_KEY register supports only 128-bit keys.
 *
 * @return  None
 */
static void ull_set_keyreg_128(dwchip_t *dw, const dwt_aes_key_t *key)
{
    /* program Key to the register : only 128 bit key can be used */
    dwt_write32bitreg(dw, AES_KEY0_ID, (uint32_t)key->key0);
    dwt_write32bitreg(dw, AES_KEY1_ID, (uint32_t)key->key1);
    dwt_write32bitreg(dw, AES_KEY2_ID, (uint32_t)key->key2);
    dwt_write32bitreg(dw, AES_KEY3_ID, (uint32_t)key->key3);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  Poll AES block waiting for completion of encrypt/decrypt job
 *         It clears all received errors/statuses.
 *         This is not a safe function as it is waiting for AES_STS_AES_DONE_BIT_MASK forever.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return  AES_STS_ID status [5:0]
 */
#define AES_STATUS_MASK (AES_STS_RAM_FULL_BIT_MASK | AES_STS_RAM_EMPTY_BIT_MASK | AES_STS_MEM_CONF_BIT_MASK \
                         | AES_STS_TRANS_ERR_BIT_MASK | AES_STS_AUTH_ERR_BIT_MASK | AES_STS_AES_DONE_BIT_MASK)
static uint8_t ull_wait_aes_poll(dwchip_t *dw)
{
    uint8_t tmp;
    do {
        tmp = dwt_read8bitoffsetreg(dw, AES_STS_ID, 0U);
    } while ((tmp & (AES_STS_AES_DONE_BIT_MASK | AES_STS_TRANS_ERR_BIT_MASK)) == 0U);

    dwt_write8bitoffsetreg(dw, AES_STS_ID, 0U, tmp); // clear all bits which were set as a result of AES operation

    return (tmp & AES_STATUS_MASK);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function update the core IV regs according to the core type (The DW3720 AES block is different from DW3000)
 *
 * DW3000 IC stores the nonce in AES_IV0_ID to AES_IV3_ID registers.
 * DW3000 IC, when operating in CCM* AES mode expects the nonce to be programmed into 4 words as follows:
 * AES_IV0_ID[0] = nonce[10]
 * AES_IV0_ID[1] = nonce[9]
 * AES_IV0_ID[2] = nonce[8]
 * AES_IV0_ID[3] = nonce[7]
 * AES_IV1_ID[0] = nonce[6]
 * AES_IV1_ID[1] = nonce[5]
 * AES_IV1_ID[2] = nonce[4]
 * AES_IV1_ID[3] = nonce[3]
 * AES_IV2_ID[0] = nonce[2]
 * AES_IV2_ID[1] = nonce[1]
 * AES_IV2_ID[2] = nonce[0]
 * AES_IV2_ID[3] = don't care
 * AES_IV3_ID[0] = payload_length[0]
 * AES_IV3_ID[1] = payload_length[1]
 * AES_IV3_ID[2] = nonce[12]
 * AES_IV3_ID[3] = nonce[11]
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] nonce: Pointer to the nonce
 * @param[in] payload: Length of data payload to encrypt/decrypt
 *
 * @return None
 */
static void ull_update_nonce_CCM(dwchip_t *dw, uint8_t *nonce, uint16_t payload)
{
    uint8_t iv[16];
    iv[0] = nonce[10];
    iv[1] = nonce[9];
    iv[2] = nonce[8];
    iv[3] = nonce[7];
    iv[4] = nonce[6];
    iv[5] = nonce[5];
    iv[6] = nonce[4];
    iv[7] = nonce[3];
    iv[8] = nonce[2];
    iv[9] = nonce[1];
    iv[10] = nonce[0];
    iv[11] = 0x00U; // don't care
    iv[12] = (uint8_t)payload;
    iv[13] = (uint8_t)(payload >> 8U);
    iv[14] = nonce[12];
    iv[15] = nonce[11];

    ull_writetodevice(dw, AES_IV0_ID, 0U, 16U, iv);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function update the nonce-IV regs when using GCM AES core type
 *
 * DW3000 IC, when operating in GCM AES mode expects the nonce to be programmed into 3 words as follows:
 * LSB (of nonce array) sent first. Nonce array is made up of 12 bytes.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] nonce: Pointer to the nonce value to set
 *
 * @return None
 */
static void ull_update_nonce_GCM(dwchip_t *dw, uint8_t *nonce)
{
    ull_writetodevice(dw, AES_IV0_ID, 0U, 12U, nonce);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function provides the API for the job of encryption/decryption of a data block
 *
 *          128 bit key shall be pre-loaded with dwt_set_aes_key()
 *          dwt_configure_aes()
 *
 *          supports AES_KEY_Src_Register mode only
 *          packet sizes < 127
 *          @note The "nonce" shall be unique for every transaction
 *
 *          Prior to calling this function the AES configuration needs to be set via dwt_configure_aes() and associated @ref dwt_aes_config_t:
 *          e.g., .key_load           = AES_KEY_Load,
 *                .key_size           = AES_KEY_128bit,
 *                .key_src            = AES_KEY_Src_RAMorOTP,
 *                .mic                = MIC_8,
 *                .mode               = AES_Encrypt,
 *                .aes_core_type      = AES_core_type_CCM,
 *                .aes_key_otp_type   = AES_key_OTP,
 *                .aes_otp_sel_key_block = AES_key_otp_sel_1st_128,
 *                .key_addr           = 0
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] job: Pointer to AES job, contains data info and encryption info, see @ref dwt_aes_job_t
 * @param[in] core_type: Core type (CCM* or GCM), see @ref dwt_aes_core_type_e
 *
 * @return  AES_STS_ID status bits
 */
static int8_t ull_do_aes(dwchip_t *dw, dwt_aes_job_t *job, dwt_aes_core_type_e core_type)
{
    uint32_t tmp, dest_reg;
    uint16_t allow_size;
    uint8_t ret;
    dwt_aes_src_port_e src_port;
    dwt_aes_dst_port_e dst_port;

    if (job->mic_size == MIC_ERROR)
    {
        return ERROR_WRONG_MIC_SIZE;
    }

    /* program Initialization Vector
     * */
    if (core_type == AES_core_type_GCM)
    {
        ull_update_nonce_GCM(dw, job->nonce);
    }
    else
    {
        ull_update_nonce_CCM(dw, job->nonce, job->payload_len);
    }

    /* write data to be encrypted.
     * for decryption data should be present in the src buffer
     * */
    tmp = (uint32_t)job->header_len + (uint32_t)job->payload_len;

    if (job->mode == AES_Encrypt)
    {
        if (job->src_port == AES_Src_Scratch)
        {
            allow_size = SCRATCH_BUFFER_MAX_LEN;
            dest_reg = SCRATCH_RAM_ID;
        }
        else
        {
            allow_size = TX_BUFFER_MAX_LEN;
            dest_reg = TX_BUFFER_ID;
        }
    }
    else if (job->mode == AES_Decrypt)
    {
        if (job->dst_port == AES_Dst_Scratch)
        {
            allow_size = SCRATCH_BUFFER_MAX_LEN;
        }
        else
        {
            allow_size = RX_BUFFER_MAX_LEN;
        }
    }
    else
    {
        return ERROR_WRONG_MODE;
    }

    if (tmp > ((uint32_t)allow_size - (uint32_t)job->mic_size - FCS_LEN))
    {
        return ERROR_DATA_SIZE;
    }

    /* write data to be encrypted.
     * for decryption data should be present in the src buffer
     * */
    if (job->mode == AES_Encrypt)
    {
        ull_writetodevice(dw, dest_reg, 0U, job->header_len, job->header);                 /*!< non-encrypted header */
        ull_writetodevice(dw, dest_reg, job->header_len, job->payload_len, job->payload); /*!< data to be encrypted */
    }

    /* Set SRC and DST ports in memory.
     * Current implementation uses frames started from start of the desired port
     * */
    src_port = job->src_port;
    if ((job->src_port == AES_Src_Rx_buf_0) || (job->src_port == AES_Src_Rx_buf_1))
    {
        if (LOCAL_DATA(dw)->dblbuffon == (uint8_t)DBL_BUFF_ACCESS_BUFFER_1) // if the flag is 0x3 we are reading from RX_BUFFER_1
        {
            src_port = AES_Src_Rx_buf_1;
        }
        else
        {
            src_port = AES_Src_Rx_buf_0;
        }
    }

    dst_port = job->dst_port;
    if ((job->dst_port == AES_Dst_Rx_buf_0) || (job->dst_port == AES_Dst_Rx_buf_1))
    {
        if (LOCAL_DATA(dw)->dblbuffon == (uint8_t)DBL_BUFF_ACCESS_BUFFER_1) // if the flag is 0x3 we are reading from RX_BUFFER_1
        {
            dst_port = AES_Dst_Rx_buf_1;
        }
        else
        {
            dst_port = AES_Dst_Rx_buf_0;
        }
    }
    else if (job->dst_port == AES_Dst_STS_key)
    {
        if (job->payload_len > STS_LEN_128BIT) // when writing to STS registers (destination port) payload cannot exceed 16 bytes
        {
            return ERROR_PAYLOAD_SIZE;
        }
    }
    else
    {
        /* Nothing to do */
    }

    tmp = (((uint32_t)src_port) << DMA_CFG0_SRC_PORT_BIT_OFFSET) | (((uint32_t)dst_port) << DMA_CFG0_DST_PORT_BIT_OFFSET);

    dwt_write32bitreg(dw, DMA_CFG0_ID, tmp);

    /* fill header length and payload length - the only known information of the frame.
     * Note, the payload length does not include MIC and FCS lengths.
     * So, if overall rx_frame length is 100 and header length is 10, MIC is 16 and FCS is 2,
     * then payload length is 100-10-16-2 = 72 bytes.
     * */
    tmp = (DMA_CFG1_HDR_SIZE_BIT_MASK & (((uint32_t)job->header_len) << DMA_CFG1_HDR_SIZE_BIT_OFFSET))
          | (DMA_CFG1_PYLD_SIZE_BIT_MASK & (((uint32_t)job->payload_len) << DMA_CFG1_PYLD_SIZE_BIT_OFFSET));

    dwt_write32bitreg(dw, DMA_CFG1_ID, tmp);

    /* start AES action encrypt/decrypt */
    dwt_write8bitoffsetreg(dw, AES_START_ID, 0U, AES_START_AES_START_BIT_MASK);
    ret = ull_wait_aes_poll(dw);

    /* Read plain header and decrypted payload on correct AES decryption
     * and if instructed to do so, i.e. if job->mode == AES_Decrypt and
     * job->header or job->payload addresses are exist
     * */

    if (((ret & ~(AES_STS_RAM_EMPTY_BIT_MASK | AES_STS_RAM_FULL_BIT_MASK)) == AES_STS_AES_DONE_BIT_MASK) && (job->mode == AES_Decrypt))
    {
        uint32_t read_addr;

        if ((job->dst_port == AES_Dst_Rx_buf_0) || (job->dst_port == AES_Dst_Rx_buf_1))
        {
            if (LOCAL_DATA(dw)->dblbuffon == (uint8_t)DBL_BUFF_ACCESS_BUFFER_1) // if the flag is 0x3 we are reading from RX_BUFFER_1
            {
                read_addr = RX_BUFFER_1_ID;
            }
            else
            {
                read_addr = RX_BUFFER_0_ID;
            }
        }
        else if (job->dst_port == AES_Dst_Tx_buf)
        {
            read_addr = TX_BUFFER_ID;
        }
        else
        {
            read_addr = SCRATCH_RAM_ID;
        }

        if (job->header != NULL)
        {
            if (job->header_len != 0U)
            {
                ull_readfromdevice(dw, read_addr, 0U, job->header_len, job->header);
            }
        }

        if (job->payload != NULL)
        {
            if (job->payload_len != 0U)
            {
                ull_readfromdevice(dw, read_addr, job->header_len, job->payload_len, job->payload);
            }
        }
    }
    return ((int8_t)ret);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function configures just the SFD type: e.g., IEEE 4a - 8, DW-8, DW-16, or IEEE 4z -8 (binary)
 * The dwt_configure() should be called prior to this to configure other parameters
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] sfdType: e.g, @ref DWT_SFD_IEEE_4A, @ref DWT_SFD_DW_8, @ref DWT_SFD_DW_16, @ref DWT_SFD_IEEE_4Z, see @ref dwt_sfd_type_e
 *
 * @return None
 */
static void ull_configuresfdtype(dwchip_t *dw, uint8_t sfdType)
{
    dwt_modify32bitoffsetreg(
        dw, CHAN_CTRL_ID, 0U, ~(uint32_t)CHAN_CTRL_SFD_TYPE_BIT_MASK, (CHAN_CTRL_SFD_TYPE_BIT_MASK & ((uint32_t)sfdType << CHAN_CTRL_SFD_TYPE_BIT_OFFSET)));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function just sets the TX preamble code: 1-8 PRF16 9-24 PRF64
 * The dwt_configure() should be called prior to this to configure other parameters
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] tx_code: 1-8 PRF16, 9-24 PRF64
 *
 * @return None
 */
static void ull_settxcode(dwchip_t *dw, uint8_t tx_code)
{
    dwt_modify32bitoffsetreg(
        dw, CHAN_CTRL_ID, 0U, ~(uint32_t)CHAN_CTRL_TX_PCODE_BIT_MASK, (CHAN_CTRL_TX_PCODE_BIT_MASK & ((uint32_t)tx_code << CHAN_CTRL_TX_PCODE_BIT_OFFSET)));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function just sets the RX preamble code: 1-8 PRF16 9-24 PRF64
 * The dwt_configure() should be called prior to this to configure other parameters
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] rx_code: 1-8 PRF16, 9-24 PRF64
 *
 * @return None
 */
static void ull_setrxcode(dwchip_t *dw, uint8_t rx_code)
{
    dwt_modify32bitoffsetreg(
        dw, CHAN_CTRL_ID, 0U, ~(uint32_t)CHAN_CTRL_RX_PCODE_BIT_MASK, (CHAN_CTRL_RX_PCODE_BIT_MASK & ((uint32_t)rx_code << CHAN_CTRL_RX_PCODE_BIT_OFFSET)));

    ull_update_dgc_config(dw, LOCAL_DATA(dw)->channel);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function writes a value to the system status register (lower).
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] mask: mask value to send to the system status register (lower 32-bits).
 *               e.g., "SYS_STATUS_TXFRS_BIT_MASK" to clear the TX frame sent event.
 *
 * @return None
 */
static void ull_writesysstatuslo(dwchip_t *dw, uint32_t mask)
{
    dwt_write32bitreg(dw, SYS_STATUS_ID, mask);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function writes a value to the system status register (higher).
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] mask: mask value to send to the system status register (higher 32-bits).
 *
 * @return None
 */
static void ull_writesysstatushi(dwchip_t *dw, uint32_t mask)
{
    dwt_write16bitoffsetreg(dw, SYS_STATUS_HI_ID, 0U, (uint16_t)mask);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function reads the current value of the system status register (lower 32 bits)
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return A uint32_t value containing the value of the system status register (lower 32 bits)
 */
static uint32_t ull_readsysstatuslo(dwchip_t *dw)
{
    return dwt_read32bitoffsetreg(dw, SYS_STATUS_ID, 0U);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function reads the current value of the system status register (higher 16 bits)
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return A value containing the value of the system status register (higher 16 bits)
 */
static uint16_t ull_readsysstatushi(dwchip_t *dw)
{
    /*
     * DW3000 SYS_STATUS_HI register is 13 bits wide, thus we revert to a 16-bit read of the register
     * instead of using a 32-bit read. Only DW3720 devices require a 32-bit write.
     */
    return dwt_read16bitoffsetreg(dw, SYS_STATUS_HI_ID, 0U);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function writes a value to the receiver double buffer status register.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] mask: mask value to send to the register.
 *               e.g. "RDB_STATUS_CLEAR_BUFF0_EVENTS" to clear the clear buffer 0 events.
 *
 * @return None
 */
static void ull_writerdbstatus(dwchip_t *dw, uint8_t mask)
{
    dwt_write8bitoffsetreg(dw, RDB_STATUS_ID, 0U, mask);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function reads the current value of the Receiver Double Buffer status register.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * return A value containing the value of the receiver double buffer status register.
 */
static uint8_t ull_readrdbstatus(dwchip_t *dw)
{
    return dwt_read8bitoffsetreg(dw, SYS_STATUS_ID, 0U);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will read the frame length of the last received frame.
 *        This function presumes that a good frame or packet has been received.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] rng_bit: This is an output, the parameter will have DWT_CB_DATA_RX_FLAG_RNG set if RNG bit is set in FINFO
 *
 * return frame_len - the number of octets in the received frame.
 */
static uint16_t ull_getframelength(dwchip_t *dw, uint8_t *rng_bit)
{
    uint16_t finfo16;

    // Read frame info - Only the first two bytes of the register are used here.
    switch ((dwt_dbl_buff_conf_e)LOCAL_DATA(dw)->dblbuffon)
    // check if in double buffer mode and if so which buffer host is currently accessing
    {
    case DBL_BUFF_ACCESS_BUFFER_1:                                                       // accessing frame info relating to the second buffer (RX_BUFFER_1)
        dwt_write8bitoffsetreg(dw, RDB_STATUS_ID, 0U, DWT_RDB_STATUS_CLEAR_BUFF1_EVENTS); // clear DB status register bits corresponding to RX_BUFFER_1
        finfo16 = dwt_read16bitoffsetreg(dw, INDIRECT_POINTER_B_ID, 0U);
        break;
    case DBL_BUFF_ACCESS_BUFFER_0:                                                       // accessing frame info relating to the first buffer (RX_BUFFER_0)
        dwt_write8bitoffsetreg(dw, RDB_STATUS_ID, 0U, DWT_RDB_STATUS_CLEAR_BUFF0_EVENTS); // clear DB status register bits corresponding to RX_BUFFER_0
        finfo16 = dwt_read16bitoffsetreg(dw, BUF0_RX_FINFO, 0U);
        break;
    default: // accessing frame info relating to the second buffer (RX_BUFFER_0) (single buffer mode)
        finfo16 = dwt_read16bitoffsetreg(dw, RX_FINFO_ID, 0U);
        break;
    }

    // Report ranging bit
    if ((finfo16 & RX_FINFO_RNG_BIT_MASK) != 0U)
    {
        *rng_bit |= (uint8_t)DWT_CB_DATA_RX_FLAG_RNG;
    }
    else
    {
        *rng_bit &= ~(uint8_t)DWT_CB_DATA_RX_FLAG_RNG;
    }

    // Report frame length - Standard frame length up to 127, extended frame length up to 1023 bytes
    if (LOCAL_DATA(dw)->longFrames == 0U)
    {
        finfo16 &= (uint16_t)RX_FINFO_STD_RXFLEN_MASK;
        LOCAL_DATA(dw)->cbData.datalength = finfo16;
    }
    else
    {
        finfo16 &= RX_FINFO_RXFLEN_BIT_MASK;
        LOCAL_DATA(dw)->cbData.datalength = finfo16;
    }

    return finfo16;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function drive the antenna configuration GPIO6/7
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] antenna_config: Configure GPIO 6 and or 7 to use for Antenna selection with expected value.
 *      bitfield configuration:
 *      Bit 0: Use GPIO 6
 *      Bit 1: Value to apply (0/1)
 *      Bit 2: Use GPIO 7
 *      Bit 3: Value to apply (0/1)
 *
 * @return None
 */
static void ull_configure_and_set_antenna_selection_gpio(dwchip_t *dw, uint8_t antenna_config)
{
    uint32_t gpio_mode_cfg = 0x0UL;
    uint32_t gpio_mode_flag = 0x0UL;
    /* GPIO 6 and 7 will be set as Output, no need to change this value */
    uint16_t gpio_dir_cfg = 0x0U;
    uint16_t gpio_dir_flag = 0x0U;

    uint16_t gpio_out_cfg = 0x0U;
    uint16_t gpio_out_flag = 0x0U;

    if ((antenna_config & ANT_GPIO6_POS_MASK) != 0U)
    {
        gpio_mode_flag |= GPIO_MODE_MSGP6_MODE_BIT_MASK;
        gpio_dir_flag |= GPIO_DIR_GDP6_BIT_MASK;
        gpio_out_cfg |= ((((uint16_t)antenna_config & ANT_GPIO6_VAL_MASK) >> ANT_GPIO6_VAL_OFFSET) << GPIO_OUT_GOP6_BIT_OFFSET);
        gpio_out_flag |= GPIO_OUT_GOP6_BIT_MASK;
    }

    if ((antenna_config & ANT_GPIO7_POS_MASK) != 0U)
    {
        gpio_mode_cfg |= (0x1UL << GPIO_MODE_MSGP7_MODE_BIT_OFFSET);
        gpio_mode_flag |= GPIO_MODE_MSGP7_MODE_BIT_MASK;
        gpio_dir_flag |= GPIO_DIR_GDP7_BIT_MASK;
        gpio_out_cfg |= ((((uint16_t)antenna_config & ANT_GPIO7_VAL_MASK) >> ANT_GPIO7_VAL_OFFSET) << GPIO_OUT_GOP7_BIT_OFFSET);
        gpio_out_flag |= GPIO_OUT_GOP7_BIT_MASK;
    }

    dwt_modify32bitoffsetreg(dw, GPIO_MODE_ID, 0U, ~(gpio_mode_flag), gpio_mode_cfg);
    dwt_modify16bitoffsetreg(dw, GPIO_DIR_ID, 0U, ~(gpio_dir_flag), gpio_dir_cfg);
    dwt_modify16bitoffsetreg(dw, GPIO_OUT_ID, 0U, ~(gpio_out_flag), gpio_out_cfg);

    return;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function can set GPIO output to high (1) or low (0) which can then be used to signal e.g., WiFi chip to
 * turn off or on. This can be used in devices with multiple radios to minimise co-existence interference.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] enable: Specifies if to enable or disable WiFi co-ex functionality on GPIO5 (or GPIO4)
 *                       depending if coex_io_swap param is set to 1 or 0
 * @param[in] coex_io_swap: When set to 0, GPIO5 is used as co-ex out, otherwise GPIO4 is used
 *
 * @return event counts from both timers: @ref DWT_TIMER0 events in bits [7:0], @ref DWT_TIMER1 events in bits [15:8]
 */
static void ull_wifi_coex_set(dwchip_t *dw, dwt_wifi_coex_e enable, int32_t coex_io_swap)
{
    uint32_t mode = (coex_io_swap == 1) ? ~(uint32_t)GPIO4_FUNC_MASK : ~(uint32_t)GPIO5_FUNC_MASK;
    uint8_t dir_out_off = (coex_io_swap == 1) ? ~(uint8_t)GPIO4_BIT_MASK : ~(uint8_t)GPIO5_BIT_MASK;
    uint8_t dir_out_on = (coex_io_swap == 1) ? (uint8_t)GPIO4_BIT_MASK : (uint8_t)GPIO5_BIT_MASK;

    dwt_and32bitoffsetreg(dw, GPIO_MODE_ID, 0U, mode);
    dwt_and8bitoffsetreg(dw, GPIO_DIR_ID, 0U, dir_out_off); // GPIO set to output

    if (enable == DWT_DIS_WIFI_COEX)
    {
        dwt_and8bitoffsetreg(dw, GPIO_OUT_ID, 0U, dir_out_off); // GPIO set to 0
    }

    if (enable == DWT_EN_WIFI_COEX)
    {
        dwt_or8bitoffsetreg(dw, GPIO_OUT_ID, 0U, dir_out_on); // GPIO set to 1
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will reset the internal system time counter. The counter will be momentarily reset to 0,
 * and then will continue counting as normal. The system time/counter is only available when device is in
 * IDLE or TX/RX states.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @return None
 */
static void ull_reset_system_counter(dwchip_t *dw)
{
    // the two bits: OSTR_MODE and FORCE_SYNC need to be set and then cleared to reset system time/counter
    dwt_or8bitoffsetreg(dw, EC_CTRL_ID, 0x1U, (EC_CTRL_OSTR_MODE_BIT_MASK >> 8U));
    dwt_or8bitoffsetreg(dw, SEQ_CTRL_ID, 0x3U, (uint8_t)(SEQ_CTRL_FORCE_SYNC_BIT_MASK >> 24U));

    // clear the bits again
    dwt_and8bitoffsetreg(dw, EC_CTRL_ID, 0x1U,  ~(uint8_t)(EC_CTRL_OSTR_MODE_BIT_MASK >> 8U));
    dwt_and8bitoffsetreg(dw, SEQ_CTRL_ID, 0x3U,  ~(uint8_t)(SEQ_CTRL_FORCE_SYNC_BIT_MASK >> 24U));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function is used to configure the device for OSTR mode (One Shot Timebase Reset mode), this will
 * prime the device to reset the internal system time counter on SYNC pulse / SYNC pin toggle.
 * For more information on this operation please consult the device User Manual.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] enable:    Set to 1 to enable OSTR mode and 0 to disable
 * @param[in] wait_time: When a counter running on the 38.4 MHz external clock and initiated on the rising edge
 *                       of the SYNC signal equals the WAIT programmed value, the UWB IC timebase counter will be reset.
 *
 * @note At the time the SYNC signal is asserted, the clock PLL dividers generating the DW3000 125 MHz system clock are reset,
 * to ensure that a deterministic phase relationship exists between the system clock and the asynchronous 38.4 MHz external clock.
 * For this reason, the WAIT value programmed will dictate the phase relationship and should be chosen to give the
 * desired phase relationship, as given by WAIT modulo 4. A WAIT value of 33 decimal is recommended,
 * but if a different value is chosen it should be chosen so that WAIT modulo 4 is equal to 1, i.e. 29, 37, and so on.
 *
 * @return None
 */
static void ull_config_ostr_mode(dwchip_t *dw, uint8_t enable, uint16_t wait_time)
{
    uint16_t temp = (wait_time << EC_CTRL_OSTS_WAIT_BIT_OFFSET) & EC_CTRL_OSTS_WAIT_BIT_MASK;
    if (enable != 0U)
    {
        temp |= EC_CTRL_OSTR_MODE_BIT_MASK;
    }

    dwt_modify16bitoffsetreg(dw, EC_CTRL_ID, 0x0U, (uint16_t) ~(EC_CTRL_OSTS_WAIT_BIT_MASK | EC_CTRL_OSTR_MODE_BIT_MASK), temp);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function calculates the adjusted Tx power setting by applying a boost over a reference Tx power setting.
 * The reference Tx power setting should correspond to a 1 ms frame (or 0 dB boost).
 * The boost (power increase) to be applied should be provided in unit of 0.1dB.
 * For example, for a 125 us packet, a theoretical boost of 9 dB can be applied, compared to a packet of 1 ms.
 * A boost of '90' should be provided as input parameter and will be applied over the reference Tx power setting for a 1 ms frame.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] boost: The boost to apply in 0.1 dB units.
 * @parblock
 * UWB radio maximum boost is 354 in channel 5, 305 in channel 9. If the input value is greater than the maximum boost,
 * then it will be discarded and maximum boost will be targeted.
 * @endparblock
 * @param[in] ref_tx_power: The Tx power_setting corresponding to a frame of 1 ms (0 dB boost)
 * @param[in] channel: The current RF channel used for transmission of UWB packets
 * @param[out] adj_tx_power: If successful, the adjusted Tx power setting will be returned through this pointer
 * @param[out] applied_boost: If successful, the exact amount of boost applied will be returned through this pointer
 *
 * @return
 * @ref DWT_SUCCESS if an adjusted Tx power setting could be calculated. In this case, the actual amount of boost that was
 * applied and the adjusted Tx power setting will be respectively returned through the parameters adj_tx_power and boost
 * @ref DWT_ERROR if the API could not calculate a valid adjusted Tx power setting
 */
static int32_t ull_adjust_tx_power(uint16_t boost, uint32_t ref_tx_power, uint8_t channel, uint32_t* adj_tx_power, uint16_t* applied_boost )
{
    uint16_t target_boost = 0U;
    uint16_t current_boost = 0U;
    uint16_t best_boost_abs = 0U;
    uint16_t best_boost = 0U;
    uint16_t upper_limit = 0U;
    uint16_t lower_limit = 0U;
    const uint8_t* lut = NULL;
    uint8_t best_index = 0U;
    uint8_t best_coarse_gain = 0U;
    uint8_t within_margin_flag = 0U;
    uint8_t reached_max_fine_gain_flag = 0U;
    uint8_t unlock = 0U;
    uint8_t tx_power_byte = 0U;
    uint8_t i = 0U;

    uint8_t ref_coarse_gain = (uint8_t) (ref_tx_power & TX_POWER_COARSE_GAIN_MASK);
    uint8_t ref_fine_gain = (uint8_t) ((ref_tx_power >> 2UL) & TX_POWER_FINE_GAIN_MASK);

    switch(channel)
    {
        case ((uint8_t) DWT_CH5) :
            lut = &fine_gain_lut_chan5[0];
            target_boost = boost < MAX_BOOST_CH5 ? boost : MAX_BOOST_CH5;
            break;

        default:    // If not chan 5 then default = 9
            lut = &fine_gain_lut_chan9[0];
            target_boost = boost < MAX_BOOST_CH9 ? boost : MAX_BOOST_CH9;
            break;
    }

    // Initial conditions
    i= ref_fine_gain;
    upper_limit =  target_boost + TXPOWER_ADJUSTMENT_MARGIN;
    lower_limit = (target_boost > TXPOWER_ADJUSTMENT_MARGIN ? target_boost - TXPOWER_ADJUSTMENT_MARGIN : 0U); // If substraction overflow => 0
    best_boost_abs = TXPOWER_ADJUSTMENT_MARGIN;

    // Checking if next fine gain setting is sufficient to be within margin of target boost.
    // If not, if the target_boost is within margin, then we do not apply any boost and return ref TxPower.
    if((target_boost < TXPOWER_ADJUSTMENT_MARGIN) && (target_boost < ((uint16_t)lut[i + 1U] - TXPOWER_ADJUSTMENT_MARGIN)))
    {
        *applied_boost = 0U;
        *adj_tx_power = ref_tx_power;
        return (int32_t)DWT_SUCCESS;
    }

    // Increase coarse setting if the required boost is greater than the TxPower gain using the increased coarse setting.
    // Recommendation DW3000 : limit usage of coarse = 0x3 because the ratio TxPower Gain/Power consumption is low.
    // Coarse gain = 0x3 should not be used in CHAN9

    while(ref_coarse_gain < 0x2U)
    {
        if(lut_coarse_gain[ref_coarse_gain] < (target_boost - current_boost))
        {
            current_boost += (uint16_t)lut_coarse_gain[ref_coarse_gain];
            ref_coarse_gain++;
        }
        else // Boost is less than the gain when increasing coarse setting
        {
            break;
        }
    }

    while(current_boost != target_boost)   // Until the applied boost reach the target value
    {
        // Security to ensure loop does not get locked.
        unlock++;
        if(unlock > (2U * LUT_COMP_SIZE))
        {
            *applied_boost = 0U;
            *adj_tx_power = ref_tx_power; // Loop got locked and could not find ajusted TxPower solution
            return (int32_t)DWT_ERROR;
        }

        // If current_boost within the margin, keep incrementing to potentially reach ideal value but keep the closest value as best solution
        if((current_boost > lower_limit) && (current_boost < upper_limit))
        {
            if((uint16_t)abs(((int32_t)target_boost - (int32_t)current_boost)) <= best_boost_abs)
            {
                best_boost_abs = (uint16_t)abs(((int32_t)target_boost - (int32_t)current_boost));
                best_boost = current_boost;
                best_index = i;
                best_coarse_gain = ref_coarse_gain;
                within_margin_flag = 1U;
            }
            else if(within_margin_flag != 0U) // Found the closest fine setting within margin to reach target_power.
            {
                i = best_index;
                ref_coarse_gain = best_coarse_gain;
                current_boost = best_boost;
                break;
            }
            else
            {
                /* Nothing to do */
            }
        }
        else if (within_margin_flag != 0U) // Previously found a solution within MARGIN and current setting is outside margin. We can stop
        {
            current_boost -= lut[i];
            i = best_index;
            break;
        }
        else
        {
            /* Nothing to do */
        }

        // If current boost is already larger than target_boost but not within margin, we return current solution
        // Increasing further fine gain will not bring a better solution.
        // This is a corner case likely to happen when fine gain setting is very low.
        if((current_boost >= upper_limit) && (reached_max_fine_gain_flag == 0U))
        {
            break;
        }

        // Reached maximum fine gain value
        if(i == (LUT_COMP_SIZE - 1U))
        {
            reached_max_fine_gain_flag = 1U; // Cannot increment i / fine_gain further

            // Previously found solution: returning solution and stopping here
            if(within_margin_flag != 0U)
            {
                i = best_index;
                ref_coarse_gain = best_coarse_gain;
                current_boost = best_boost;
                break;
            }

            if((ref_coarse_gain == 0x3U) || ((ref_coarse_gain == 0x2U) && (channel == (uint8_t) DWT_CH9)))
            {
                break;  // Reached the max power for CHAN5/CHAN9
            }

            if((current_boost + (uint16_t)lut_coarse_gain[ref_coarse_gain]) <= target_boost) // Increasing coarse gain is not sufficient to be within margin
            {
                current_boost += (uint16_t)lut_coarse_gain[ref_coarse_gain];
                ref_coarse_gain++;
                break; // Reached the max power for CHAN5
            }
            else // Reached the max fine gain value for coarse_gain-1. Increasing coarse gain and decreasing fine gain until within margin.
            {
               current_boost += (uint16_t)lut_coarse_gain[ref_coarse_gain];
               ref_coarse_gain++;
            }
        }

        if(reached_max_fine_gain_flag == 0U) // If not reached max fine gain, then we move to next one
        {
            i++;
            i &= 0x3FU; // Masking mask value to 63 as a protection
            current_boost += lut[i];
        }
        else // Reverting back after reaching the max fine gain.
        {
            current_boost -= lut[i];
            i--;
            i &= 0x3FU; // Masking mask value to 63 as a protection

            if(i == 0U)
            {
                reached_max_fine_gain_flag = 0U; // Cannot decrement i further
            }
        }
    }

    *applied_boost = current_boost;

    // Calculating the TxPower corresponding to target boost
    tx_power_byte = (i << 2U) | ref_coarse_gain;
    *adj_tx_power = (((uint32_t)tx_power_byte << 24UL) | ((uint32_t)tx_power_byte << 16U) |
                      ((uint32_t) tx_power_byte << 8U) | ((uint32_t)tx_power_byte));
    return (int32_t)DWT_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function reads the CIA version in this device, it needs to use indirect register read to achieve this.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return None
 */
static uint32_t ull_readCIAversion(dwchip_t* dw)
{
    uint32_t indirect_pointer_a = 0UL;
    uint32_t write_buff = (CIA_VERSION_REG >> 16UL);

    /* Program the indirect register access - using indirect pointer A */
    ull_writetodevice(dw, INDIRECT_ADDR_A_ID, 0U, 4U, (uint8_t*)&write_buff);

    write_buff = CIA_VERSION_REG & 0xFFFFUL;
    ull_writetodevice(dw, ADDR_OFFSET_A_ID, 0U, 4U, (uint8_t*)&write_buff);

    /* Indirectly read data pointed to by the indirect pointer A */
    ull_readfromdevice(dw, INDIRECT_POINTER_A_ID, 0U, 4U, (uint8_t*)&indirect_pointer_a);
    return indirect_pointer_a;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will read Ipatov, STS1 and STS2 diagnostics registers from the device, which can help in
 *        determining if packet has been received in LOS (line-of-sight) or NLOS (non-line-of-sight) condition.
 *        To help determine/estimate NLOS condition either Ipatov, STS1 or STS2 can be used, (or all three).
 *
 * @note  CIA diagnostics need to be enabled with @ref DW_CIA_DIAG_LOG_ALL, otherwise the diagnostic registers read will be 0,
 *        please see dwt_configciadiag().
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] all_diag: This is the pointer to the dwt_nlos_alldiag_t structure into which to read the data.
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error.
 */

static int ull_nlos_alldiag(dwchip_t *dw, dwt_nlos_alldiag_t *all_diag)
{
    if (all_diag->diag_type == IPATOV)
    {
        all_diag->accumCount = dwt_read32bitoffsetreg(dw, IP_DIAG_12_ID, 0U) & IP_DIAG_12_IPNACC_BIT_MASK;
        all_diag->F1 = dwt_read32bitoffsetreg(dw, IP_DIAG_2_ID, 0U) & IP_DIAG_2_IPF1_BIT_MASK;
        all_diag->F2 = dwt_read32bitoffsetreg(dw, IP_DIAG_3_ID, 0U) & IP_DIAG_3_IPF2_BIT_MASK;
        all_diag->F3 = dwt_read32bitoffsetreg(dw, IP_DIAG_4_ID, 0U) & IP_DIAG_4_IPF3_BIT_MASK;
        all_diag->cir_power = dwt_read32bitoffsetreg(dw, IP_DIAG_1_ID, 0U) & IP_DIAG_1_IPCHANNELAREA_BIT_MASK;
    }
    else if (all_diag->diag_type == STS1)
    {
        all_diag->accumCount = dwt_read32bitoffsetreg(dw, STS_DIAG_12_ID, 0U) & STS_DIAG_12_CYNACC_BIT_MASK;
        all_diag->F1 = dwt_read32bitoffsetreg(dw, STS_DIAG_2_ID, 0U) & STS_DIAG_2_CY0F1_BIT_MASK;
        all_diag->F2 = dwt_read32bitoffsetreg(dw, STS_DIAG_3_ID, 0U) & STS_DIAG_3_CY0F2_BIT_MASK;
        all_diag->F3 = dwt_read32bitoffsetreg(dw, STS_DIAG_4_ID, 0U) & STS_DIAG_4_CY0F3_BIT_MASK;
        all_diag->cir_power = dwt_read32bitoffsetreg(dw, STS_DIAG_1_ID, 0U) & STS_DIAG_1_CY0CHANNELAREA_BIT_MASK;
    }
    else if (all_diag->diag_type == STS2)
    {
        all_diag->accumCount = dwt_read32bitoffsetreg(dw, STS1_DIAG_12_ID, 0U) & STS1_DIAG_12_CY1NACC_BIT_MASK;
        all_diag->F1 = dwt_read32bitoffsetreg(dw, STS1_DIAG_2_ID, 0U) & STS1_DIAG_2_CY1F1_BIT_MASK;
        all_diag->F2 = dwt_read32bitoffsetreg(dw, STS1_DIAG_3_ID, 0U) & STS1_DIAG_3_CY1F2_BIT_MASK;
        all_diag->F3 = dwt_read32bitoffsetreg(dw, STS1_DIAG_4_ID, 0U) & STS1_DIAG_4_CY1F3_BIT_MASK;
        all_diag->cir_power = dwt_read32bitoffsetreg(dw, STS1_DIAG_1_ID, 0U) & STS1_DIAG_1_CY1CHANNELAREA_BIT_MASK;
    }
    else
    {
        return (int)DWT_ERROR;
    }

    all_diag->D = ull_get_dgcdecision(dw);

    return (int)DWT_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will read the Ipatov diagnostic registers to get the first path (FP) and peak path (PP) index value.
 *        This function is used when signal power is low to determine the signal type (LOS or NLOS). Hence only
 *        Ipatov diagnostic registers are used to determine the signal type.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[out] index: This is the pointer to the dwt_nlos_ipdiag_t structure into which to read the data.
 *
 * @return None
 */
static void ull_nlos_ipdiag(dwchip_t *dw, dwt_nlos_ipdiag_t *index)
{
    index->index_fp_u32 = dwt_read32bitoffsetreg(dw, IP_DIAG_8_ID, 0U) & IP_DIAG_8_IPFPLOC_BIT_MASK;  // 0x0c:48:bits15-0 - preamble diagnostic 8: IP_FP
    index->index_pp_u32 = (dwt_read32bitoffsetreg(dw, IP_DIAG_0_ID, 0U) & IP_DIAG_0_PEAKLOC_BIT_MASK) >> 21UL;       // 0x0c:28:bits30-21 - preamble diagnostic 0: IP_PEAKI
    index->index_pp_u32 <<= 6; // shift left by 6 digits so it compares with first path index, to avoid using double/float
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief For optimal channel 5 performance the PLL LDO tune should be increased.
 *
 * @param dw: DW3000 chip descriptor handler.
 *
 * @return  None
 */
static void ull_increase_ch5_pll_ldo_tune(dwchip_t *dw)
{
    uint8_t ldo_tune_pll = dwt_read8bitoffsetreg(dw, LDO_TUNE_LO_ID, 2U) & 0x0FU;
    ldo_tune_pll += 3U;
    if (ldo_tune_pll > 0x0FU)
    {
        ldo_tune_pll = 0x0FU;
    }

    dwt_and_or8bitoffsetreg(dw, LDO_TUNE_LO_ID, 2U, 0xF0U, ldo_tune_pll);
}

/*!
 * @brief This function will run the automotive PLL calibration for channel 5 or 9.
 *
 * @note Make sure the PLL is not locked before calling this function.
 * 
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param ch - Channel number (5 or 9)
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERR_PLL_LOCK for error
*/
static int32_t ull_run_auto_pll_cal(dwchip_t *dw, uint8_t ch)
{
    int32_t ret_val = (int32_t)DWT_SUCCESS;
    uint8_t steps_to_lock = 0;

#if DWT_DEBUG_PRINT
    uint32_t coarse = 0UL;
    printf("AUTO PLL calibration for channel %d\n", ch);
#endif
    if (ch == (uint8_t)DWT_CH9)
    {
        ret_val = ull_pll_ch9_auto_cal(dw, LOCAL_DATA(dw)->coarse_code_pll_cal_ch9, 0U, AUTO_PLL_CAL_STEPS, &steps_to_lock );
    }
    else //(ch == 5)
    {
        ret_val = ull_pll_ch5_auto_cal(dw, LOCAL_DATA(dw)->coarse_code_pll_cal_ch5, 0U, AUTO_PLL_CAL_STEPS, &steps_to_lock, LOCAL_DATA(dw)->temperature);
    }

#if DWT_DEBUG_PRINT
    if(ret_val == (int32_t)DWT_SUCCESS)
    {

        coarse = ch == (uint8_t)DWT_CH9 ? LOCAL_DATA(dw)->coarse_code_pll_cal_ch9 : LOCAL_DATA(dw)->coarse_code_pll_cal_ch5;
        printf("AUTO PLL locked after %d steps with coarse code 0x%08X\n", steps_to_lock, coarse);
    }
    else
    {
        /**
         * @note Getting here means that the automotive PLL calibration has failed.
         * It is likely due to an invalid coarse code value stored in OTP
         * In such condition the driver falls back to the default PLL calibration.
        */
        coarse = ch == (uint8_t)DWT_CH9 ? LOCAL_DATA(dw)->coarse_code_pll_cal_ch9 : LOCAL_DATA(dw)->coarse_code_pll_cal_ch5;
        printf("ERROR AUTO PLL failed to lock with OTP coarse = 0x%08x\n" , coarse);
    }
#endif

    return ret_val;
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @brief This function will run the hardware PLL calibration for channel 5 or 9.
 *
 * @note Make sure the PLL is not locked before calling this function.
 * 
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param ch - Channel number (5 or 9)
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERR_PLL_LOCK for error
*/
static int32_t ull_run_hardware_pll_cal(dwchip_t *dw, uint8_t ch)
{
    int32_t ret_val = (int32_t)DWT_SUCCESS;
    uint8_t ldo_tune_pll = 0;

    // Setup PLL calibration values for first calibration attempt
    if (ch == (uint8_t) DWT_CH9)
    {
        // Setup TX analog for ch9
        dwt_write32bitoffsetreg(dw, TX_CTRL_HI_ID, 0U, RF_TXCTRL_CH9);
        dwt_write16bitoffsetreg(dw, PLL_CFG_ID, 0U, RF_PLL_CFG_CH9);

        if (LOCAL_DATA(dw)->channel != ch)
        {
            // Restore original LDO tune value when changing from CH5 to CH9
            ldo_tune_pll = (uint8_t)(LOCAL_DATA(dw)->otp_ldo_tune_lo >> 16U);
            dwt_and_or8bitoffsetreg(dw, LDO_TUNE_LO_ID, 2U, 0xF0U, ldo_tune_pll);
        }
    }
    else
    {
        // Setup TX analog for ch5
        dwt_write32bitoffsetreg(dw, TX_CTRL_HI_ID, 0U, RF_TXCTRL_CH5);
        dwt_write16bitoffsetreg(dw, PLL_CFG_ID, 0U, RF_PLL_CFG_CH5);

        if (LOCAL_DATA(dw)->channel != ch)
        {
            ull_increase_ch5_pll_ldo_tune(dw);
        }
    }

    dwt_write8bitoffsetreg(dw, LDO_RLOAD_ID, 1U, LDO_RLOAD_VAL_B1);
    dwt_write8bitoffsetreg(dw, TX_CTRL_LO_ID, 2U, RF_TXCTRL_LO_B2);
    dwt_write8bitoffsetreg(dw, PLL_CAL_ID, 0U, RF_PLL_CFG_LD); // Extend the lock delay
    
    for (int cal_run = 0; cal_run < MAX_PLL_CAL_LOOP; cal_run++)
    {            
        // Calibrate the PLL and change to IDLE_PLL state
        ret_val = ull_setdwstate(dw, (int32_t)DWT_DW_IDLE);
        if(ret_val == (int32_t)DWT_SUCCESS)
        {
            break;
        }
        else
        {
            // If PLL calibration failed, switch back to IDLE_RC before retrying to calibrate the PLL
            (void)ull_setdwstate(dw, (int32_t)DWT_DW_IDLE_RC);
        }
        
        // If calibration didn't work first time try again but different PLL config settings
        if (ch == (uint8_t) DWT_CH9)
        {
            // Setup TX analog for ch9
            dwt_write16bitoffsetreg(dw, PLL_CFG_ID, 0U, RF_PLL_CFG_CH9_2);
        }
        else
        {
            // Setup TX analog for ch5
            dwt_write16bitoffsetreg(dw, PLL_CFG_ID, 0U, RF_PLL_CFG_CH5_2);
        }

        dwt_and_or32bitoffsetreg(dw, PLL_CAL_ID, 0U, 0xFFFFFFFFUL, PLL_CAL_PLL_WD_EN_BIT_MASK); //set watchdog flag
    }

    return ret_val;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will configure the channel number.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param ch - Channel number (5 or 9)
 *
 * @return @ref DWT_SUCCESS for success, or @ref DWT_ERR_PLL_LOCK for error
 */
static int32_t ull_setchannel(dwchip_t *dw, uint8_t ch)
{
    int32_t ret_val = (int32_t)DWT_SUCCESS;
    uint8_t dw_state = dwt_read8bitoffsetreg(dw, SYS_STATE_LO_ID, 2U);

    // Check if UWB radio is in TX or RX state, if true return an error. User should call dwt_forcetrxoff() prior to changing channel
    if (dw_state > DW_SYS_STATE_IDLE) 
    {
        return (int32_t) DWT_ERR_WRONG_STATE;
    }

    if ((LOCAL_DATA(dw)->channel != ch) || (dw_state != DW_SYS_STATE_IDLE))
    {
        if (dw_state == DW_SYS_STATE_IDLE) // If in IDLE_PLL then need to switch to IDLE_RC prior to re-calibrating the PLL for different channel
        {
            (void)ull_setdwstate(dw, (int32_t)DWT_DW_IDLE_RC);
        }

        // Prepare the channel register
        uint8_t chan_ctrl_reg = dwt_read8bitoffsetreg(dw, CHAN_CTRL_ID, 0U);
        chan_ctrl_reg &= ~((uint8_t)CHAN_CTRL_RF_CHAN_BIT_MASK);
        if(ch == (uint8_t) DWT_CH9)
        {
            /*
             * Change CHAN field to 1 for channel 9 in CHAN_CTRL_ID
             * If ch is == 5 leave CHAN field = 0
            */
            chan_ctrl_reg |= (uint8_t)CHAN_CTRL_RF_CHAN_BIT_MASK;
        }
        dwt_write8bitoffsetreg(dw, CHAN_CTRL_ID, 0U, chan_ctrl_reg);

#ifdef AUTO_DW3300Q_DRIVER
        ret_val = ull_run_auto_pll_cal(dw, ch);
        if(ret_val != (int32_t)DWT_SUCCESS)
        {
            /* Switch back to IDLE_RC before retrying to calibrate the PLL */
            (void)ull_setdwstate(dw, (int32_t)DWT_DW_IDLE_RC);

            ret_val = ull_run_hardware_pll_cal(dw, ch);
        }
#else
        ret_val = ull_run_hardware_pll_cal(dw, ch);
        if(ret_val != (int32_t)DWT_SUCCESS)
        {
            /* Switch back to IDLE_RC before retrying to calibrate the PLL */
            (void)ull_setdwstate(dw, (int32_t)DWT_DW_IDLE_RC);

            ret_val = ull_run_auto_pll_cal(dw, ch);
        }
#endif

        if (ret_val != (int32_t)DWT_SUCCESS)
        {
            ch = 0U;
            ret_val = (int32_t)DWT_ERR_PLL_LOCK;
        }
        LOCAL_DATA(dw)->channel = ch;
    }
    return ret_val;
}

/* @brief This function disables integrated power supply (IPS) and needs to be called prior to
 *        the device going to sleep or when an OTP low power mode is required.
 *
 * @note Synopsys have released an errata against the OTP power macro, in relation to low power
 *        SLEEP/RETENTION modes where the VDD and VCC are removed.
 *        On exit from low-power mode this is called to restore/enable the OTP IPS for normal OTP use (RD/WR).
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] mode: If set to 1 this will configure OTP for low-power
 *
 * @return  None
 */
static void ull_dis_otp_ips(dwchip_t *dw, int32_t mode)
{
    // set low power mode
    if (mode == 1)
    {
        // configure mode for programming
        dwt_write16bitoffsetreg(dw, OTP_CFG_ID, 0U, (0x10U | OTP_CFG_OTP_WRITE_MR_BIT_MASK));

        dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, (0x0200U | 0x2U));  // Address phase, pulse clock
        dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, (0x0000U | 0x2U));  // Address phase end, pulse clock

        dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, (0x0300U | 0xF3U)); // Instr phase
        dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, (0x0100U | 0xF3U)); // end Instr phase

        dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, (0x0300U | 0x1U));  // Write data phase
        dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, (0x0100U | 0x1U));  // Write data end phase

        dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, (0x0200U | 0x0U));
    }
    else
    {
        // Enable the OTP IPS - set it back to normal operation mode with MRR[0]=0x1 and MRR[24]=0x0 so the RQ data byte is 0x04
        dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, (0x0200U | 0x2U));  // Address phase, pulse clock
        dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, (0x0000U | 0x2U));  // Address phase end, pulse clock

        dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, (0x0300U | 0xF3U)); // Instr phase
        dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, (0x0100U | 0xF3U)); // end Instr phase

        dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, (0x0300U | 0x4U));  // Write data phase
        dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, (0x0100U | 0x4U));  // Write data end phase

        dwt_write16bitoffsetreg(dw, OTP_WDATA_ID, 0U, (0x0200U | 0x0U));
    }

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function runs the auto PLL calibration for channel 9.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] coarse_code: VCO coarse tune code
 * @param[in] sleep_us: Delay to wait in microseconds between each steps
 * @param[in] steps: The max number of steps over which to run PLL cal
 * @param[out] p_num_steps_lock: At return, number of steps required to get PLL locked
 *
 * @return DWT_SUCCESS (i.e PLL lock OK) or DWT_ERR_PLL_LOCK
 */
static int32_t ull_pll_ch9_auto_cal(dwchip_t *dw, uint32_t coarse_code, uint16_t sleep_us,
    uint8_t steps, uint8_t *p_num_steps_lock )
{
    int8_t increment = 0;
    uint32_t coarse_tmp;
    int32_t coarse_tuned = (int32_t)coarse_code;
    uint32_t lock_delay_setting = 0x000000FCUL; // controls lower byte of PLL_CAL[5 MSBs set lock delay]
    uint8_t pll_status, rf_status, pll_status_mask, rf_status_mask,  high_vth, mid_vth;
    int ret_val = (int)DWT_ERR_PLL_LOCK;
    uint32_t mask_temp = 0;

    dwt_write32bitoffsetreg(dw, LDO_CTRL_ID, 0x0U, (LDO_CTRL_LDO_VDDPLL_VREF_BIT_MASK | LDO_CTRL_LDO_VDDVCO_VREF_BIT_MASK | LDO_CTRL_LDO_VDDMS2_VREF_BIT_MASK | LDO_CTRL_LDO_VDDPLL_EN_BIT_MASK | LDO_CTRL_LDO_VDDVCO_EN_BIT_MASK | LDO_CTRL_LDO_VDDMS2_EN_BIT_MASK)); //overrides LDO enables

    // Channel 9 settings
    dwt_and_or32bitoffsetreg(dw, RF_CTRL_MASK_ID, 0U, ~(uint32_t)RF_EN_CH5, RF_EN_CH9);

    dwt_or8bitoffsetreg(dw, CHAN_CTRL_ID, 0U, (uint8_t)CHAN_CTRL_RF_CHAN_BIT_MASK); // Sets RF_CHAN to Channel 9

    if ((dwt_read8bitoffsetreg(dw, SYS_STATE_LO_ID, 2U) == DW_SYS_STATE_IDLE))
    {
        // Force sys clk to FOSC
        uint8_t clk_temp = dwt_read8bitoffsetreg(dw, CLK_CTRL_ID, 0x0U);
        dwt_write8bitoffsetreg(dw, CLK_CTRL_ID, 0x0U, clk_temp | (uint8_t)CLK_CTRL_SYS_CLK_SEL_BIT_MASK);

        // disable auto init to idle, enable force 2 init for TSE
        mask_temp = ~(uint32_t)(SEQ_CTRL_FORCE2INIT_BIT_MASK | SEQ_CTRL_AINIT2IDLE_BIT_MASK);
        dwt_and_or32bitoffsetreg(dw, SEQ_CTRL_ID, 0x0U, mask_temp, SEQ_CTRL_FORCE2INIT_BIT_MASK);
        // clear the force bit
        dwt_and32bitoffsetreg(dw, SEQ_CTRL_ID, 0U, ~(uint32_t)SEQ_CTRL_FORCE2INIT_BIT_MASK);
        // set CLK_CTRL back to default
        dwt_write8bitoffsetreg(dw, CLK_CTRL_ID, 0x0U, clk_temp);
    }

    // TX buffer tune; same as f00
    dwt_write32bitoffsetreg(dw, TX_CTRL_HI_ID, 0x0U, RF_TXCTRL_CH9);
    // Use phase based lock detect 0F3C, counter based 1F3C (this is CH5 setting)
    dwt_write32bitoffsetreg(dw, PLL_CFG_ID, 0x0U, RF_PLL_CFG_CH5);

    // Set TX LDO RLoad;
    dwt_write8bitoffsetreg(dw, LDO_RLOAD_ID, 1U, LDO_RLOAD_VAL_B1);

    // TX buffer bias ctrl; same as f00
    mask_temp = ~(uint32_t)(TX_CTRL_LO_TX_LOBUF_CTRL_BIT_MASK | TX_CTRL_LO_TX_VBULK_CTRL_BIT_MASK | TX_CTRL_LO_TX_VCASC_CTRL_BIT_MASK);
    dwt_and_or32bitoffsetreg(dw, TX_CTRL_LO_ID, 0U, mask_temp, (TX_CTRL_LO_TX_VBULK_CTRL_BIT_MASK | (0x2UL << TX_CTRL_LO_TX_VCASC_CTRL_BIT_OFFSET)));

    // setup PLL CAL -- turn on watchdog
    mask_temp = ~(uint32_t)(PLL_CAL_PLL_CAL_EN_BIT_MASK | PLL_CAL_PLL_WD_EN_BIT_MASK | PLL_LOCK_DLY_BIT_MASK | PLL_CAL_PLL_TUNE_OVR_BIT_MASK | 
    PLL_CAL_PLL_USE_OLD_BIT_MASK | PLL_CH9_FB_OVR_BIT_MASK);
    dwt_and_or32bitoffsetreg(dw, PLL_CAL_ID, 0U, mask_temp, 
    (PLL_CAL_PLL_WD_EN_BIT_MASK | lock_delay_setting));

    /* Turn ON CH9 prebufs and CAL */
    // Enable the PLL TX prebuffers, when the PLL is active
    dwt_or32bitoffsetreg(dw, RF_ENABLE_ID, 0U, RF_ENABLE_PLL_TX_PRE_EN_BIT_MASK);

    // Set the VCO coarse tune value to use if override is in place
    coarse_tmp = (coarse_code & (1U << (PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_OFFSET + PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_LEN)));
    coarse_tmp <<= PLL_COARSE_CODE_CH9_RVCO_FREQ_BOOST_BIT_OFFSET - (PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_OFFSET + PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_LEN); 
    coarse_tmp += (coarse_code & PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_MASK) << PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_OFFSET;

    dwt_and_or32bitoffsetreg(dw, PLL_COARSE_CODE_ID, 0U, PLL_COARSE_CODE_CH5_VCO_COARSE_TUNE_BIT_MASK, (coarse_tmp << PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_OFFSET));

    // this code will enable watchdog clock [also gates normal WD enable]
    dwt_or32bitoffsetreg(dw, PLL_COMMON_ID, 0x0U, PLL_COMMON_DIG_PLL_WD_SEL_REF_CLK_DIVBY16_ULV_MASK);

    // this code will override PLL controls
    // now enable TX prefbufsand set LF to 1; turn on PLLand PLL CH9and set resetn to 1
    dwt_write32bitoffsetreg(dw, RF_ENABLE_ID, 0x0U, RF_EN_CH9);

    deca_usleep(sleep_us);

    /*
        When PLL is locked, because we are not using HW calbration:
         - RF status mask should be 0x0B for CH9
         - PLL status mask should be 0x46
    */
    rf_status_mask = (uint8_t)(RF_STATUS_PLL1_MID_FLAG_BIT_MASK | RF_STATUS_PLL1_LO_FLAG_BIT_MASK | RF_STATUS_PLL1_LOCK_BIT_MASK);
    pll_status_mask = (uint8_t)(PLL_STATUS_XTAL_AMP_SETTLED_BIT_MASK | PLL_STATUS_PLL_LO_FLAG_N_BIT_MASK | PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK);

    for (uint8_t i = 0U; i < steps; i++)
    {
        pll_status = dwt_read8bitoffsetreg(dw, PLL_STATUS_ID, 0x0U);
        rf_status = dwt_read8bitoffsetreg(dw, RF_STATUS_ID, 0x0U);
        
        if(((rf_status & rf_status_mask) == rf_status_mask) && 
        ((pll_status & pll_status_mask) == pll_status_mask))
        {
            dwt_or16bitoffsetreg(dw, CLK_CTRL_ID, 0U,  FORCE_SYSCLK_PLL); // Force use PLL clock
            dwt_and_or32bitoffsetreg(dw, SEQ_CTRL_ID, 0U, ~(uint32_t)SEQ_CTRL_FORCE2INIT_BIT_MASK, SEQ_CTRL_FORCE2IDLE_BIT_MASK);
            dwt_and32bitoffsetreg(dw, SEQ_CTRL_ID, 0U, ~(uint32_t)SEQ_CTRL_FORCE2IDLE_BIT_MASK);
            dwt_write32bitoffsetreg(dw, RF_CTRL_MASK_ID, 0U, 0UL);
            dwt_write32bitoffsetreg(dw, RF_ENABLE_ID, 0U, 0UL);
            *p_num_steps_lock = i;
            ret_val = (int)DWT_SUCCESS;
            LOCAL_DATA(dw)->coarse_code_pll_cal_ch9 = coarse_tmp;
            break;
        }
        else
        {
            high_vth = (rf_status & RF_STATUS_PLL1_HI_FLAG_BIT_MASK) >> 2U; // shift to get either 1 or 0
            mid_vth  = (rf_status & RF_STATUS_PLL1_MID_FLAG_BIT_MASK) >> 3U; // shift to get either 1 or 0
            if (high_vth == 1U)
            {
                increment = -1;
            }
            else if(mid_vth == 0U)
            {
                increment = 1;
            }
            else
            {
                increment = 0;
            }
        }
        coarse_tuned += increment; //turn on cal w/ prebufs & write coarse code start into reg
        coarse_tmp = ((uint32_t)coarse_tuned & (1U << (PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_OFFSET + PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_LEN)));
        coarse_tmp <<= PLL_COARSE_CODE_CH9_RVCO_FREQ_BOOST_BIT_OFFSET - (PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_OFFSET + PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_LEN); 
        coarse_tmp += ((uint32_t)coarse_tuned & PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_MASK) << PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_OFFSET;
        dwt_modify32bitoffsetreg(dw, PLL_COARSE_CODE_ID, 0x0U, ~(uint32_t)(PLL_COARSE_CODE_CH9_RVCO_FREQ_BOOST_BIT_MASK + PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_MASK), coarse_tmp);
        deca_usleep(sleep_us);
    }

    return ret_val;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function runs the auto PLL calibration for channel 5.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] coarse_code: VCO coarse tune code
 * @param[in] sleep_us: Delay to wait in microseconds between each steps
 * @param[in] steps: The max number of steps over which to run PLL cal
 * @param[out] p_num_steps_lock: At return, number of steps required to get PLL locked
 * @param[in] temperature  - device temperature, if TEMP_INIT is passed it will read temperature from the device
 *
 * @return @ref DWT_SUCCESS (i.e PLL lock OK) or @ref DWT_ERR_PLL_LOCK
 */
static int32_t ull_pll_ch5_auto_cal(dwchip_t *dw, uint32_t coarse_code, uint16_t sleep_us, uint8_t steps, uint8_t *p_num_steps_lock, int8_t temperature)
{
    uint32_t ldo_tune_lo;
    int8_t temp2;
    uint32_t lock_delay_setting = 0x000000FCUL; // controls lower byte of PLL_CAL[5 MSBs set lock delay]
    uint8_t pll_status, rf_status, pll_status_mask, rf_status_mask, high_vth, lo_vth;
    uint16_t tempvbat;
    int32_t ret_val = (int32_t)DWT_ERR_PLL_LOCK;
    uint32_t mask_temp = 0;

    if (temperature == TEMP_INIT) // If set to TEMP_INIT use temperature sensor to read the temp
    {
        tempvbat = ull_readtempvbat(dw);
        temperature = (int8_t)ull_convertrawtemperature(dw, (uint8_t)(tempvbat >> 8U));  // Temperature in upper 8 bits
    }

    if (temperature > 95) // If hot - change the LDO_PLL tune
    {
        //Read LDO_TUNE and BIAS_TUNE from OTP
        ldo_tune_lo = LOCAL_DATA(dw)->otp_ldo_tune_lo;

        if (ldo_tune_lo != 0UL)
        {
            uint16_t lower_ldo_pll_tune = (uint16_t)((ldo_tune_lo & LDO_PLL_TUNE_BIT_MASK) >> LDO_PLL_TUNE_BIT_OFFSET) - 2U;
            temp2 = (int8_t)lower_ldo_pll_tune; //lower LDO_PLL_TUNE by 2 at hot
            if (temp2 < 0)
            {
                temp2 = 0;
            }
            ldo_tune_lo = (ldo_tune_lo & ~LDO_PLL_TUNE_BIT_MASK) | ((uint32_t)temp2 << LDO_PLL_TUNE_BIT_OFFSET);
            dwt_write32bitoffsetreg(dw, LDO_TUNE_LO_ID, 0U, ldo_tune_lo);
        }
    }

    dwt_write32bitoffsetreg(dw, LDO_CTRL_ID, 0x0U, (LDO_CTRL_LDO_VDDPLL_VREF_BIT_MASK | LDO_CTRL_LDO_VDDVCO_VREF_BIT_MASK | LDO_CTRL_LDO_VDDMS2_VREF_BIT_MASK | LDO_CTRL_LDO_VDDPLL_EN_BIT_MASK | LDO_CTRL_LDO_VDDVCO_EN_BIT_MASK | LDO_CTRL_LDO_VDDMS2_EN_BIT_MASK)); //overrides LDO enables

    dwt_and_or32bitoffsetreg(dw, RF_CTRL_MASK_ID, 0U, ~(uint32_t)RF_EN_CH9, RF_EN_CH5);// Channel 5 settings

    dwt_and_or8bitoffsetreg(dw, CHAN_CTRL_ID, 0U, ~((uint8_t)CHAN_CTRL_RF_CHAN_BIT_MASK), 0x0U); // Sets RF_CHAN to Channel 5 //LSB is 0

    if ((dwt_read8bitoffsetreg(dw, SYS_STATE_LO_ID, 2U) == DW_SYS_STATE_IDLE))
    {
        // Force sys clk to FOSC
        uint8_t clk_temp = dwt_read8bitoffsetreg(dw, CLK_CTRL_ID, 0x0U);
        dwt_write8bitoffsetreg(dw, CLK_CTRL_ID, 0x0U, (clk_temp | (uint8_t)CLK_CTRL_SYS_CLK_SEL_BIT_MASK));

        // disable auto init to idle, enable force 2 init for TSE
        mask_temp = ~(uint32_t)(SEQ_CTRL_FORCE2INIT_BIT_MASK | SEQ_CTRL_AINIT2IDLE_BIT_MASK);
        dwt_and_or32bitoffsetreg(dw, SEQ_CTRL_ID, 0x0U, mask_temp, SEQ_CTRL_FORCE2INIT_BIT_MASK);
        // clear the force bit
        dwt_and32bitoffsetreg(dw, SEQ_CTRL_ID, 0U, ~(uint32_t)SEQ_CTRL_FORCE2INIT_BIT_MASK);
        // set CLK_CTRL back to default
        dwt_write8bitoffsetreg(dw, CLK_CTRL_ID, 0x0U, clk_temp);
    }

    // TX buffer tune; same as f00
    dwt_write32bitoffsetreg(dw, TX_CTRL_HI_ID, 0x0U, RF_TXCTRL_CH5);
    // Use phase based lock detect 0F3C, counter based 1F3C (this is CH5 setting)
    dwt_write32bitoffsetreg(dw, PLL_CFG_ID, 0x0U, RF_PLL_CFG_CH5);

    // Set TX LDO RLoad;
    dwt_write8bitoffsetreg(dw, LDO_RLOAD_ID, 1U, LDO_RLOAD_VAL_B1);

    // TX buffer bias ctrl; same as f00
    // TX bias and voltage contol, TXLO BUffer bias
    mask_temp = ~(uint32_t)(TX_CTRL_LO_TX_LOBUF_CTRL_BIT_MASK | TX_CTRL_LO_TX_VBULK_CTRL_BIT_MASK | TX_CTRL_LO_TX_VCASC_CTRL_BIT_MASK);
    dwt_and_or32bitoffsetreg(dw, TX_CTRL_LO_ID, 0U, mask_temp, (TX_CTRL_LO_TX_VBULK_CTRL_BIT_MASK | (0x2UL << TX_CTRL_LO_TX_VCASC_CTRL_BIT_OFFSET)));

    // setup PLL CAL -- turn on watchdog
    mask_temp = ~(uint32_t)(PLL_CAL_PLL_CAL_EN_BIT_MASK | PLL_CAL_PLL_WD_EN_BIT_MASK | PLL_LOCK_DLY_BIT_MASK | PLL_CAL_PLL_TUNE_OVR_BIT_MASK | 
                            PLL_CAL_PLL_USE_OLD_BIT_MASK | PLL_CH9_FB_OVR_BIT_MASK);
    dwt_and_or32bitoffsetreg(dw, PLL_CAL_ID, 0U, mask_temp, (PLL_CAL_PLL_WD_EN_BIT_MASK | lock_delay_setting));

    /* Turn ON CH5 prebufs and CAL */
    // Enable the PLL TX prebuffers, when the PLL is active
    dwt_or32bitoffsetreg(dw, RF_ENABLE_ID, 0U, RF_ENABLE_PLL_TX_PRE_EN_BIT_MASK);

    // Set the VCO coarse tune value
    dwt_and_or32bitoffsetreg(dw, PLL_COARSE_CODE_ID, 0U, PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_MASK, (coarse_code << PLL_COARSE_CODE_CH5_VCO_COARSE_TUNE_BIT_OFFSET));

    // this code will enable watchdog clock [also gates normal WD enable]
    dwt_or32bitoffsetreg(dw, PLL_COMMON_ID, 0x0U, PLL_COMMON_DIG_PLL_WD_SEL_REF_CLK_DIVBY16_ULV_MASK);

    // this code will override PLL controls
    // now enable TX prefbufs and set LF to 1; turn on PLL and PLL CH9 and set resetn to 1
    dwt_write32bitoffsetreg(dw, RF_ENABLE_ID, 0x0U, RF_EN_CH5);

    deca_usleep(sleep_us);

    /*
        When PLL is locked, because we are not using HW calbration:
         - RF status mask should be 0x03 for CH5
         - PLL status mask should be 0x46
    */
    rf_status_mask = (uint8_t)(RF_STATUS_PLL1_LO_FLAG_BIT_MASK | RF_STATUS_PLL1_LOCK_BIT_MASK);
    pll_status_mask = (uint8_t)(PLL_STATUS_XTAL_AMP_SETTLED_BIT_MASK | PLL_STATUS_PLL_LO_FLAG_N_BIT_MASK | PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK);

    for (uint8_t i = 0U; i < steps; i++)
    {
        pll_status = dwt_read8bitoffsetreg(dw, PLL_STATUS_ID, 0x0U);
        rf_status = dwt_read8bitoffsetreg(dw, RF_STATUS_ID, 0x0U);

        if (((rf_status & rf_status_mask) == rf_status_mask) && 
            ((pll_status & pll_status_mask ) == pll_status_mask))
        {
            dwt_or16bitoffsetreg(dw, CLK_CTRL_ID, 0U, FORCE_SYSCLK_PLL); // Force use PLL clock
            dwt_and_or32bitoffsetreg(dw, SEQ_CTRL_ID, 0U, ~(uint32_t)SEQ_CTRL_FORCE2INIT_BIT_MASK, SEQ_CTRL_FORCE2IDLE_BIT_MASK);
            dwt_and32bitoffsetreg(dw, SEQ_CTRL_ID, 0U, ~(uint32_t)SEQ_CTRL_FORCE2IDLE_BIT_MASK);
            dwt_write32bitoffsetreg(dw, RF_CTRL_MASK_ID, 0U, 0UL);
            dwt_write32bitoffsetreg(dw, RF_ENABLE_ID, 0U, 0UL);
            *p_num_steps_lock = i;
            ret_val = (int32_t)DWT_SUCCESS;
            LOCAL_DATA(dw)->coarse_code_pll_cal_ch5 = coarse_code;
            break;
        }
        else
        {
            high_vth = (rf_status & RF_STATUS_PLL1_HI_FLAG_BIT_MASK) >> 2U; // shift to get either 1 or 0
            lo_vth  = (rf_status & RF_STATUS_PLL1_LO_FLAG_BIT_MASK) >> 1U; // shift to get either 1 or 0
            if (high_vth == 1U)
            {
                coarse_code = ((coarse_code + 1UL) >> 1UL) - 1UL;  // Themomometer decrement
            }
            else if(lo_vth == 0U)
            {
                coarse_code = ((coarse_code + 1UL) << 1UL) - 1UL;  // Themomometer increment
            }
            else
            {
                // Do nothing
            }
        }
        //turn on cal w/ prebufs & write coarse code start into reg
        dwt_modify32bitoffsetreg(dw, PLL_COARSE_CODE_ID, 0x0U, ~(uint32_t)PLL_COARSE_CODE_CH5_VCO_COARSE_TUNE_BIT_MASK, coarse_code << PLL_COARSE_CODE_CH5_VCO_COARSE_TUNE_BIT_OFFSET);
        deca_usleep(sleep_us);
    }
    return ret_val; // Shouldn't reach this code, but make the compiler happy!
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Static function to retrieve the Tx power look-up table corresponding to current configuration.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] channel: Transmit channel (9 or 5)
 * @param[in] bias: Bias trim configuration (7 or 1)
 * @param[out] p_lut: Pointer to the LUT corresponding to input configuration and to be returned
 *
 * @return @ref DWT_SUCCESS if corresponding LUT is found, @ref DWT_ERROR otherwise.
 */
static int32_t OPTSPEED ull_get_txp_lut(uint8_t channel, uint8_t bias, tx_adj_lut_t *p_lut)
{
    uint32_t cfg =  (((uint32_t)channel << 16UL) | (uint32_t)bias);
    p_lut->bias = bias;
    dwt_error_e retVal = DWT_SUCCESS;

    switch(cfg)
    {
        case 0x00090007UL:
            p_lut->lut = GET_TXP_LUT(9, 0, 7);
            p_lut->lut_size = GET_TXP_LUT_SIZE(9, 0, 7);
            p_lut->end_index = GET_MAX_INDEX_SOC(9, 0, 7);
            p_lut->start_index = GET_MIN_INDEX_SOC(9, 0, 7);
        break;

        case 0x00090001UL:
            p_lut->lut = GET_TXP_LUT(9, 0, 1);
            p_lut->lut_size = GET_TXP_LUT_SIZE(9, 0, 1);
            p_lut->end_index = GET_MAX_INDEX_SOC(9, 0, 1);
            p_lut->start_index = GET_MIN_INDEX_SOC(9, 0, 1);
        break;

        case 0x00050007UL:
            p_lut->lut = GET_TXP_LUT(5, 0, 7);
            p_lut->lut_size = GET_TXP_LUT_SIZE(5, 0, 7);
            p_lut->end_index = GET_MAX_INDEX_SOC(5, 0, 7);
            p_lut->start_index = GET_MIN_INDEX_SOC(5, 0, 7);
        break;

        case 0x00050001UL:
            p_lut->lut = GET_TXP_LUT(5, 0, 1);
            p_lut->lut_size = GET_TXP_LUT_SIZE(5, 0, 1);
            p_lut->end_index = GET_MAX_INDEX_SOC(5, 0, 1);
            p_lut->start_index = GET_MIN_INDEX_SOC(5, 0, 1);
        break;

        default:
            retVal = DWT_ERROR;
        break;
    }
    return (int32_t)retVal;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Static function to check if the input indexes for packet fits within the range of the reference LUT.
 *        If the index fits, then the reference LUT will be stored in the corresponding LUT pointers.
 *        This function will be called in iteration until LUTs are found.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] ref_lut: Reference LUT.
 * @param[in] fr_index: Frame index for which we want to check if index is within the range of the reference LUT.
 * @param[out] p_fr_lut: Pointer to frame LUT. This will be set to the reference LUT if the input frame index is within range of reference LUT.
 *
 * @return 1 if a solution was found, 0 otherwise.
 */
static uint8_t OPTSPEED ull_check_lut(tx_adj_lut_t ref_lut, uint8_t fr_index ,tx_adj_lut_t *p_fr_lut)
{
    // Static variable required to store current state until a solution is found
    static uint8_t found_fr_lut = 0U;
    static uint8_t last_offset = 0U;
    uint8_t adjusted_index;
    uint8_t ret_val = 0U;

    // Adjust index depending on pa_compensation if no NULL
    adjusted_index = ref_lut.end_index;

    if((fr_index <= adjusted_index) && (found_fr_lut == 0U))
    {
        *p_fr_lut = ref_lut;
        p_fr_lut->offset_index = last_offset;
        found_fr_lut = 1U;
    }
    last_offset = adjusted_index + 1U;

    // If solution was found set local variable to default value.
    if(found_fr_lut != 0U)
    {
        found_fr_lut = 0U;
        last_offset = 0U;
        ret_val = 1U;
    }
    return ret_val;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Static function to return the LUT that corresponds to the input power indexes.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] channel: Input power indexes for which power must be calculated.
 * @param[in] p_indexes: Input power indexes for which power must be calculated.
 *                    The API will return the LUT corresponding to the minimum index value required in input.
 * @param[out] p_txp_luts: One LUT is returned with this structure @ref tx_adj_lut_t,
 *                         the LUT to be used to calculate the packet tx configuration
 *
 * @return @ref DWT_SUCCESS if corresponding LUT is found, @ref DWT_ERROR otherwise.
 */
static int32_t OPTSPEED ull_find_best_lut(uint32_t channel, power_indexes_t *p_indexes, txp_lut_t *p_txp_lut)
{
    uint8_t min_frame_index;
    dwt_error_e ret_val = DWT_ERROR;

    tx_adj_lut_t ref_lut= {0};
    tx_adj_lut_t *frame_lut = &p_txp_lut->tx_frame_lut;

    // Configuration (TxSetting, PA, Bias) must remain the same for the whole duration of a frame
    // For frame power, LUT corresponding to the highest input index is selected.
    // By design, priority is given to highest output power.
    min_frame_index = p_indexes->input[DWT_DATA_INDEX];
    for(uint8_t i = (uint8_t)DWT_PHR_INDEX; i < (uint8_t)DWT_MAX_POWER_INDEX; i++)
    {
        min_frame_index = MIN(min_frame_index, p_indexes->input[i]);
    }

    // Implementation of SOC transistions
    // SOC TRANSITION1: PA0, BIAS7
    (void)ull_get_txp_lut((uint8_t)channel, 7U, &ref_lut);

    if(ull_check_lut(ref_lut, min_frame_index, frame_lut) != 0U)
    {
        ret_val = DWT_SUCCESS;
    }
    else
    {
        // SOC TRANSITION2: PA0, BIAS1
        (void)ull_get_txp_lut((uint8_t)channel, 1U, &ref_lut);
        min_frame_index = MIN(min_frame_index, ref_lut.end_index);

        if(ull_check_lut(ref_lut, min_frame_index, frame_lut) != 0U)
        {
            ret_val = DWT_SUCCESS;
        }
    }

    return (int32_t)ret_val; // not matching case
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API is used to calculate a transmit power configuration in a linear manner (step of 0.25 dB)
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] channel: The channel for which the linear Tx power setting must be calculated
 * @param[in] p_indexes: Pointer to an object (@ref power_indexes_t) containing two members:
 *                   - in : The inputs indexes for which Tx power configuration must be calculated.
 *                          This is an array of size 4 allowing to set individual indexes for each section of a packet as well.
 *
 *                          DWT_DATA_INDEX = 0
 *                          DWT_PHR_INDEX = 1
 *                          DWT_SHR_INDEX = 2
 *                          DWT_STS_INDEX = 3
 *
 *                          Index step is 0.25 dB. Power will decrease linearly by 0.25dB step with the index:
 *                          0 corresponds to maximum output power
 *                          Output power = Power(0) - 0.25dB * Index
 *
 *                          Effective maximum index value depends on UWB IC part and channel configuration. If the required index is higher than
 *                          the maximum supported value, then the API will apply the maximum index.
 *
 *                  - out: output Tx power indexes corresponding to the indexes that were actually applied.
 *                         These may differ from the input indexes in the two cases below:
 *                              1. The input indexes correspond to a different PLLCFG for different section of packet. This is not
 *                              supported by the IC. In such condition, all indexes will belong to the same tx configuration state. The state
 *                              to be used is the one corresponding to the highest required power (lower index)
 *
 *                              2. The input index are greater than the maximum value supported for the (channel, device) current
 *                              configuration. In which case, the maximum index supported is returned.
 * @param[out] p_res: Pointer to an object (@ref tx_adj_res_t) containing the output configuration corresponding to input index.
 *              The object contain two members:
 *              - tx_frame_cfg: the power configuration to use during packet transmission
 *
 *              A configuration is a combination of two parameters:
 *              - uint32_t tx_power_setting: Tx power setting to be written to register TX_POWER_ID
 *              - uint8_t pll_bias: PLL bias trim configuration to be written to register PLL_COMMON_ID
 *
 * @return
 * @ref DWT_SUCCESS if an adjusted Tx power setting could be calculated. In this case, the configuration to be applied is return through
 * p_res parameter.
 * @ref DWT_ERROR if the API could not calculate a valid configuration.
 */
static int32_t OPTSPEED ull_calculate_linear_tx_power(uint32_t channel, power_indexes_t *p_indexes, tx_adj_res_t *p_res)
{
    dwt_error_e err = DWT_SUCCESS;
    uint8_t offset = 0U;
    uint8_t start = 0U;
    uint8_t index = 0U;
    uint8_t lut_size = 0U;
    txp_lut_t luts = {0};
    uint32_t tx_power = 0UL;

    // Query best lookup table for inputs indexes.
    // For frame indexes, priority is given to output power. Hence is PA or high BIAS is required
    // for any index, it will be applied for all.
    if (ull_find_best_lut(channel, p_indexes, &luts) != (int32_t)DWT_SUCCESS)
    {
        err = DWT_ERROR;
    }
    else
    {
        // Calculate adjusted power for frame indexes
        offset = luts.tx_frame_lut.offset_index;
        start = luts.tx_frame_lut.start_index;
        lut_size = luts.tx_frame_lut.lut_size;

        for(uint8_t i = 0U; i < (uint8_t)DWT_MAX_POWER_INDEX; i++)
        {
            index = p_indexes->input[i] - offset + start;
            index = MIN(index, lut_size - 1U);
            tx_power = tx_power | (uint32_t)luts.tx_frame_lut.lut[index] << i*8U;
            p_indexes->output[i] = index + offset - start;
        }
        p_res->tx_frame_cfg.pll_bias = luts.tx_frame_lut.bias;
        p_res->tx_frame_cfg.tx_power_setting = tx_power;
    }

    return (int32_t)err;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @brief This API is used to convert a transmit power value into its corresponding Tx power index.
*
* @param[in] channel: The channel for which the Tx power index must be calculated
* @param[in] tx_power: The Transmit power to convert
* @param[out] tx_power_idx: Pointer to the returned Tx power index
*
* @return
* @ref DWT_SUCCESS if the conversion succeeded
* @ref DWT_ERROR if the API could not calculate a valid configuration.
*/
static int OPTSPEED ull_convert_tx_power_to_index(uint32_t channel, uint8_t tx_power, uint8_t *tx_power_idx)
{
    tx_adj_lut_t ref_lut={0};
    uint8_t tx_power_coarse = (tx_power & TX_POWER_COARSE_BIT_MASK) >> TX_POWER_COARSE_BIT_OFFSET;
    uint8_t tx_power_fine = (tx_power & TX_POWER_FINE_BIT_MASK) >> TX_POWER_FINE_BIT_OFFSET;
    uint8_t closest_fine_lower = 0U;
    uint8_t closest_fine_higher = 0xFFU;
    int8_t closest_idx_lower = -1;
    int8_t closest_idx_higher = -1;
    uint8_t diff_idx;
    uint8_t diff_fine;
    uint8_t offset_idx;

    /* TX Power has always been calibrated using PLL bias 7. */
    (void)ull_get_txp_lut((uint8_t)channel, 7U, &ref_lut);

    /* In the table dwt_txp_lut_p0_b7_cX corresponding to the channel, look for the closest value to
     * the tx_power input. That closest value is the one defined by:
     *  - Same coarse value.
     *  - Among all the values having the same coarse, look for the closest fine value.
     */
    for (uint8_t i = 0U; i < ref_lut.lut_size; i++)
    {
        uint8_t cur_tx_power = ref_lut.lut[i];
        uint8_t cur_coarse = (cur_tx_power & TX_POWER_COARSE_BIT_MASK) >> TX_POWER_COARSE_BIT_OFFSET;
        uint8_t cur_fine = (cur_tx_power & TX_POWER_FINE_BIT_MASK) >> TX_POWER_FINE_BIT_OFFSET;

        if (cur_tx_power == tx_power)
        {
            /* Exact same value, cannot find a closer one. So return immediately. */
            *tx_power_idx = i;
            return (int)DWT_SUCCESS;
        }

        /* Find the closest higher and lower indexes with same coarse value. */
        if (cur_coarse == tx_power_coarse)
        {
            if ((cur_fine > tx_power_fine) && (cur_fine < closest_fine_higher))
            {
                closest_fine_higher = cur_fine;
                closest_idx_higher = (int8_t)i;
            }
            else if ((cur_fine < tx_power_fine) && (cur_fine > closest_fine_lower))
            {
                closest_fine_lower = cur_fine;
                closest_idx_lower = (int8_t)i;
            }
            else
            {
                /* Nothing to do */
            }
        }
    }

    /* No value found with the same coarse. */
    if ((closest_idx_higher < 0) && (closest_idx_lower < 0))
    {
        return (int)DWT_ERROR;
    }

    /* Compute the index depending on lower and higher value. */
    diff_idx = (uint8_t)closest_idx_lower - (uint8_t)closest_idx_higher; /* idx lower > idx higher. */
    diff_fine = closest_fine_higher - closest_fine_lower;
    offset_idx = ((tx_power_fine - closest_fine_lower) * diff_idx) / diff_fine;
    *tx_power_idx = (uint8_t)closest_idx_lower - offset_idx;
    return (int)DWT_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will set the PLL bias trim value in PLL_COMMON register. This is useful
 * for applying the adjusted Tx power configuration returned by dwt_calculate_linear_tx_power().
 *
 * @note If the RX is required, to provide optimal performance, the PLL bias trim should be set to the default value.
 * The @ref dwt_rxenable API and @ref dwt_isr will do this automatically. 
 * If an external ISR is used and the RX is required after a TX, the application should set the PLL bias trim to the default 
 * value manually.
 * Use @ref DWT_DEF_PLLBIASTRIM to restore the default value.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] pll_bias_trim: PLL bias trim.
 *
 * @return None.
 */
static void ull_setpllbiastrim(struct dwchip_s *dw, uint8_t pll_bias_trim)
{
    if(LOCAL_DATA(dw)->pll_bias_trim != pll_bias_trim)
    {
        dwt_and_or8bitoffsetreg(dw, PLL_COMMON_ID, 1U, (uint8_t)(~PLL_COMMON_PLL_BIAS_TRIM_MASK >> 8U), pll_bias_trim << 5U);
        LOCAL_DATA(dw)->pll_bias_trim = pll_bias_trim;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Static function to update the DGC settings to match the specified device config.
 *
 * This function should be called when RX code is reconfigured.
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 * @param[in] channel  Channel as in dwt_configure()
 *
 * @return None
 */
static void ull_update_dgc_config(struct dwchip_s *dw, uint32_t channel)
{
    /* Always enable the DGC to improve RX performance.*/
    /* If the OTP has DGC configurations programmed into it, load from OTP. */
    if (LOCAL_DATA(dw)->dgc_otp_set == DWT_DGC_LOAD_FROM_OTP)
    {
        dwt_kick_dgc_on_wakeup(dw, (int8_t)channel);
    }
    /* Else we manually program hard-coded values into the DGC registers. */
    else
    {
        // Load RX LUTs
        ull_configmrxlut(dw, (int32_t)channel);
    }

    dwt_modify16bitoffsetreg(dw, DGC_CFG_ID, 0x0U, (uint16_t)~DGC_CFG_THR_64_BIT_MASK, (uint16_t)DWT_DGC_CFG << DGC_CFG_THR_64_BIT_OFFSET);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this a chip-specific implementation of ioctl()
 *
 * @param[in] dw: DW3000 chip descriptor handler.
 *
 * @return value depending on what is returned by the called API
 */
static int32_t dwt_ioctl(dwchip_t *dw, dwt_ioctl_e fn, int32_t parm, void *ptr)
{
    int32_t ret = (int32_t)DWT_SUCCESS;

    switch (fn)
    {
    case DWT_WAKEUP:
        ull_wakeup_ic(dw);
        break;

    case DWT_FORCETRXOFF:
        ull_forcetrxoff(dw);
        break;

    case DWT_STARTTX:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            ret = ull_starttx(dw, *tmp);
        }
        break;

    case DWT_SETDELAYEDTRXTIME:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            ull_setdelayedtrxtime(dw, *tmp);
        }
        break;

    case DWT_SETKEYREG128:
        ull_set_keyreg_128(dw, (const dwt_aes_key_t *)ptr);
        break;

    case DWT_CONFIGURELEADDRESS:
        if (ptr != NULL)
        {
            struct dwt_configure_le_address_s *tmp = (struct dwt_configure_le_address_s *)ptr;
            ull_configure_le_address(dw, tmp->addr, tmp->leIndex);
        }
        break;

    case DWT_SET_TXPOWER:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            ull_settxpower(dw, *tmp);
        }
        break;

    case DWT_CONFIGURESFDTYPE:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            ull_configuresfdtype(dw, *tmp);
        }
        break;

    case DWT_SETTXCODE:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            ull_settxcode(dw, *tmp);
        }
        break;

    case DWT_SETRXCODE:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            ull_setrxcode(dw, *tmp);
        }
        break;

    case DWT_ENABLEGPIOCLOCKS:
        ull_enablegpioclocks(dw);
        break;

    case DWT_OTPREVISION:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            *tmp = ull_otprevision(dw);
        }
        break;

    case DWT_GETICREFVOLT:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            *tmp = ull_geticrefvolt(dw);
        }
        break;

    case DWT_GETICREFTEMP:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            *tmp = ull_geticreftemp(dw);
        }
        break;

    case DWT_GETPARTID:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            *tmp = ull_getpartid(dw);
        }
        break;

    case DWT_GETLOTID:
        if (ptr != NULL)
        {
            uint64_t *tmp = (uint64_t *)ptr;
            *tmp = ull_getlotid(dw);
        }
        break;

    case DWT_SIGNALRXBUFFFREE:
        ull_signal_rx_buff_free(dw);
        break;

    case DWT_SETRXAFTERTXDELAY:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            ull_setrxaftertxdelay(dw, *tmp);
        }
        break;

    case DWT_ENABLESPICRCCHECK:
        if (ptr != NULL)
        {
            struct dwt_enable_spi_crc_check_s *tmp = (struct dwt_enable_spi_crc_check_s *)ptr;
            ull_enablespicrccheck(dw, tmp->crc_mode, tmp->spireaderr_cb);
        }
        break;

    case DWT_ENABLEAUTOACK:
        if (ptr != NULL)
        {
            struct dwt_enable_auto_ack_s *tmp = (struct dwt_enable_auto_ack_s *)ptr;
            ull_enableautoack(dw, tmp->responseDelayTime, tmp->enable);
        }
        break;

    case DWT_CHECKDEVID:
        ret = ull_check_dev_id(dw);
        break;

    case DWT_CONFIGCIADIAG:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            ull_configciadiag(dw, *tmp);
        }
        break;

    case DWT_ENTERSLEEPAFTERTX:
        ull_entersleepaftertx(dw, parm);
        break;

    case DWT_ENTERSLEEPAFTER:
        ull_entersleepafter(dw, parm);
        break;

    case DWT_SETFINEGRAINTXSEQ:
        ull_setfinegraintxseq(dw, parm);
        break;

    case DWT_SETLNAPAMODE:
        ull_setlnapamode(dw, parm);
        break;

    case DWT_READPGDELAY:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            *tmp = ull_readpgdelay(dw);
        }
        break;

    case DWT_CONFIGURESTSKEY:
        ull_configurestskey(dw, ptr);
        break;

    case DWT_CONFIGURESTSIV:
        ull_configurestsiv(dw, ptr);
        break;

    case DWT_CONFIGURESTSLOADIV:
        ull_configurestsloadiv(dw);
        break;

    case DWT_CONFIGMRXLUT:
        ull_configmrxlut(dw, parm);
        break;

    case DWT_RESTORECONFIG:
        {
            dwt_restore_type_e tmp = (dwt_restore_type_e)parm;
            ret = ull_restoreconfig(dw, tmp);
        }
        break;
        
    case DWT_RESTORECOMMON:
        ull_restore_common(dw);
        break;
    
    case DWT_RESTORETXRX:
        {
          uint8_t tmp = (uint8_t)parm;
          ret = ull_restore_txrx(dw, tmp);
        }
        break;

    case DWT_CONFIGURESTSMODE:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            ull_configurestsmode(dw, *tmp);
        }
        break;

    case DWT_SETRXANTENNADELAY:
        if (ptr != NULL)
        {
            uint16_t *tmp = (uint16_t *)ptr;
            ull_setrxantennadelay(dw, *tmp);
        }
        break;

    case DWT_GETRXANTENNADELAY:
        if (ptr != NULL)
        {
            uint16_t* tmp = (uint16_t*)ptr;
            *tmp = ull_getrxantennadelay(dw);
        }
        break;

    case DWT_SETTXANTENNADELAY:
        if (ptr != NULL)
        {
            uint16_t *tmp = (uint16_t *)ptr;
            ull_settxantennadelay(dw, *tmp);
        }
        break;

    case DWT_GETTXANTENNADELAY:
        if (ptr != NULL)
        {
            uint16_t* tmp = (uint16_t*)ptr;
            *tmp = ull_gettxantennadelay(dw);
        }
        break;

    case DWT_WRITESCRATCHDATA:
        if (ptr != NULL)
        {
            struct dwt_rw_data_s *rd = (struct dwt_rw_data_s *)ptr;
            ull_write_scratch_data(dw, rd->buffer, rd->length, rd->offset);
        }
        break;

    case DWT_READSCRATCHDATA:
        if (ptr != NULL)
        {
            struct dwt_rw_data_s *rd = (struct dwt_rw_data_s *)ptr;
            ull_read_scratch_data(dw, rd->buffer, rd->length, rd->offset);
        }
        break;

    case DWT_READRXDATA:
        if (ptr != NULL)
        {
            struct dwt_rw_data_s *rd = (struct dwt_rw_data_s *)ptr;
            ull_readrxdata(dw, rd->buffer, rd->length, rd->offset);
        }
        break;

    case DWT_WRITETXDATA:
        if (ptr != NULL)
        {
            struct dwt_rw_data_s *wr = (struct dwt_rw_data_s *)ptr;
            (void)ull_writetxdata(dw, wr->length, wr->buffer, wr->offset);
        }
        break;

    case DWT_RXENABLE:
        (void)ull_rxenable(dw, parm);
        break;

    case DWT_WRITETXFCTRL:
        if (ptr != NULL)
        {
            struct dwt_tx_fctrl_s *txfctrl = (struct dwt_tx_fctrl_s *)ptr;
            ull_writetxfctrl(dw, txfctrl->txFrameLength, txfctrl->txBufferOffset, txfctrl->ranging);
        }
        break;

    case DWT_READCLOCKOFFSET:
        if (ptr != NULL)
        {
            int16_t *tmp = (int16_t *)ptr;
            *tmp = ull_readclockoffset(dw);
        }
        break;

    case DWT_READCARRIERINTEGRATOR:
        if (ptr != NULL)
        {
            int32_t *tmp = (int32_t *)ptr;
            *tmp = ull_readcarrierintegrator(dw);
        }
        break;

    case DWT_CLEARAONCONFIG:
        ull_clearaonconfig(dw);
        break;

    case DWT_CALCBANDWIDTHADJ:
        if (ptr != NULL)
        {
            struct dwt_calc_bandwidth_adj_s *tmp = (struct dwt_calc_bandwidth_adj_s *)ptr;
            tmp->result = ull_calcbandwidthadj(dw, tmp->target_count);
        }
        break;

    case DWT_READDIAGNOSTICS:
        ull_readdiagnostics(dw, (dwt_rxdiag_t *)ptr);
        break;

    case DWT_READDIAGNOSTICS_ACC:
        if(ptr != NULL)
        {
            struct dwt_readdiagnostics_acc_s *tmp = (struct dwt_readdiagnostics_acc_s *)ptr;
            ret = ull_readdiagnostics_acc(dw, tmp->cir_diag, tmp->acc_idx);
        }
        break;

    case DWT_READTXTIMESTAMPHI32:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            *tmp = ull_readtxtimestamphi32(dw);
        }
        break;

    case DWT_READTXTIMESTAMPLO32:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            *tmp = ull_readtxtimestamplo32(dw);
        }
        break;

    case DWT_READTXTIMESTAMP:
        ull_readtxtimestamp(dw, (uint8_t *)ptr);
        break;

    case DWT_READPDOA:
        if (ptr != NULL)
        {
            int16_t *tmp = (int16_t *)ptr;
            *tmp = ull_readpdoa(dw);
        }
        break;

    case DWT_READTDOA:
        ull_readtdoa(dw, (uint8_t *)ptr);
        break;

    case DWT_READWAKEUPTEMP:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            *tmp = ull_readwakeuptemp(dw);
        }
        break;

    case DWT_READWAKEUPVBAT:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            *tmp = ull_readwakeupvbat(dw);
        }
        break;

    case DWT_OTPWRITE:
        if (ptr != NULL)
        {
            struct dwt_opt_write_and_verify_s *tmp = (struct dwt_opt_write_and_verify_s *)ptr;
            ret = ull_otpwrite(dw, tmp->value, tmp->address);
        }
        break;

    case DWT_OTPWRITEANDVERIFY:
        if (ptr != NULL)
        {
            struct dwt_opt_write_and_verify_s *tmp = (struct dwt_opt_write_and_verify_s *)ptr;
            ret = ull_otpwriteandverify(dw, tmp->value, tmp->address);
        }
        break;

    case DWT_ENTERSLEEP:
        ull_entersleep(dw, parm);
        break;

    case DWT_CONFIGURESLEEPCNT:
        if (ptr != NULL)
        {
            uint16_t *tmp = (uint16_t *)ptr;
            ull_configuresleepcnt(dw, *tmp);
        }
        break;

    case DWT_CALIBRATESLEEPCNT:
        if (ptr != NULL)
        {
            uint16_t *tmp = (uint16_t *)ptr;
            *tmp = ull_calibratesleepcnt(dw);
        }
        break;

    case DWT_CONFIGURESLEEP:
        if (ptr != NULL)
        {
            struct dwt_configure_sleep_s *tmp = (struct dwt_configure_sleep_s *)ptr;
            ull_configuresleep(dw, tmp->mode, tmp->wake);
        }
        break;

    case DWT_SOFTRESET:
        ull_softreset(dw);
        break;

    case DWT_SETXTALTRIM:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            ull_setxtaltrim(dw, *tmp);
        }
        break;

    case DWT_GETXTALTRIM:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            *tmp = ull_getxtaltrim(dw);
        }
        break;

    case DWT_CONFIGCWMODE:
        ull_configcwmode(dw);
        break;

    case DWT_REPEATEDCW:
        if (ptr != NULL)
        {
            struct dwt_repeated_cw_s *tmp = (struct dwt_repeated_cw_s *)ptr;
            ull_repeated_cw(dw, tmp->cw_enable, tmp->cw_mode_config);
        }
        break;

    case DWT_READTEMPVBAT:
        if (ptr != NULL)
        {
            uint16_t *tmp = (uint16_t *)ptr;
            *tmp = ull_readtempvbat(dw);
        }
        break;

    case DWT_CONVERTRAWTEMP:
        if (ptr != NULL)
        {
            struct dwt_convert_raw_temp_s *tmp = (struct dwt_convert_raw_temp_s *)ptr;
            tmp->result = ull_convertrawtemperature(dw, tmp->raw_temp);
        }
        break;

    case DWT_CONVERTRAWVBAT:
        if (ptr != NULL)
        {
            struct dwt_convert_raw_volt_s *tmp = (struct dwt_convert_raw_volt_s *)ptr;
            tmp->result = ull_convertrawvoltage(dw, tmp->raw_voltage);
        }
        break;

    case DWT_CONFIGCONTINUOUSFRAMEMODE:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            ull_configcontinuousframemode(dw, *tmp);
        }
        break;

    case DWT_DISABLECONTINUOUSFRAMEMODE:
        ull_disablecontinuousframemode(dw);
        break;

    case DWT_DISABLECONTINUOUSWAVEMODE:
        ull_disablecontinuouswavemode(dw);
        break;

    case DWT_STOPREPEATEDFRAMES:
        ull_stop_repeated_frames(dw);
        break;

    case DWT_REPEATEDPREAMBLE:
        if (ptr != NULL)
        {
            struct dwt_repeated_p_s *tmp = (struct dwt_repeated_p_s *)ptr;
            ull_send_test_preamble(dw, tmp->delay, tmp->test_txpower);
        }
        break;

    case DWT_REPEATEDFRAMES:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            ull_repeated_frames(dw, *tmp);
        }
        break;

    case DWT_READSTSQUALITY:
        ret = ull_readstsquality(dw, (int16_t *)ptr);
        break;

    case DWT_DOAES:
        if (ptr != NULL)
        {
            struct dwt_do_aes_s *tmp = (struct dwt_do_aes_s *)ptr;
            tmp->result = ull_do_aes(dw, tmp->job, tmp->core_type);
        }
        break;

    case DWT_CONFIGUREAES:
        ull_configure_aes(dw, (const dwt_aes_config_t *)ptr);
        break;

    case DWT_MICSIZEFROMBYTES:
        if (ptr != NULL)
        {
            struct dwt_mic_size_from_bytes_s *tmp = (struct dwt_mic_size_from_bytes_s *)ptr;
            tmp->result = ull_mic_size_from_bytes(dw, tmp->mic_size_in_bytes);
        }
        break;

    case DWT_READEVENTCOUNTERS:
        ull_readeventcounters(dw, (dwt_deviceentcnts_t *)ptr);
        break;

    case DWT_CONFIGEVENTCOUNTERS:
        ull_configeventcounters(dw, parm);
        break;

    case DWT_SETPREAMBLEDETECTTIMEOUT:
        if (ptr != NULL)
        {
            uint16_t *tmp = (uint16_t *)ptr;
            ull_setpreambledetecttimeout(dw, *tmp);
        }
        break;

    case DWT_SETSNIFFMODE:
        if (ptr != NULL)
        {
            struct dwt_set_sniff_mode_s *tmp = (struct dwt_set_sniff_mode_s *)ptr;
            ull_setsniffmode(dw, tmp->enable, tmp->timeOn, tmp->timeOff);
        }
        break;

    case DWT_SETRXTIMEOUT:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            ull_setrxtimeout(dw, *tmp);
        }
        break;

    case DWT_AONREAD:
        if (ptr != NULL)
        {
            struct dwt_aon_read_s *tmp = (struct dwt_aon_read_s *)ptr;
            tmp->ret_val = ull_aon_read(dw, tmp->aon_address);
        }
        break;

    case DWT_AONWRITE:
        if (ptr != NULL)
        {
            struct dwt_aon_write_s *tmp = (struct dwt_aon_write_s *)ptr;
            ull_aon_write(dw, tmp->aon_address, tmp->aon_write_data);
        }
        break;

    case DWT_READSTSSTATUS:
        ret = ull_readstsstatus(dw, (uint16_t *)ptr, parm);
        break;

    case DWT_SETLEDS:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            ull_setleds(dw, *tmp);
        }
        break;

    case DWT_SETDWSTATE:
        ret = ull_setdwstate(dw, parm);
        break;

    case DWT_READSYSTIME:
        ull_readsystime(dw, (uint8_t *)ptr);
        break;

    case DWT_CHECKIDLERC:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            *tmp = ull_checkidlerc(dw);
        }
        break;

    case DWT_CHECKIRQ:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            *tmp = ull_checkirq(dw);
        }
        break;

    case DWT_CONFIGUREFRAMEFILTER:
        if (ptr != NULL)
        {
            struct dwt_configure_ff_s *tmp = (struct dwt_configure_ff_s *)ptr;
            ull_configureframefilter(dw, tmp->enabletype, tmp->filtermode);
        }
        break;

    case DWT_SETEUI:
        ull_seteui(dw, (uint8_t *)ptr);
        break;

    case DWT_GETEUI:
        ull_geteui(dw, (uint8_t *)ptr);
        break;

    case DWT_SETPANID:
        if (ptr != NULL)
        {
            uint16_t *tmp = (uint16_t *)ptr;
            ull_setpanid(dw, *tmp);
        }
        break;

    case DWT_SETADDRESS16:
        if (ptr != NULL)
        {
            uint16_t *tmp = (uint16_t *)ptr;
            ull_setaddress16(dw, *tmp);
        }
        break;

    case DWT_READRXTIMESTAMP:
        ull_readrxtimestamp(dw, (uint8_t *)ptr);
        break;

    case DWT_READRXTIMESTAMP_IPATOV:
        ull_readrxtimestamp_ipatov(dw, (uint8_t *)ptr);
        break;

    case DWT_READRXTIMESTAMPUNADJ:
        ull_readrxtimestampunadj(dw, (uint8_t *)ptr);
        break;

    case DWT_READRXTIMESTAMPHI32:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            *tmp = ull_readrxtimestamphi32(dw);
        }
        break;

    case DWT_READRXTIMESTAMPLO32:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            *tmp = ull_readrxtimestamplo32(dw);
        }
        break;

    case DWT_READRXTIMESTAMP_STS:
        ull_readrxtimestamp_sts(dw, (uint8_t *)ptr);
        break;

    case DWT_READSYSTIMESTAMPHI32:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            *tmp = ull_readsystimehi32(dw);
        }
        break;

    case DWT_OTPREAD:
        if (ptr != NULL)
        {
            struct dwt_otp_read_s *d = (struct dwt_otp_read_s *)ptr;
            ull_otpread(dw, d->address, d->array, d->length);
        }
        break;

    case DWT_SETPLENFINE:
        if (ptr != NULL)
        {
            uint16_t *tmp = (uint16_t *)ptr;
            ret = ull_setplenfine(dw, *tmp);
        }
        break;

    case DWT_SETPLLRXPREBUFEN:
        if (ptr != NULL)
        {
            dwt_pll_prebuf_cfg_e *tmp = (dwt_pll_prebuf_cfg_e *)ptr;
            ret = ull_setpllrxprebufen(dw, *tmp);
        }
        break;

    case DWT_RUNPGFCAL:
        ret = ull_run_pgfcal(dw);
        break;

    case DWT_PGF_CAL:
        ret = ull_pgf_cal(dw, parm);
        break;

    case DWT_CALCPGCOUNT:
        if (ptr != NULL)
        {
            struct dwt_calc_pg_count_s *tmp = (struct dwt_calc_pg_count_s *)ptr;
            tmp->result = ull_calcpgcount(dw, tmp->pgdly);
        }
        break;

    case DWT_PLL_STATUS:
        ret = (int32_t)ull_readpllstatus(dw);
        break;

    case DWT_PLL_CAL:
        ret = ull_pll_cal(dw);
        break;

    case DWT_CONFIGURE_RF_PORT:
        {
            dwt_rf_port_ctrl_e port_control = (dwt_rf_port_ctrl_e) parm;
            ull_configure_rf_port(dw, port_control);
        }
        break;

    case DWT_SETGPIOMODE:
        if (ptr != NULL)
        {
            struct dwt_set_gpio_mode_s *tmp = (struct dwt_set_gpio_mode_s *)ptr;
            ull_setgpiomode(dw, tmp->mask, tmp->mode);
        }
        break;

    case DWT_SETGPIODIR:
        if (ptr != NULL)
        {
            uint16_t *tmp = (uint16_t *)ptr;
            ull_setgpiodir(dw, *tmp);
        }
        break;

    case DWT_GETGPIODIR:
        if (ptr != NULL)
        {
            ull_getgpiodir(dw, (uint16_t *)ptr);
        }
        break;

    case DWT_SETGPIOVALUE:
        if (ptr != NULL)
        {
            struct dwt_set_gpio_value_s *tmp = (struct dwt_set_gpio_value_s *)ptr;
            ull_setgpiovalue(dw, tmp->gpio, tmp->value);
        }
        break;

    case DWT_SETDBLRXBUFFMODE:
        if (ptr != NULL)
        {
            struct dwt_set_dbl_rx_buff_mode_s *tmp = (struct dwt_set_dbl_rx_buff_mode_s *)ptr;
            ull_setdblrxbuffmode(dw, tmp->dbl_buff_state, tmp->dbl_buff_mode);
        }
        break;

    case DWT_SETREFERENCETRXTIME:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            ull_setreferencetrxtime(dw, *tmp);
        }
        break;

    case DWT_READ_REG:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            *tmp = dwt_read32bitreg(dw, (uint32_t)parm);
        }
        break;

    case DWT_WRITE_REG:
        dwt_write32bitreg(dw, (uint32_t)parm, (uint32_t)(uint32_t *)ptr);
        break;

    case DWT_GETDGCDECISION:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            *tmp = ull_get_dgcdecision(dw);
        }
        break;

#ifdef WIN32
    case DWT_SPICSWAKEUP:
        if (ptr != NULL)
        {
            struct dwt_spi_cs_wakeup_s *tmp = (struct dwt_spi_cs_wakeup_s *)ptr;
            ret = ull_spicswakeup(dw, tmp->buff, tmp->length);
        }
        break;
#endif // WIN32

    case DWT_WRITESYSSTATUSLO:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            ull_writesysstatuslo(dw, *tmp);
        }
        break;

    case DWT_WRITESYSSTATUSHI:
        if (ptr != NULL)
        {
            /*
             * DW3000 SYS_STATUS_HI register is 13 bits wide, thus we revert to a 16-bit write of the register
             * instead of using a 32-bit write. Only DW3720 devices require a 32-bit write.
             */
            uint32_t *tmp = (uint32_t *)ptr;
            ull_writesysstatushi(dw, (uint16_t)*tmp);
        }
        break;

    case DWT_READSYSSTATUSLO:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            *tmp = ull_readsysstatuslo(dw);
        }
        break;

    case DWT_READSYSSTATUSHI:
        if (ptr != NULL)
        {
            /*
             * DW3000 SYS_STATUS_HI register is 13 bits wide, thus we revert to a 16-bit read of the register
             * instead of using a 32-bit read. Only DW3720 devices require a 32-bit write.
             */
            uint32_t *tmp = (uint32_t *)ptr;
            *tmp = (uint32_t)ull_readsysstatushi(dw);
        }
        break;

    case DWT_WRITERDBSTATUS:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            ull_writerdbstatus(dw, *tmp);
        }
        break;

    case DWT_READRDBSTATUS:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            *tmp = ull_readrdbstatus(dw);
        }
        break;

    case DWT_GETFRAMELENGTH:
        if (ptr != NULL)
        {
            struct dwt_getframelength_s *tmp = (struct dwt_getframelength_s *)ptr;
            tmp->frame_len = ull_getframelength(dw, &tmp->rng_bit);
        }
        break;

    case DWT_READGPIOVALUE:
        if (ptr != NULL)
        {
            uint16_t *tmp = (uint16_t *)ptr;
            *tmp = ull_readgpiovalue(dw);
        }
        break;

    case DWT_READPDOAOFFSET:
        if (ptr != NULL)
        {
            uint32_t *tmp = (uint32_t *)ptr;
            *tmp = dwt_read32bitreg(dw, CIA_ADJUST_ID);
        }
        break;

    case DWT_SETPDOAOFFSET:
        if (ptr != NULL)
        {
            uint16_t tmp = *((uint16_t *)ptr);
            tmp &= CIA_ADJUST_PDOA_ADJ_OFFSET_BIT_MASK;
            dwt_modify16bitoffsetreg(dw, CIA_ADJUST_ID, 0U, (uint16_t)~CIA_ADJUST_PDOA_ADJ_OFFSET_BIT_MASK, tmp);
        }
        break;

    case DWT_ADJ_TXPOWER:
        if (ptr != NULL)
        {
            struct dwt_adj_tx_power_s *tmp = (struct dwt_adj_tx_power_s *)ptr;
            tmp->result = ull_adjust_tx_power(tmp->boost, tmp->ref_tx_power, tmp->channel, tmp->adj_tx_power, tmp->applied_boost);
        }
        break;

    case DWT_LINEAR_TXPOWER:
        if (ptr != NULL)
        {
            struct dwt_calculate_linear_tx_power_s *tmp = (struct dwt_calculate_linear_tx_power_s *)ptr;
            tmp->result = ull_calculate_linear_tx_power(tmp->channel, tmp->txp_indexes, tmp->txp_res);
        }
        break;

    case DWT_CONVERT_TXPOWER_TO_IDX:
        if (ptr != NULL)
        {
            struct dwt_convert_tx_power_to_index_s *tmp = (struct dwt_convert_tx_power_to_index_s *)ptr;
            tmp->result = ull_convert_tx_power_to_index(tmp->channel, tmp->tx_power, tmp->tx_power_idx);
        }
        break;

    case DWT_SET_PLLBIASTRIM:
        if (ptr != NULL)
        {
            uint8_t tmp = *((uint8_t *)ptr);
            ull_setpllbiastrim(dw, tmp);
        }
        break;

    case DWT_CFGWIFICOEXSET:
        if (ptr != NULL)
        {
            struct dwt_cfg_wifi_coex_set_s *tmp = (struct dwt_cfg_wifi_coex_set_s *)ptr;
            ull_wifi_coex_set(dw, tmp->enable, tmp->coex_io_swap);
        }
        break;

    case DWT_CFGANTSEL:
        if (ptr != NULL)
        {
            uint8_t *tmp = (uint8_t *)ptr;
            ull_configure_and_set_antenna_selection_gpio(dw, *tmp);
        }
        break;

    case DWT_RSTSYSTEMCNT:
        ull_reset_system_counter(dw);
        break;

    case DWT_CFGOSTRMODE:
        if (ptr != NULL)
        {
            struct dwt_ostr_mode_s *tmp = (struct dwt_ostr_mode_s *)ptr;
            ull_config_ostr_mode(dw, tmp->enable, tmp->wait_time);
        }
        break;

    /* DW3000 salt */
    case DWT_PLL_AUTO_CAL:
        if(ptr != NULL)
        {
        	struct dwt_set_pll_cal_s *tmp = (struct dwt_set_pll_cal_s *)ptr;
            uint8_t steps_to_lock;

            if(parm == 5)
            {
            	ret = (int32_t)ull_pll_ch5_auto_cal(dw, tmp->coarse_code, tmp->sleep,
                        tmp->steps, &steps_to_lock, tmp->temp);
            }
            else
            {
            	ret = (int32_t)ull_pll_ch9_auto_cal(dw, tmp->coarse_code, tmp->sleep,
                        tmp->steps, &steps_to_lock );
            }

            if(ret == (int32_t)DWT_SUCCESS)
            {
                ret = (int32_t)steps_to_lock;
            }
        }
    	break;

        /* MCPS salt */
    case DWT_SET_STS_LEN:
        if (ptr != NULL)
        {
            dwt_sts_lengths_e *sts_len = (dwt_sts_lengths_e*)ptr;
	    ull_setstslength(dw, *sts_len);
        }
        break;

    case DWT_SETPDOAMODE:
        ret = ull_setpdoamode(dw, (dwt_pdoa_mode_e)parm);
        break;

    case DWT_SET_FCS_MODE:
        if (ptr != NULL)
        {
            uint8_t *fcs_mode = (uint8_t *)ptr;

            uint32_t fcs = dwt_read32bitoffsetreg(dw,SYS_CFG_ID,0U) & ~(SYS_CFG_DIS_FCS_TX_BIT_MASK | SYS_CFG_DIS_FCE_BIT_MASK);

            if (((uint8_t)*fcs_mode & (uint8_t)DWT_FCS_TX_OFF) != 0U)
            {
                fcs |= SYS_CFG_DIS_FCS_TX_BIT_MASK;
            }

            if (((uint8_t)*fcs_mode & (uint8_t)DWT_FCS_RX_OFF) != 0U)
            {
                fcs |= SYS_CFG_DIS_FCE_BIT_MASK;
            }

            dwt_write32bitoffsetreg(dw, SYS_CFG_ID, 0U, fcs);

            LOCAL_DATA(dw)->sys_cfg_dis_fce_bit_flag = ((fcs & SYS_CFG_DIS_FCE_BIT_MASK) != 0UL ? 1U : 0U);
        }
        break;

    case DWT_SET_PHR:
        if (ptr != NULL)
        {
            struct dwt_set_phr_s *tmp = (struct dwt_set_phr_s *)ptr;
            dwt_modify32bitoffsetreg(dw, SYS_CFG_ID, 0U, ~(uint32_t)(SYS_CFG_PHR_MODE_BIT_MASK | SYS_CFG_PHR_6M8_BIT_MASK),
            ((SYS_CFG_PHR_6M8_BIT_MASK & ((uint32_t)tmp->phrRate << SYS_CFG_PHR_6M8_BIT_OFFSET)) | (uint32_t)tmp->phrMode));
        }
        break;

    case DWT_SET_DATARATE:
        if (ptr != NULL)
        {
            dwt_uwb_bit_rate_e *bitRate = (dwt_uwb_bit_rate_e*)ptr;
            dwt_modify32bitoffsetreg(dw, TX_FCTRL_ID, 0U, ~(uint32_t)(TX_FCTRL_TXBR_BIT_MASK), (uint32_t)*bitRate<< TX_FCTRL_TXBR_BIT_OFFSET);
        }
        break;

    case DWT_SET_PAC:
        if (ptr != NULL)
        {
            dwt_pac_size_e *rxPAC = (dwt_pac_size_e*)ptr;
            dwt_modify8bitoffsetreg(dw, DTUNE0_ID, 0U, (uint8_t) ~(DTUNE0_PRE_PAC_SYM_BIT_MASK), (uint8_t)*rxPAC);
        }
        break;

    case DWT_SET_SFDTO:
        if (ptr != NULL)
        {
                uint16_t *sfdTO = (uint16_t *)ptr;
                if (*sfdTO == 0U)
                {
                    *sfdTO = DWT_SFDTOC_DEF;
                }
                dwt_write16bitoffsetreg(dw, DTUNE0_ID, 2U, *sfdTO);
        }
        break;

    case DWT_READDGCDBG:
        if (ptr != NULL)
        {
            uint32_t* tmp = (uint32_t*)ptr;
            *tmp = dwt_read32bitreg(dw, DGC_DBG_ID);
        }
        break;

    case DWT_READCTRDBG:
        if (ptr != NULL)
        {
            uint32_t* tmp = (uint32_t*)ptr;
            *tmp = dwt_read32bitreg(dw, CTR_DBG_ID);
        }
        break;

    case DWT_GET_CIR_REGADD:
        if (ptr != NULL)
        {
            uint32_t* tmp = (uint32_t*)ptr;
            *tmp = ACC_MEM_ID;
        }
        break;

    case DWT_CIA_VERSION:
        if (ptr != NULL)
        {
            uint32_t* tmp = (uint32_t*)ptr;
            *tmp = ull_readCIAversion(dw);
        }
        break;

    case DWT_NLOS_IPDIAG:
        if (ptr != NULL)
        {
            dwt_nlos_ipdiag_t *tmp = (dwt_nlos_ipdiag_t *)ptr;
            ull_nlos_ipdiag(dw, tmp);
        }
        break;

    case DWT_NLOS_ALLDIAG:
        if (ptr != NULL)
        {
            dwt_nlos_alldiag_t *tmp = (dwt_nlos_alldiag_t *)ptr;
            ret = ull_nlos_alldiag(dw, tmp);
        }
        break;

    case DWT_DIS_OTP_IPS:
        ull_dis_otp_ips(dw, parm);
        break;

    case DWT_CALCULATE_RSSI:
        if (ptr != NULL)
        {
            struct dwt_calculate_rssi_s *tmp= (struct dwt_calculate_rssi_s *)ptr;
            ret = ull_calculate_rssi(dw, tmp->cir_diagnostics, tmp->acc_idx, tmp->signal_strength);
        }
        break;

    case DWT_CALCULATE_FIRST_PATH_POWER:
        if (ptr != NULL)
        {
            struct dwt_calculate_first_path_power_s *tmp= (struct dwt_calculate_first_path_power_s *)ptr;
            ret = ull_calculate_first_path_power(dw, tmp->cir_diagnostics, tmp->acc_idx, tmp->signal_strength);
        }
        break;

    case DWT_SET_ISR_FLAGS:
        dw->isrFlags = (dwt_isr_flags_e)parm;
	    break;

    default:
        ret = -1; //_Err_not_supported;
        break;
    }

    return (ret);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this a chip-specific implementation of dbg_fn()
 *
 * */
static void* dwt_dbg_fn(dwchip_t* dw, dwt_ioctl_e fn, int32_t parm, void* ptr)
{
    void* ret = NULL;
    (void)dw;
    (void)parm;
    (void)ptr;

    switch (fn)
    {

    case DWT_DBG_REGS:
        {
            ret = (void*) regNames;
        }
        break;

    default:
        ret = NULL; //_Err_not_supported;
        break;
    }

    return (ret);
}

#ifdef AUTO_DW3300Q_DRIVER

/* Non-STD interface fn() */
static int32_t init_no_chan(struct dwchip_s *dw)
{
    int32_t ret;
    ret = ull_initialise(dw, dw->config->mode);
    dw->SPI->setfastrate();
    uint32_t dev_id;
#define DW3XXX_DEVICE_ID 0
    dwt_ioctl(dw, DWT_READ_REG, DW3XXX_DEVICE_ID , (void *)&dev_id);

    ull_setinterrupt(dw, dw->config->bitmask_lo, dw->config->bitmask_hi, dw->config->int_options);
    // Apply XTAL TRIM from the OTP or use the DEFAULT_XTAL_TRIM
    uint8_t trim = ull_getxtaltrim(dw);
    if ((trim == DEFAULT_XTAL_TRIM) || (dw->config->xtalTrim & ~XTAL_TRIM_BIT_MASK))
    {
        trim = dw->config->xtalTrim & XTAL_TRIM_BIT_MASK;
        ull_setxtaltrim(dw, trim);
    }
    return ret;
}
#endif /* AUTO_DW3300Q_DRIVER */

/* Non-STD interface fn() */
static int32_t init(struct dwchip_s *dw)
{
    int32_t ret = ull_initialise(dw, dw->config->mode);
    dw->SPI->setfastrate();
    uint32_t dev_id;
#define DW3XXX_DEVICE_ID  0
    (void)dwt_ioctl(dw, DWT_READ_REG, DW3XXX_DEVICE_ID , (void *)&dev_id);

    ret = ull_configure(dw, dw->config->rxtx_config->pdwCfg);
    ull_configuretxrf(dw, dw->config->rxtx_config->txConfig);

    /* set antenna delays */
    ull_setrxantennadelay(dw, dw->config->rxtx_config->rxAntDelay);
    ull_settxantennadelay(dw, dw->config->rxtx_config->txAntDelay);

    ull_setrxaftertxdelay(dw, 0); /**< no any delays set by default : part of config of receiver on Tx sending */
    ull_setrxtimeout(dw, 0);      /**< no any delays set by default : part of config of receiver on Tx sending */

    ull_configureframefilter(dw, dw->config->rxtx_config->frameFilter, dw->config->rxtx_config->frameFilterMode);

    ull_setpanid(dw, dw->config->rxtx_config->panId);

    ull_setaddress16(dw, dw->config->rxtx_config->shortadd);

    ull_setleds(dw, (uint8_t)dw->config->led_mode);

    ull_setlnapamode(dw, dw->config->lnapamode);

    ull_setinterrupt(dw, (uint32_t)dw->config->bitmask_lo, (uint32_t)dw->config->bitmask_hi, (dwt_INT_options_e)dw->config->int_options);

    ull_configuresleep(dw, dw->config->sleep_config.mode, dw->config->sleep_config.wake);

    // Apply XTAL TRIM from the OTP or use the DEFAULT_XTAL_TRIM
    uint8_t trim = ull_getxtaltrim(dw);
    if ((trim == DEFAULT_XTAL_TRIM) || ((dw->config->xtalTrim & ~(uint8_t)XTAL_TRIM_BIT_MASK) != 0U))
    {
        trim = dw->config->xtalTrim & XTAL_TRIM_BIT_MASK;
        ull_setxtaltrim(dw, trim);
    }

    ull_configciadiag(dw, dw->config->cia_enable_mask);

    ull_configurestskey(dw, dw->config->stsKey);
    ull_configurestsiv(dw, dw->config->stsIv);
    if (dw->config->loadIv != 0U)
    {
        ull_configurestsloadiv(dw);
    }

    ull_configeventcounters(dw, (int32_t)dw->config->event_counter);

    if (dw->coex_gpio_pin >= 0)
    {
        uint16_t gpio_direction;
        /* Ensure selected GPIO is well configured */
        uint16_t gpio = (uint16_t)(1UL << (uint8_t)dw->coex_gpio_pin);

        ull_setgpiomode(dw, gpio, ENABLE_ALL_GPIOS_MASK);
        ull_readfromdevice(dw, GPIO_MODE_ID, 0U, 2U, (uint8_t*)&gpio_direction);
        // Setting GPIO direction to 0 = output
        gpio_direction &= ~gpio;
        ull_setgpiodir(dw, gpio_direction);

        /* Set pin for Wi-Fi coexistance signaling */
        ull_setgpiovalue(dw, gpio, (dw->coex_gpio_active_state != 0) ? (0) : (1));
    }

    return ret;
}

static void deinit(struct dwchip_s *p)
{
    (void)p;
    return;
}

//_prs: this fn() are for compatibility with MCPS: could migrate to the interface
static int32_t prs_sys_status_and_or(struct dwchip_s *dw, uint32_t and_value, uint32_t or_value)
{
    dwt_modify32bitoffsetreg(dw, SYS_STATUS_ID, 0U, and_value, or_value);
    return 0;
}

static void prs_ack_enable(struct dwchip_s *dw, int32_t en)
{
    dwt_modify8bitoffsetreg(dw, SYS_CFG_ID, 1U, (en != 0) ? (~(uint8_t)(SYS_CFG_AUTO_ACK_BIT_MASK >> 8U)) : (uint8_t)(0xFFU), (en != 0) ? (uint8_t)(SYS_CFG_AUTO_ACK_BIT_MASK >> 8U) : (0U));
    return;
}

/* chip-specific operation structure with chip-specific ioctl() */
static const struct dwt_ops_s dw3000_ops = {

    // interface fn() here
    .configure = ull_configure,
    .write_tx_data = ull_writetxdata,
    .write_tx_fctrl = ull_writetxfctrl,
    .read_rx_data = ull_readrxdata,
    .read_acc_data = ull_readaccdata,
    .read_cir = ull_readcir,
    .read_rx_timestamp = ull_readrxtimestamp,
    .configure_tx_rf = ull_configuretxrf,
    .set_interrupt = ull_setinterrupt,
    .rx_enable = ull_rxenable,
    .initialize = ull_initialise,
    .xfer = dwt_xfer3xxx,
    // ioctl()
    .ioctl = dwt_ioctl, // chip-specific
    .isr = ull_isr,
    .dbg_fn = dwt_dbg_fn,
};

static const struct dwt_mcps_ops_s dw3000_mcps_ops = {
#ifndef WIN32
    .init = init,
#ifdef AUTO_DW3300Q_DRIVER
    .init_no_chan = init_no_chan,
#endif
    .deinit = deinit,
    .tx_frame = interface_tx_frame,
    .rx_enable = interface_rx_enable,
    .rx_disable = interface_rx_disable,
    .get_timestamp = interface_get_timestamp,
    .get_rx_frame = interface_read_rx_frame,
    .set_hrp_uwb_params = NULL,
    .set_channel = ull_setchannel,
    .set_hw_addr_filt = NULL,
    .write_to_device = ull_writetodevice,
    .read_from_device = ull_readfromdevice,
#endif
    .ioctl = dwt_ioctl, // chip-specific

    .mcps_compat = {
        .sys_status_and_or = prs_sys_status_and_or,
        .ack_enable = prs_ack_enable,
        .set_interrupt = ull_setinterrupt
    },

    .isr = ull_isr,
};

/* DW3000 Driver descriptor */
#ifdef WIN32
#pragma section(".dwt$c") // Specific entry for DW3000 structure
__declspec(allocate(".dwt$c")) const struct dwt_driver_s dw3000_driver = {
#else
const struct dwt_driver_s dw3000_driver = {
#endif // WIN32
    .devid = (uint32_t)DWT_DW3000_PDOA_DEV_ID,
    .devmatch = 0xFFFFFF0FUL,
    .name = DRIVER_NAME,
    .version = DRIVER_VERSION_STR,
    .dwt_ops = &dw3000_ops,
    .dwt_mcps_ops = &dw3000_mcps_ops,
    .vernum = DRIVER_VERSION_HEX
};

/* ===============================================================================================
   List of expected (known) device ID handled by this software
   ===============================================================================================

   as defined in the deca_device_api.h

   ===============================================================================================
*/
