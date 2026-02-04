/**
 * @file      deca_device_api.h
 *
 * @brief     QM33xxx Device API Definitions and Functions
 *
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */
#ifndef DECA_DEVICE_API_H
#define DECA_DEVICE_API_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "deca_types.h"

#define DWT_DEBUG_PRINT  0 //!< set to 1 to enable debug prints
#if (DWT_DEBUG_PRINT == 1)
#include <stdio.h>
#endif

#ifndef DWT_NUM_DW_DEV
#define DWT_NUM_DW_DEV (1)
#endif

#define DWT_BIT_MASK(bit_num) (((uint32_t)1) << (bit_num))

/*! @name QM33xxx and DW3xxx IC ID enums, device IDs belonging to QM33xxx/DW3xxx family
 *@{
 */
    typedef enum
    {
        DWT_DW3000_DEV_ID = 0xDECA0302,      //!< DW3000 (non PDOA) silicon device ID ($$$)
        DWT_QM33110_DEV_ID = 0xDECA0304,     //!< QM33110 (non PDOA) silicon device ID
        DWT_DW3000_PDOA_DEV_ID = 0xDECA0312, //!< DW3000 (with PDOA) silicon device ID
        DWT_QM33120_PDOA_DEV_ID = 0xDECA0314 //!< QM33120 (with PDOA) silicon device ID
    } dw_chip_id_e;

#define DWT_DW3720_PDOA_DEV_ID   DWT_QM33120_PDOA_DEV_ID   //!< Backward compatibility definition of the product number (P/N)

/*! QM33 and DW IC types (AOA or non-AON) */
    typedef enum
    {
        AOA,        //!< device which supports AOA feature (has two RF ports)
        NON_AOA     //!< device which does not support AOA feature (has a single RF ports)
    } dw3000type_e;
/**@}*/

/*! @name Defines for converting between DW IC freq. and time units
 *@{
 */
#define DELAY_20uUSec       (20UL) //!< Delay of 20us (used by deca_usleep(), this is platform/MCU specific implementation)
                                   //!< On nRF this takes about 24 us.

#define DWT_TIME_UNITS (1.0 / 499.2e6 / 128.0) //!< Units are (1.0 / 499.2e6 / 128.0)s ~= 15.65e-12 s

#define DW3000_CHIP_FREQ    499200000ULL //!< Frequency is 499.2 MHz
#define DW3000_CHIP_PER_DTU 2ULL
#define DW3000_CHIP_PER_DLY 512U
#define DW3000_DTU_FREQ     (DW3000_CHIP_FREQ / DW3000_CHIP_PER_DTU)
#define DW3000_DTU_FREQ_S   ((int64_t)DW3000_CHIP_FREQ / (int64_t)DW3000_CHIP_PER_DTU)
#define DTU_TO_US(x)        (uint32_t)((uint64_t)(x) * 1000000ULL / DW3000_DTU_FREQ)
#define US_TO_DTU(x)        (uint32_t)((uint64_t)(x) * DW3000_DTU_FREQ / 1000000ULL)
#define DTU_TO_US_S(x)      (int32_t)((int64_t)(x) * 1000000LL / DW3000_DTU_FREQ_S)
#define US_TO_DTU_S(x)      (int32_t)((int64_t)(x) * DW3000_DTU_FREQ_S / 1000000LL)
/**@}*/

/*! @name Defines and enums for SPI CRC mode
 *@{
 */
 /*! When this define is defined the code to handle SPI CRC functionality is enabled,
                             otherwise the code will be disabled to save space, e.g., when SPI CRC is not required. */
#define DWT_ENABLE_CRC

/*! Defined constants when SPI CRC mode is used */
    typedef enum
    {
        DWT_SPI_CRC_MODE_NO = 0, //!< No SPI CRC
        DWT_SPI_CRC_MODE_WR,     //!< This is used to enable SPI CRC check (the SPI CRC check will be enabled on DW IC and CRC-8 added for SPI write transactions)
        DWT_SPI_CRC_MODE_WRRD    //!< This is used to optionally enable additional CRC check on the SPI read operations, while the CRC check on the SPI write operations is also enabled
    } dwt_spi_crc_mode_e;
/**@}*/

/*! @name DW IC SPI modes used in the SPI transactions */
    typedef enum
    {
        DW3000_SPI_RD_BIT = 0x0000U,     //!< SPI read register CMD
        DW3000_SPI_RD_FAST_CMD = 0x0001U,//!< SPI read "fast" CMD (two byte transaction, header byte + read byte)
        DW3000_SPI_WR_FAST_CMD = 0x0002U,//!< SPI write "fast" CMD (single byte transaction)
        DW3000_SPI_WR_BIT = 0x8000U,     //!< SPI write register CMD
        DW3000_SPI_AND_OR_8 = 0x8001U,   //!< SPI modify register CMD (apply AND mask and then OR mask, on the 8-bit register)
        DW3000_SPI_AND_OR_16 = 0x8002U,  //!< SPI modify register CMD (apply AND mask and then OR mask, on the 16-bit register)
        DW3000_SPI_AND_OR_32 = 0x8003U,  //!< SPI modify register CMD (apply AND mask and then OR mask, on the 32-bit register)
    } spi_modes_e;


/*! @name General API error codes */
    typedef enum
    {
        DWT_SUCCESS = 0,         //!< No error
        DWT_ERROR = -1,          //!< Error
        DWT_ERR_PLL_LOCK = -2,   //!< PLL lock error / PLL configuration/calibration failed
        DWT_ERR_RX_CAL_PGF = -3, //!< PGF calibration failed
        DWT_ERR_RX_CAL_RESI = -4,//!< RX clibration failed
        DWT_ERR_RX_CAL_RESQ = -5,//!< RX clibration failed
        DWT_ERR_RX_ADC_CAL = -6, //!< ADC clibration failed
        DWT_ERR_WRONG_STATE = -9 //!< TSE is not in IDLE, need to call dwt_forcetrxoff() first to place TSE in IDLE
    } dwt_error_e;


/*! @name Defines used for PLL and PGF calibration routines
 *
 *@{
 */
#define DWT_DEF_PLLBIASTRIM 7U //!< Default PLL bias trim value

#define MAX_RETRIES_FOR_PLL (50U) /*!< The PLL calibration should take less than 400 us,
                                       typically it is < 100 us (however on some parts (process corners) with channel 9 it can take ~900 us) */
#define MAX_PLL_CAL_LOOP    (2)   //!< Maximum number of retries for the PLL calibration routine
#define MAX_RETRIES_FOR_PGF (3U)  //!< Maximum number of retries for PGF calibration routine
/**@}*/


/*! @name Enums for selecting the bit rate for data transmission (and reception)
 *
 *  These are defined for writing (with just a shift) the TX_FCTRL register
 *@{
 */
    typedef enum
    {
        DWT_BR_850K = 0,   //!< UWB bit rate 850 kbits/s
        DWT_BR_6M8 = 1,    //!< UWB bit rate 6.8 Mbits/s
        DWT_BR_NODATA = 2, //!< No data (SP3 packet format)
    } dwt_uwb_bit_rate_e;
/**@}*/

/*! @name Enums for specifying the (nominal) mean Pulse Repetition Frequency
*
*   These are defined for direct writing (with a shift if necessary) to CHAN_CTRL and TX_FCTRL regs
*@{
*/
    typedef enum
    {
        DWT_PRF_16M = 1, //!< UWB PRF 16 MHz
        DWT_PRF_64M = 2, //!< UWB PRF 64 MHz
        DWT_PRF_SCP = 3, //!< SCP UWB PRF ~100 MHz (proprietary PRF only available in DW3xxx/QM33xxx parts)
    } dwt_prf_e;
/**@}*/

/*! @name Enums for specifying Preamble Acquisition Chunk (PAC) Size in symbols
 *@{
 */
    typedef enum
    {
        DWT_PAC8 = 0,  //!< PAC  8 (recommended for reception of preamble length of 128 and below)
        DWT_PAC16 = 1, //!< PAC 16 (recommended for reception of preamble length of 256)
        DWT_PAC32 = 2, //!< PAC 32 (recommended for reception of preamble length of 512)
        DWT_PAC4 = 3,  //!< PAC  4 (recommended for reception of preamble length of less than 128)
    } dwt_pac_size_e;
/**@}*/

/*! @name Enums for specifying SFD types and size
 *@{
 */
    typedef enum
    {
        DWT_SFD_IEEE_4A = 0, //!< IEEE 8-bit ternary
        DWT_SFD_DW_8 = 1,    //!< Decawave/Qorvo propriatary 8-bit
        DWT_SFD_DW_16 = 2,   //!< Decawave/Qorvo propriatary 16-bit
        DWT_SFD_IEEE_4Z = 3, //!< IEEE 8-bit binary (4z)
        DWT_SFD_LEN8 = 8,    //!< IEEE, and Decawave/Qorvo 8-bit are length 8
        DWT_SFD_LEN16 = 16,  //!< Decawave/Qorvo 16-bit is length 16
    } dwt_sfd_type_e;
/**@}*/

/*! @name Common preamble length codes
*
* These constants can be used with for dwt_setplenfine() and dwt_configure().
* @{
*/
    #define DWT_PLEN_4096 (4096U) //!< Standard preamble length of 4096 symbols
    #define DWT_PLEN_2048 (2048U) //!< Non-standard preamble length of 2048 symbols
    #define DWT_PLEN_1536 (1536U) //!< Non-standard preamble length of 1536 symbols
    #define DWT_PLEN_1024 (1024U) //!< Standard preamble length of 1024 symbols
    #define DWT_PLEN_512  (512U) //!< Non-standard preamble length of 512 symbols
    #define DWT_PLEN_256  (256U) //!< Non-standard preamble length of 256 symbols
    #define DWT_PLEN_128  (128U) //!< Non-standard preamble length of 128 symbols
    #define DWT_PLEN_72   (72U) //!< Non-standard preamble length of 72 symbols
    #define DWT_PLEN_64   (64U) //!< Standard preamble length of 64 symbols
    #define DWT_PLEN_32   (32U) //!< Non-standard preamble length of 32 symbols
    #define DWT_PLEN_16   (16U) //!< Non-standard preamble length of 16 symbols (not recommended for DW3xxx/QM33xxx parts)

    /*! Check preamble length is in allowed range, between 16 and 2048 or 4096 symbols.
        The preamble length must be multiple of 8 symbols. */
    #define CHECK_PREAMBLE_LEN_VALIDITY(x) (((((x) >= DWT_PLEN_16) && ((x) <= DWT_PLEN_2048)) && (((x) % 8U) == 0U)) || ((x) == DWT_PLEN_4096))

    #define DWT_SFDTOC_DEF 129U //!< Default SFD timeout value (matches the device's default Ipatov length of 128 symbols)
/**@}*/

/*! @name Enums for selecting PHR modes
* @{
*/
    typedef enum
    {
        DWT_PHRMODE_STD = 0x0, //!< Standard PHR mode
        DWT_PHRMODE_EXT = 0x1, //!< Extended frames PHR mode (frame length of 0-1023)
    } dwt_phr_mode_e;
/**@}*/

/*! @name Enums for selecting PHR rate
* @{
*/
    typedef enum
    {
        DWT_PHRRATE_STD = 0x0, //!< Standard PHR rate
        DWT_PHRRATE_DTA = 0x1, //!< PHR sent at data rate (6.81 Mbps)
    } dwt_phr_rate_e;
/**@}*/

/*! @name Enums for selecting Frame Check Sequence (FCS) generation and check in TX and RX modes
* @warning If both FCS and CIA run are disabled then RXFWTO should be configured otherwise the receiver 
*          will not generate end of packet (RXFR and RXFCE) events/interrupts. 
* @{
*/
    typedef enum
    {
        DWT_FCS_ENABLE = 0x0, //!< Default mode, enable FCS generation in the TX and check in RX
        DWT_FCS_TX_OFF = 0x1, //!< Disable FCS generation in the transmitter
        DWT_FCS_RX_OFF = 0x2  //!< Disable FCS checking in the receiver
    } dwt_fcs_mode_e;

#define FCS_LEN               2UL //!< Default FCS is 2 bytes

/**@}*/

/*! @name Enums for setting DW3xxx/QM33xxx PDOA modes
* @{
*/
    typedef enum
    {
        DWT_PDOA_M0 = 0x0, //!< DW PDOA mode is off
        DWT_PDOA_M1 = 0x1, //!< DW PDOA mode 1
        DWT_PDOA_M3 = 0x3, //!< DW PDOA mode 3
    } dwt_pdoa_mode_e;
/**@}*/

/*! @name Enums for setting DW3xxx/QM33xxx STS modes (packet types)
* @{
*/
    typedef enum
    {
        DWT_STS_MODE_OFF = 0x0, //!< STS is off (SP0)
        DWT_STS_MODE_1 = 0x1,   //!< STS mode 1 (SP1)
        DWT_STS_MODE_2 = 0x2,   //!< STS mode 2 (SP2)
        DWT_STS_MODE_ND = 0x3,  //!< STS with no data (SP3)
        DWT_STS_MODE_SDC = 0x8, //!< Enable Super Deterministic Codes (STS is SDC, KEY/IV are not used)
        DWT_STS_CONFIG_MASK = 0xB,
        DWT_STS_CONFIG_MASK_NO_SDC = 0x3,
    } dwt_sts_mode_e;
/**@}*/

/*! @name Enums for setting DW3xxx/QM33xxx RX/TX PLL Pre-Buffers Enable Configuration
* @{
*/
    typedef enum
    {   
        DWT_PLL_RX_PREBUF_DISABLE = 0,  //!< Disable the DW RX PLL Pre-Buffers
        DWT_PLL_RX_PREBUF_ENABLE,       //!< Enable the DW RX PLL Pre-Buffers
    } dwt_pll_prebuf_cfg_e;
/**@}*/

/*! @name CIR/accumulator index values
*        @parblock
*        These are to be used in various API calls and are arranged sequentially and can be used as array indices.
*        Starting with Ipatov, then first STS block and then the second STS block (when PDOA mode 3 is used).
*
*        Ipatov is at complex sample offset 0x0 from ACC_MEM_ID, contains the Ipatov segnemet/block.
*        STS1 is at complex sample offset 0x400 from ACC_MEM_ID, contains the first STS segment/block.
*        STS2 is at complex sample offset 0x600 from ACC_MEM_ID, contains the second STS segment/block.
*        @endparblock
*@{*/
    typedef enum
    {
        DWT_ACC_IDX_IP_M = 0,  //!< Ipatov preamble CIR index
        DWT_ACC_IDX_STS0_M,    //!< STS1 CIR index (1st half of STS when PDOA mode 3 is used)
        DWT_ACC_IDX_STS1_M,    //!< STS2 CIR index (2nd half of STS when PDOA mode 3 is used)
        NUM_OF_DWT_ACC_IDX     //!< Total number of CIRs
    } dwt_acc_idx_e;
/**@}*/

    /*!  Check whether the accumulator index is for Ipatov (master or slave) */
    #define DWT_ACC_IDX_IS_IPATOV(acc_idx) ((acc_idx) == DWT_ACC_IDX_IP_M)

    /*!  Check whether the accumulator index is for STS (any segment, master or slave) */
    #define DWT_ACC_IDX_IS_STS(acc_idx) (!DWT_ACC_IDX_IS_IPATOV(acc_idx))

/*! @name Dummy enum for compatibility. NOT supported in single receiver QM33xxx/DW3xxx devices.
*   Define CIR segments/blocks (used for reading RX timestamps)
*@{*/
    typedef enum
    {
        DWT_IP_M = 0x0,    //!< Ipatov CIR received with the primary receiver
        DWT_STS0_M = 0x8,  //!< STS0 CIR received with the primary receiver
        DWT_STS1_M = 0x10, //!< STS1 CIR received with the primary receiver
        DWT_STS2_M = 0x18, //!< STS1 CIR received with the primary receiver
        DWT_STS3_M = 0x20, //!< STS1 CIR received with the primary receiver
        DWT_IP_S = 0x28,   //!< Ipatov CIR received with the secondary receiver
        DWT_STS0_S = 0x30, //!< STS0 CIR received with the secondary receiver
        DWT_STS1_S = 0x38, //!< STS1 CIR received with the secondary receiver
        DWT_STS2_S = 0x40, //!< STS2 CIR received with the secondary receiver
        DWT_STS3_S = 0x48, //!< STS3 CIR received with the secondary receiver
        DWT_COMPAT_NONE = 0xFF, //!< Use this with QM33xxx/DW3XXX dwt_uwb_driver.
    } dwt_ip_sts_segment_e;
/**@}*/

/*! @name Defined constants for "mode" bitmask parameter passed into dwt_starttx() function.
*@{*/
    typedef enum
    {
        DWT_START_TX_IMMEDIATE = 0x00, //!< Send the frame immediately
        DWT_START_TX_DELAYED = 0x01,   //!< Send the frame at specified time (time must be less that half period away)
        DWT_RESPONSE_EXPECTED = 0x02,  //!< Will enable the receiver after TX has completed
        DWT_START_TX_DLY_REF = 0x04,   //!< Send the frame at specified time (time in DREF_TIME register + any time in DX_TIME register)
        DWT_START_TX_DLY_RS = 0x08,    //!< Send the frame at specified time (time in RX_TIME_0 register + any time in DX_TIME register)
        DWT_START_TX_DLY_TS = 0x10,    //!< Send the frame at specified time (time in TX_TIME_LO register + any time in DX_TIME register)
        DWT_START_TX_CCA = 0x20,       //!< Send the frame if no preamble detected within PTO time
    } dwt_starttx_mode_e;
/**@}*/

/*! @name Defined constants for "mode" bitmask parameter passed into dwt_rxenable() function.
*@{*/
    typedef enum
    {
        DWT_START_RX_IMMEDIATE = 0x00, //!< Enable the receiver immediately
        DWT_START_RX_DELAYED = 0x01,   //!< Set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
        /*!  If delay RX fails due to "late" error then if this flag is set, the RX will not be re-enabled immediately,
             and device will be in IDLE when function exits. */
        DWT_IDLE_ON_DLY_ERR = 0x02,
        DWT_START_RX_DLY_REF = 0x04, //!< Enable the receiver at specified time (time in DREF_TIME register + any time in DX_TIME register)
        DWT_START_RX_DLY_RS = 0x08,  //!< Enable the receiver at specified time (time in RX_TIME_0 register + any time in DX_TIME register)
        DWT_START_RX_DLY_TS = 0x10,  //!< Enable the receiver at specified time (time in TX_TIME_LO register + any time in DX_TIME register)
    } dwt_startrx_mode_e;
/**@}*/

/*! @name Bit definition of the SYS_ENABLE register
*   exported for dwt_setinterrupt() API
*@{*/
    typedef enum
    {
        DWT_INT_TIMER1_BIT_MASK = 0x80000000UL, //!< TIMER1 expiry
        DWT_INT_TIMER0_BIT_MASK = 0x40000000UL,      //!< TIMER0 expiry
        DWT_INT_ARFE_BIT_MASK = 0x20000000UL,        //!< Frame filtering error
        DWT_INT_CPERR_BIT_MASK = 0x10000000UL,       //!< STS quality warning/error
        DWT_INT_HPDWARN_BIT_MASK = 0x8000000UL,      //!< Half period warning flag when delayed TX/RX is used
        DWT_INT_RXSTO_BIT_MASK = 0x4000000UL,        //!< SFD timeout
        DWT_INT_PLL_HILO_BIT_MASK = 0x2000000UL,     //!< PLL calibration flag
        DWT_INT_RCINIT_BIT_MASK = 0x1000000UL,       //!< Device has entered IDLE_RC
        DWT_INT_SPIRDY_BIT_MASK = 0x800000UL,        //!< SPI ready flag
        DWT_INT_RXPTO_BIT_MASK = 0x200000UL,         //!< Preamble timeout
        DWT_INT_RXOVRR_BIT_MASK = 0x100000UL,        //!< RX overrun event when double RX buffer is used
        DWT_INT_VWARN_BIT_MASK = 0x80000UL,          //!< Brownout event detected
        DWT_INT_CIAERR_BIT_MASK = 0x40000UL,         //!< CIA error
        DWT_INT_RXFTO_BIT_MASK = 0x20000UL,          //!< RX frame wait timeout
        DWT_INT_RXFSL_BIT_MASK = 0x10000UL,          //!< Reed-Solomon error (RX sync loss)
        DWT_INT_RXFCE_BIT_MASK = 0x8000U,            //!< RX frame CRC error
        DWT_INT_RXFCG_BIT_MASK = 0x4000U,            //!< RX frame CRC good
        DWT_INT_RXFR_BIT_MASK = 0x2000U,             //!< RX ended - frame ready
        DWT_INT_RXPHE_BIT_MASK = 0x1000U,            //!< PHY header error
        DWT_INT_RXPHD_BIT_MASK = 0x800U,             //!< PHY header detected
        DWT_INT_CIADONE_BIT_MASK = 0x400U,           //!< CIA done
        DWT_INT_RXSFDD_BIT_MASK = 0x200U,            //!< SFD detected
        DWT_INT_RXPRD_BIT_MASK = 0x100U,             //!< Preamble detected
        DWT_INT_TXFRS_BIT_MASK = 0x80U,              //!< Frame sent
        DWT_INT_TXPHS_BIT_MASK = 0x40U,              //!< Frame PHR sent
        DWT_INT_TXPRS_BIT_MASK = 0x20U,              //!< Frame preamble sent
        DWT_INT_TXFRB_BIT_MASK = 0x10U,              //!< Frame transmission begins
        DWT_INT_AAT_BIT_MASK = 0x8U,                 //!< Automatic ACK transmission pending
        DWT_INT_SPICRCE_BIT_MASK = 0x4U,             //!< SPI CRC error
        DWT_INT_CP_LOCK_BIT_MASK = 0x2U,             //!< PLL locked
        DWT_INT_IRQS_BIT_MASK = 0x1U,                //!< Interrupt set
    } dwt_int_conf_e;
/**@}*/

/*! @name Bit definition of the double RX buffer status events
*@{*/
    typedef enum
    {
        DWT_RDB_STATUS_CP_ERR1_BIT_MASK = 0x80U,  //!< STS quality warning/error in RX buffer 1
        DWT_RDB_STATUS_CIADONE1_BIT_MASK = 0x40U, //!< CIA done for frame in RX buffer 1
        DWT_RDB_STATUS_RXFR1_BIT_MASK = 0x20U,    //!< Frame ready in RX buffer 1
        DWT_RDB_STATUS_RXFCG1_BIT_MASK = 0x10U,   //!< Frame CC good in RX buffer 1
        DWT_RDB_STATUS_CP_ERR0_BIT_MASK = 0x8U,   //!< STS quality warning/error in RX buffer 0
        DWT_RDB_STATUS_CIADONE0_BIT_MASK = 0x4U,  //!< CIA done for frame in RX buffer 0
        DWT_RDB_STATUS_RXFR0_BIT_MASK = 0x2U,     //!< Frame ready in RX buffer 0
        DWT_RDB_STATUS_RXFCG0_BIT_MASK = 0x1U,    //!< Frame CC good in RX buffer 0
    } dwt_rdb_e;
/**@}*/
/*!  RX events mask relating to reception into RX buffer 0, when double buffer is used */
#define DWT_RDB_STATUS_CLEAR_BUFF0_EVENTS (RDB_STATUS_CP_ERR0_BIT_MASK | RDB_STATUS_CIADONE0_BIT_MASK | RDB_STATUS_RXFR0_BIT_MASK | RDB_STATUS_RXFCG0_BIT_MASK)
/*!  RX events mask relating to reception into RX buffer 1, when double buffer is used */
#define DWT_RDB_STATUS_CLEAR_BUFF1_EVENTS (RDB_STATUS_CP_ERR1_BIT_MASK | RDB_STATUS_CIADONE1_BIT_MASK | RDB_STATUS_RXFR1_BIT_MASK | RDB_STATUS_RXFCG1_BIT_MASK)

#define RDB_STATUS_RXOK  ((uint8_t)DWT_RDB_STATUS_RXFCG0_BIT_MASK | (uint8_t)DWT_RDB_STATUS_RXFR0_BIT_MASK | \
                          (uint8_t)DWT_RDB_STATUS_CIADONE0_BIT_MASK | (uint8_t)DWT_RDB_STATUS_CP_ERR0_BIT_MASK | \
                          (uint8_t)DWT_RDB_STATUS_RXFCG1_BIT_MASK | (uint8_t)DWT_RDB_STATUS_RXFR1_BIT_MASK | \
                          (uint8_t)DWT_RDB_STATUS_CIADONE1_BIT_MASK | (uint8_t)DWT_RDB_STATUS_CP_ERR1_BIT_MASK)

/*!  DW3720 double RX buffer interrupt events */
#define DWT_DB_INT_RX           (RDB_STATUS_RXOK)

/*!  UWB IC interrupt events */
#define DWT_INT_RX                                                                                                                                             \
    (DWT_INT_CIADONE_BIT_MASK | DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFR_BIT_MASK | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK      \
        | DWT_INT_RXFTO_BIT_MASK | DWT_INT_CIAERR_BIT_MASK | DWT_INT_RXPTO_BIT_MASK | DWT_INT_RXSTO_BIT_MASK | DWT_INT_ARFE_BIT_MASK)
#define DWT_INT_ALL_LO (0xffffffffUL)
#define DWT_INT_ALL_HI (0xffffffffUL)

/*!  User defined RX timeouts (frame wait timeout and preamble detect timeout) mask. */
#define SYS_STATUS_ALL_RX_TO ((uint32_t)DWT_INT_RXFTO_BIT_MASK | (uint32_t)DWT_INT_RXPTO_BIT_MASK | (uint32_t)DWT_INT_CPERR_BIT_MASK)

/*!  All RX errors mask. */
#define SYS_STATUS_ALL_RX_ERR                                                                                                                                  \
    ((uint32_t)DWT_INT_RXPHE_BIT_MASK | (uint32_t)DWT_INT_RXFCE_BIT_MASK | (uint32_t)DWT_INT_RXFSL_BIT_MASK  | \
     (uint32_t)DWT_INT_RXSTO_BIT_MASK | (uint32_t)DWT_INT_ARFE_BIT_MASK  | (uint32_t)DWT_INT_CIAERR_BIT_MASK | \
     (uint32_t)DWT_INT_CPERR_BIT_MASK)

/*!  All RX events after a correct packet reception mask. */
#define SYS_STATUS_ALL_RX_GOOD                                                                                                                                 \
    ((uint32_t)DWT_INT_RXFR_BIT_MASK | (uint32_t)DWT_INT_RXFCG_BIT_MASK | (uint32_t)DWT_INT_RXPRD_BIT_MASK | (uint32_t)DWT_INT_RXSFDD_BIT_MASK | (uint32_t)DWT_INT_RXPHD_BIT_MASK | (uint32_t)DWT_INT_CIADONE_BIT_MASK)

/*!  All STS Mode 3 RX errors mask. */
#define SYS_STATUS_ALL_ND_RX_ERR     (DWT_INT_CIAERR_BIT_MASK | DWT_INT_RXSTO_BIT_MASK)
#define DWT_INT_HI_CCA_FAIL_BIT_MASK 0x1000U

#define DWT_INT_AES_STS_MEM_CONF_BIT_MASK  0x8U
#define DWT_INT_AES_STS_TRANS_ERR_BIT_MASK 0x4U
#define DWT_INT_AES_STS_AUTH_ERR_BIT_MASK  0x2U

#define DWT_AES_ERRORS (DWT_INT_AES_STS_AUTH_ERR_BIT_MASK | DWT_INT_AES_STS_TRANS_ERR_BIT_MASK | DWT_INT_AES_STS_MEM_CONF_BIT_MASK)


#define RX_BUFFER_MAX_LEN 1023U
#define TX_BUFFER_MAX_LEN 1024U

/*! @name Defined constants and enums for GPIO configuration
*@{*/

/*! @name GPIO configuration enums */
    typedef enum
    {
        // Common for all devices
        GPIO_PIN0_EXTTXE = 0x2 << 0,     //!< @deprecated Only works for DW3000. The pin operates as the EXTTXE output (output TX state)
        GPIO_PIN1_EXTRXE = 0x2 << (1*3), //!< @deprecated Only works for DW3000. The pin operates as the EXTRXE output (output RX state)
        GPIO_PIN2_RXLED  = 0x1 << (2*3), //!< The pin operates as the RXLED output
        GPIO_PIN3_TXLED  = 0x1 << (3*3), //!< The pin operates as the TXLED output
        GPIO_PIN4_EXTDA  = 0x1 << (4*3), //!< @deprecated Only works for DW3000. The pin operates to support external DA/PA
        GPIO_PIN4_EXTTXE = 0x2 << (4*3), //!< @deprecated Only works for DW3720. The pin operates as the EXTTXE output (output TX state)
        GPIO_PIN5_EXTTX  = 0x1 << (5*3), //!< @deprecated Only works for DW3000. The pin operates to support external PA / TX enable
        GPIO_PIN5_EXTRXE = 0x2 << (5*3), //!< @deprecated Only works for DW3720. The pin operates as the EXTRXE output (output RX state)
        GPIO_PIN6_EXTRX  = 0x1 << (6*3), //!< @deprecated Only works for DW3000. The pin operates to support external LNA
        // DW3000
        DW3000_GPIO_PIN0_GPIO        = 0x0,
        DW3000_GPIO_PIN0_RXOKLED     = 0x1,
        DW3000_GPIO_PIN0_PDOA_SW_TX  = 0x2,
        DW3000_GPIO_PIN1_GPIO        = 0x0 << (1*3),
        DW3000_GPIO_PIN1_SFDLED      = 0x1 << (1*3),
        DW3000_GPIO_PIN1_PDOA_SW_RX  = 0x2 << (1*3),
        DW3000_GPIO_PIN2_GPIO        = 0x0 << (2*3),
        DW3000_GPIO_PIN2_RXLED       = 0x1 << (2*3),
        DW3000_GPIO_PIN2_PDOA_SW_RF1 = 0x2 << (2*3),
        DW3000_GPIO_PIN3_GPIO        = 0x0 << (3*3),
        DW3000_GPIO_PIN3_TXLED       = 0x1 << (3*3),
        DW3000_GPIO_PIN3_PDOA_SW_RF2 = 0x2 << (3*3),
        DW3000_GPIO_PIN4_GPIO        = 0x0 << (4*3),
        DW3000_GPIO_PIN4_EXTPA       = 0x1 << (4*3),
        DW3000_GPIO_PIN4_IRQ         = 0x2 << (4*3),
        DW3000_GPIO_PIN5_GPIO        = 0x0 << (5*3),
        DW3000_GPIO_PIN5_EXTTXE      = 0x1 << (5*3),
        DW3000_GPIO_PIN6_GPIO        = 0x0 << (6*3),
        DW3000_GPIO_PIN6_EXTRXE      = 0x1 << (6*3),
        DW3000_GPIO_PIN7_SYNC        = 0x0 << (7*3),
        DW3000_GPIO_PIN7_GPIO        = 0x1 << (7*3),
        DW3000_GPIO_PIN8_IRQ         = 0x0 << (8*3),
        DW3000_GPIO_PIN8_GPIO        = 0x1 << (8*3),
        // DW3720
        DW37XX_GPIO_PIN0_SPI2_CLK    = 0x0,
        DW37XX_GPIO_PIN0_RXOKLED     = 0x1,
        DW37XX_GPIO_PIN0_GPIO        = 0x2,
        DW37XX_GPIO_PIN1_SPI2_MISO   = 0x0 << (1*3),
        DW37XX_GPIO_PIN1_SFDLED      = 0x1 << (1*3),
        DW37XX_GPIO_PIN1_GPIO        = 0x2 << (1*3),
        DW37XX_GPIO_PIN2_IRQ2        = 0x0 << (2*3),
        DW37XX_GPIO_PIN2_RXLED       = 0x1 << (2*3),
        DW37XX_GPIO_PIN2_GPIO        = 0x2 << (2*3),
        DW37XX_GPIO_PIN3_SPI2_MOSI   = 0x0 << (3*3),
        DW37XX_GPIO_PIN3_TXLED       = 0x1 << (3*3),
        DW37XX_GPIO_PIN3_GPIO        = 0x2 << (3*3),
        DW37XX_GPIO_PIN4_GPIO        = 0x0 << (4*3),
        DW37XX_GPIO_PIN4_COEX_IN     = 0x1 << (4*3),
        DW37XX_GPIO_PIN4_PDOA_SW_TX  = 0x2 << (4*3),
        DW37XX_GPIO_PIN5_GPIO        = 0x0 << (5*3),
        DW37XX_GPIO_PIN5_COEX_OUT    = 0x1 << (5*3),
        DW37XX_GPIO_PIN5_PDOA_SW_RX  = 0x2 << (5*3),
        DW37XX_GPIO_PIN6_GPIO        = 0x0 << (6*3),
        DW37XX_GPIO_PIN6_EXT_SW_RX   = 0x1 << (6*3),
        DW37XX_GPIO_PIN6_PDOA_SW_RF1 = 0x2 << (6*3),
        DW37XX_GPIO_PIN7_SYNC        = 0x0 << (7*3),
        DW37XX_GPIO_PIN7_GPIO        = 0x1 << (7*3),
        DW37XX_GPIO_PIN7_PDOA_SW_RF2 = 0x2 << (7*3),
        DW37XX_GPIO_PIN8_IRQ         = 0x0 << (8*3),
        DW37XX_GPIO_PIN8_GPIO        = 0x1 << (8*3)
    } dwt_gpio_pin_e;

    /*!  Mask that can be used in e.g dwt_setgpiomode() to use the GPIO mode of all GPIO pins on a DW3000 IC */
    #define DW3000_ENABLE_ALL_GPIOS_MASK  0x1200000
    /*!  Mask that can be used in e.g dwt_setgpiomode() to use the GPIO mode of all GPIO pins on a DW3720 IC*/
    #define DW37XX_ENABLE_ALL_GPIOS_MASK  0x1200492

    #define GPIO_MFIO_MODE_MASK 0x7UL
/*! GPIO MFIO configuration mask enums */
    typedef enum
    {
        GPIO0_FUNC_MASK = 0x0000007,
        GPIO1_FUNC_MASK = 0x0000038,
        GPIO2_FUNC_MASK = 0x00001c0,
        GPIO3_FUNC_MASK = 0x0000e00,
        GPIO4_FUNC_MASK = 0x0007000,
        GPIO5_FUNC_MASK = 0x0038000,
        GPIO6_FUNC_MASK = 0x01c0000,
        GPIO7_FUNC_MASK = 0x0e00000,
        GPIO8_FUNC_MASK = 0x7000000,
    } dwt_gpio_func_mask_e;

/*! Mask values for GPIO pins on DW3xxx enums */
    typedef enum
    {
        GPIO0_BIT_MASK = 0x001,
        GPIO1_BIT_MASK = 0x002,
        GPIO2_BIT_MASK = 0x004,
        GPIO3_BIT_MASK = 0x008,
        GPIO4_BIT_MASK = 0x010,
        GPIO5_BIT_MASK = 0x020,
        GPIO6_BIT_MASK = 0x040,
        GPIO7_BIT_MASK = 0x080,
        GPIO8_BIT_MASK = 0x100,
    } dwt_gpio_mask_e;

    #define GPIO_BIT_MASK_ALL 0x1FF
/*! Mask values for GPIO pins.
*   @deprecated Use @ref dwt_gpio_mask_e instead
*/
    typedef enum
    {
        GPIO_0 = GPIO0_BIT_MASK,
        GPIO_1 = GPIO1_BIT_MASK,
        GPIO_2 = GPIO2_BIT_MASK,
        GPIO_3 = GPIO3_BIT_MASK,
        GPIO_4 = GPIO4_BIT_MASK,
        GPIO_5 = GPIO5_BIT_MASK,
        GPIO_6 = GPIO6_BIT_MASK,
        GPIO_7 = GPIO7_BIT_MASK,
        GPIO_8 = GPIO8_BIT_MASK,
        GPIO_ALL = GPIO_BIT_MASK_ALL,
    } gpio_num_e;

/*! Defined constants for "lna_pa" bit field parameter passed to dwt_setlnapamode() function */
    typedef enum
    {
        DWT_LNA_PA_DISABLE = 0x00,
        DWT_LNA_ENABLE = 0x01,
        DWT_PA_ENABLE = 0x02,
        DWT_TXRX_EN = 0x04,
    } dwt_setlnapmodes_e;

/*! Defined constants for "mode" bit field parameter passed to dwt_setleds() function. */
    typedef enum
    {
        DWT_LEDS_DISABLE = 0x00,
        DWT_LEDS_ENABLE = 0x01,
        DWT_LEDS_INIT_BLINK = 0x02,
        DWT_LEDS_BLINK_TIME_DEF = 0x10, //!< Default blink time. Blink time is expressed in multiples of 14 ms. The value defined here (0x10) is ~225 ms.
    } dwt_setleds_mode_e;

/**@}*/

/*!
 * Defines used with dwt_configure_and_set_antenna_selection_gpio() API to configure antenna selection on specific modules
*@{*/
#define ANT_GPIO6_POS_MASK   0x1U
#define ANT_GPIO6_POS_OFFSET (0U)
#define ANT_GPIO6_VAL_MASK   0x2U
#define ANT_GPIO6_VAL_OFFSET (1U)
#define ANT_GPIO7_POS_MASK   0x4U
#define ANT_GPIO7_POS_OFFSET (2U)
#define ANT_GPIO7_VAL_MASK   0x8U
#define ANT_GPIO7_VAL_OFFSET (3U)
/**@}*/

/*! @name Defines for the WiFi co-existence configuration
 * These can be used when UWB radio transceiver is used in a system which also has a WiFi radio. The co-existence GPIOs can be used to 
 * notify WiFi radio that UWB radio will turning on so that WiFi can be turned off. 
*@{*/
#define COEX_TIME_US        1000UL                   //!< Time in us to toggle the GPIO prior to UWB operation (e.g., TX)
#define COEX_TIME_DTU       US_TO_DTU(COEX_TIME_US)  //!< Time in device time units to toggle the GPIO prior to UWB operation (e.g., TX)
#define COEX_MARGIN_US      20UL                     //!< Margin to account for GPIO toggle
#define COEX_MARGIN_DTU     US_TO_DTU(COEX_MARGIN_US)//!< Margin in device time units to account for GPIO toggle

/*! Enable/disable WiFi co-existence */
    typedef enum
    {
        DWT_EN_WIFI_COEX = 0, //!< Configure GPIO for WiFi co-ex - GPIO high
        DWT_DIS_WIFI_COEX     //!< Configure GPIO for WiFi co-ex - GPIO low
    } dwt_wifi_coex_e;

/**@}*/

/*! @name Defines for the DW Timers configuration
*@{*/

/*! Timers enums
    There are two instances of the DW Timer */
    typedef enum
    {
        DWT_TIMER0 = 0,
        DWT_TIMER1
    } dwt_timers_e;

/*! Timer modes
    Timer support single or repeating mode*/
    typedef enum
    {
        DWT_TIM_SINGLE = 0,
        DWT_TIM_REPEAT
    } dwt_timer_mode_e;

/*! Timer periods */
    typedef enum
    {
        DWT_XTAL = 0,       //!< 38.4 MHz
        DWT_XTAL_DIV2 = 1,  //!< 19.2 MHz
        DWT_XTAL_DIV4 = 2,  //!< 9.6 MHz
        DWT_XTAL_DIV8 = 3,  //!< 4.8 MHz
        DWT_XTAL_DIV16 = 4, //!< 2.4 MHz
        DWT_XTAL_DIV32 = 5, //!< 1.2 MHz
        DWT_XTAL_DIV64 = 6, //!< 0.6 MHz
        DWT_XTAL_DIV128 = 7 //!< 0.3 MHz
    } dwt_timer_period_e;

/*! Timer configuration structure */
    typedef struct
    {
        dwt_timers_e timer;           //!< Select the timer to use.
        dwt_timer_period_e timer_div; //!< Select the timer frequency (divider).
        dwt_timer_mode_e timer_mode;  //!< Select the timer mode.
        uint8_t timer_gpio_stop;      //!< Set to '1' to halt GPIO on interrupt.
        uint8_t timer_coexout;        //!< Configure GPIO for WiFi co-ex.
    } dwt_timer_cfg_t;
/**@}*/

/*! @name Frame filtering and LE configuration options
*@{*/

/*! @name Frame filtering configuration options */
    typedef enum
    {
        DWT_FF_ENABLE_802_15_4 = 0x2, //!< Enable FF for 802.15.4
        DWT_FF_DISABLE = 0x0,         //!< Disable FF
        DWT_FF_BEACON_EN = 0x001,     //!< Beacon frames allowed
        DWT_FF_DATA_EN = 0x002,       //!< Data frames allowed
        DWT_FF_ACK_EN = 0x004,        //!< ACK frames allowed
        DWT_FF_MAC_EN = 0x008,        //!< MAC control frames allowed
        DWT_FF_RSVD_EN = 0x010,       //!< Reserved frame types allowed
        DWT_FF_MULTI_EN = 0x020,      //!< Multipurpose frames allowed
        DWT_FF_FRAG_EN = 0x040,       //!< Fragmented frame types allowed
        DWT_FF_EXTEND_EN = 0x080,     //!< Extended frame types allowed
        DWT_FF_COORD_EN = 0x100,      //!< Behave as coordinator (can receive frames with no dest address (PAN ID has to match))
        DWT_FF_IMPBRCAST_EN = 0x200,  //!< Allow MAC implicit broadcast
        DWT_FF_MAC_LE0_EN = 0x408,    //!< MAC frames allowed if address in LE0_PEND matches source address
        DWT_FF_MAC_LE1_EN = 0x808,    //!< MAC frames allowed if address in LE1_PEND matches source address
        DWT_FF_MAC_LE2_EN = 0x1008,   //!< MAC frames allowed if address in LE2_PEND matches source address
        DWT_FF_MAC_LE3_EN = 0x2008,   //!< MAC frames allowed if address in LE3_PEND matches source address
    } dwt_ff_conf_options_e;


/*! Low Energy (LE) device addresses, can support up to 4 LE devices */
    typedef enum
    {
        LE0 = 0, //!< LE0_PEND address
        LE1 = 1, //!< LE1_PEND address
        LE2 = 2, //!< LE2_PEND address
        LE3 = 3, //!< LE3_PEND address
    } dwt_le_addresses_e;
/**@}*/

/*! @name DW3xxx SLEEP and WAKEUP configuration options
*@{*/
/*! @name On-wake configuration options */
    typedef enum
    {
        DWT_PGFCAL = 0x0800, //!< !!! Should not be used for optimal performance !!!
        DWT_GOTORX = 0x0200,
        DWT_GOTOIDLE = 0x0100,
        DWT_SEL_OPS3 = 0x00C0,
        DWT_SEL_OPS2 = 0x0080, //!< Short OPS table
        DWT_SEL_OPS1 = 0x0040, //!< SCP OPS table
        DWT_SEL_OPS0 = 0x0000, //!< Long OPS table
        DWT_ALT_OPS = 0x0020,
        DWT_LOADLDO = 0x0010,
        DWT_LOADDGC = 0x0008,
        DWT_LOADBIAS = 0x0004,
        DWT_RUNSAR = 0x0002,
        DWT_CONFIG = 0x0001, //!< Download the AON array into the device registers (configuration download)
    } dwt_on_wake_param_e;

/*! Wakeup configuration options */
    typedef enum
    {
        DWT_PRES_SLEEP = 0x20, //!< Allows for SLEEP_EN bit to be "preserved", although it will self - clear on wake up
        DWT_WAKE_WUP = 0x10,   //!< Wake up on WAKEUP PIN
        DWT_WAKE_CSN = 0x8,    //!< Wake up on chip select
        DWT_BROUT_EN = 0x4,    //!< Enable brownout detector during sleep/deep sleep
        DWT_SLEEP = 0x2,       //!< Enable sleep (if this bit is clear the device will enter deep sleep)
        DWT_SLP_EN = 0x1,      //!< Enable sleep/deep sleep functionality
    } dwt_wkup_param_e;

/*! Events that can be used to automatically transition to SLEEP or DEEPSLEEP */
    typedef enum
    {
        DWT_TX_COMPLETE = 0x01,
        DWT_RX_COMPLETE = 0x02
    } dwt_sleep_after_param_e;

/*! QM33xxx/DW3xxx AON sleep counter configurations */
    typedef enum
    {
        AON_SLPCNT_LO = (0x102),       //!< address of SLEEP counter bits [19:12] in AON memory
        AON_SLPCNT_HI = (0x103),       //!< address of SLEEP counter bits [27:20] in AON memory
        AON_SLPCNT_CAL_CTRL = (0x104), //!< address of SLEEP counter cal control
        AON_LPOSC_TRIM = (0x10B),      //!< address of LP OSC trim code
        AON_VDD_DIG    = (0x10C),      //!< address of VDD DIG configuration
        AON_SLPCNT_CAL_LO = (0x10E),   //!< address of SLEEP counter cal value low byte
        AON_SLPCNT_CAL_HI = (0x10F),   //!< address of SLEEP counter cal value high byte
    } dwt_aon_sleep_conf_e;

/*! Enum used for selecting channel for DGC on-wake kick */
    typedef enum
    {
        DWT_DGC_SEL_CH5 = 0,
        DWT_DGC_SEL_CH9
    } dwt_dgc_chan_sel;

/*! Enum used for selecting location to load DGC data from */
    typedef enum
    {
        DWT_DGC_LOAD_FROM_SW = 0,
        DWT_DGC_LOAD_FROM_OTP
    } dwt_dgc_load_location;

/*! Enum used for selecting calibration and restoration after wakeup */
    typedef enum
    {
        DWT_FAST_RESTORE = 0U,             /*!< @deprecated Do not use this flag in new design. Will be 
                                                    removed in the future.
                                                For faster wakeups, no calibration is done and DGC values
                                                are not updated. It should be used only if dwt_configure
                                                will be called after wakeup. */
        DWT_STANDARD_RESTORE = 1U,         /*!< @deprecated Do not use this flag in new design. Will be 
                                                    removed in the future.
                                                Should be the default restore method.
                                                It restores ADC threshold values instead of re-calibrating.
                                                ADC cal can be requested with DWT_FORCE_ADCOFFSET_CAL.
                                                PGF calibration is always performed in this mode. */
        DWT_FORCE_ADCOFFSET_CAL = 2U,      //!< Force ADC calibration. This is only used in DW3720
        DWT_RESTORE_RX_ONLY_MODE = 0x04U,   //!< Restore RX only mode
        DWT_RESTORE_TX_ONLY_MODE = 0x08U,   //!< Restore TX only mode
        DWT_RESTORE_TXRX_MODE = 0x0CU       //!< Restore TX and RX mode
    } dwt_restore_type_e;
/**@}*/

/*! @name QM33xxx/DW3xxx IDLE/INIT mode definitions
*@{*/
    typedef enum
    {
        DWT_DW_INIT = 0x0, //!< UWB radio is in INIT state (PLL is off, system clock is FOSC/4)
        DWT_DW_IDLE = 0x1, //!< UWB radio is in IDLE/IDLE_PLL state (PLL is enabled and running)
        DWT_DW_IDLE_RC = 0x2, //!< UWB radio is in INIT state (PLL is off, system clock is FOSC)
    } dwt_idle_init_modes_e;
/**@}*/

/*! @name DW3xxx OTP read/load options
    @deprecated These optptions are deprecated and should not be used in new designs. Use the "Initialisation mode bits" below instead.
*@{*/
    typedef enum
    {
        DWT_READ_OTP_PID = 0x01, //!< Read part ID from OTP
        DWT_READ_OTP_LID = 0x02, //!< Read lot ID from OTP
        DWT_READ_OTP_BAT = 0x04, //!< Read reference voltage from OTP
        DWT_READ_OTP_TMP = 0x08, //!< Read reference temperature from OTP
    } dwt_read_otp_modes_e;
/**@}*/

/*! @name Initialisation mode bits
 *
 * These bits are to be used with dwt_initialise().
 * @{
 */
#define DWT_READ_OTP_ALL 0x00 //!< read the part ID, lot ID, ref voltage and temperature from the OTP
#define DWT_READ_OTP_PLID_DIS  0x10 //!< don't read the part ID or the lot ID from the OTP (neither SIP/SOC status)
#define DWT_READ_OTP_VTBAT_DIS 0x40 //!< don't read the ref voltage from the OTP
#define DWT_READ_OTP_TMP_DIS   0x80 //!< don't read the ref temperature from the OTP
 /**@}*/

/*! @name Reset options
*@{*/
    typedef enum
    {
        DWT_RESET_ALL = 0x00,
        DWT_RESET_CTRX = 0x0F,
        DWT_RESET_RX = 0xEF,
        DWT_RESET_CLEAR = 0xFF,
    } dwt_reset_options_e;
/**@}*/

 /*! @name RF port control enums for enabling manual control of antenna/RF port selection
 *@{
 */
    typedef enum
    {
	DWT_RF_PORT_MANUAL_DISABLED = 0UL,
        /* Force RF Port 1. PDoA is not possible in that mode. */
        DWT_RF_PORT_MANUAL_1 = 1UL,
        /* Force RF Port 2. PDoA is not possible in that mode. */
        DWT_RF_PORT_MANUAL_2 = 2UL,
	/* The RF port is automatically switched depending on PDoA mode, starting by RF Port 1. */
        DWT_RF_PORT_AUTO_1_2 = 3UL,
        /* The RF port is automatically switched depending on PDoA mode, starting by RF Port 2. */
        DWT_RF_PORT_AUTO_2_1 = 4UL,
    } dwt_rf_port_ctrl_e;
/**@}*/

#define TEMP_INIT -127
#define DEFAULT_XTAL_TRIM_TEMP 25

 /*! @name Defines for freq. to ppm conversion
 *@{
 */
/*!  Conversion factor to convert clock offset from PPM to ratio */
#define CLOCK_OFFSET_PPM_TO_RATIO (1.0 / (1 << 26))

/*! Multiplication factor to convert carrier integrator value to a frequency offset in Hertz */
#define FREQ_OFFSET_MULTIPLIER (998.4e6 / 2.0 / 1024.0 / 131072.0)

/*! Multiplication factor for channel 5 to convert frequency offset in Hertz to PPM crystal offset */
/*!  @note This will also change the sign, a positive value means the local RX clock is running slower than the remote TX device. */
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_5 (-1.0e6 / 6489.6e6)
/*! Multiplication factor for channel 9 to convert frequency offset in Hertz to PPM crystal offset */
/*!  @note This will also change the sign, a positive value means the local RX clock is running slower than the remote TX device. */
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_9 (-1.0e6 / 7987.2e6)
/**@}*/

#define DWT_VALID_TDOA_LIMIT (100) //!< If the abs(TDoA) value is larger than this constant, it means the PDoA will not be valid, and should not be used.

/*! @name RX call-back flags, set in the call-back data when frame is received
*@{*/
    typedef enum
    {
        DWT_CB_DATA_RX_FLAG_RNG = 0x01,  //!< Ranging bit was set
        DWT_CB_DATA_RX_FLAG_ND = 0x02,   //!< No data mode
        DWT_CB_DATA_RX_FLAG_CIA = 0x04,  //!< CIA done
        DWT_CB_DATA_RX_FLAG_CER = 0x08,  //!< CIA error
        DWT_CB_DATA_RX_FLAG_CPER = 0x10, //!< STS error
    } dwt_cb_data_rx_flags_e;
/**@}*/

/*! @name TX/RX call-back data structure
*@{*/
    typedef struct
    {
        uint32_t status;     //!< Initial value of register as ISR is entered
        uint16_t status_hi;  //!< Initial value of register as ISR is entered, if relevant for that event type
        uint16_t datalength; //!< Length of frame
        uint8_t  rx_flags;   //!< RX frame flags, see above
        uint8_t  dss_stat;   //!< Dual SPI status reg 11:38, 2 LSbits relevant : bit0 (DWT_CB_DSS_SPI1_AVAIL) and bit1 (DWT_CB_DSS_SPI2_AVAIL)
        struct dwchip_s *dw;
    } dwt_cb_data_t;
/**@}*/

    /*! Call-back type for SPI read error event (if the UWB IC generated CRC does not match the one calculated by the dwt_generatecrc8 function) */
    typedef void (*dwt_spierrcb_t)(void);

    /*! Call-back type for all interrupt events */
    typedef void (*dwt_cb_t)(const dwt_cb_data_t *cb_data);

/*! @name Call-back functions
*@{*/
    typedef struct
    {
        dwt_cb_t cbTxDone;         //!< Callback for TX confirmation event
        dwt_cb_t cbRxOk;           //!< Callback for RX good frame event
        dwt_cb_t cbRxTo;           //!< Callback for RX timeout events
        dwt_cb_t cbRxErr;          //!< Callback for RX error events
        //!<  DWT/QM33 needs
        dwt_cb_t cbSPIErr;         //!< Callback for SPI error events
    	dwt_spierrcb_t cbSPIRDErr; //!< Callback for SPI read error events
        dwt_cb_t cbSPIRdy;         //!< Callback for SPI ready events
        dwt_cb_t cbDualSPIEv;      //!< Callback for dual SPI events
        //!<  QM35/QPF needs
        dwt_cb_t cbFrmRdy;         //!< Callback for RX frame ready events
        dwt_cb_t cbCiaDone;        //!< Callback for RX CIA processing done events
        dwt_cb_t devErr;           //!< Callback for device error events such as: PGF calibration error
        dwt_cb_t cbSysEvent;       //!< Callback for UWB ready, timer or other system events
    } dwt_callbacks_s;
/**@}*/

/*! @name ISR configuration flags
*         Use with dwt_configureisr().
*@{*/
    typedef enum
    {
        DWT_LEN0_RXGOOD   = 0x1, //!< Treat 0-length packets as good RX
    } dwt_isr_flags_e;
/**@}*/

/*! @name This enum holds interrupt events configuration options.
*@{*/
    typedef enum
    {
        DWT_DISABLE_INT = 0,          //!<  Disable these INT
        DWT_ENABLE_INT,               //!<  Enable these INT
        DWT_ENABLE_INT_ONLY,          //!<  Enable only these INT
        DWT_ENABLE_INT_DUAL_SPI,      //!<  Enable these INT, dual SPI mode
        DWT_ENABLE_INT_ONLY_DUAL_SPI  //!<  Enable only these INT, dual SPI mode
    } dwt_INT_options_e;
/**@}*/

#define SQRT_FACTOR       181UL //!< Factor of sqrt(2) for calculation
#define STS_LEN_SUPPORTED 8U    //!< The supported STS length options
#define SQRT_SHIFT_VAL    7UL
#define SHIFT_VALUE       11UL
#define MOD_VALUE         2048UL
#define HALF_MOD          (MOD_VALUE >> 1UL)


/*! @name These enums are used for STS length configuration
*@{*/
    typedef enum
    {
        DWT_STS_LEN_16 = 1, /*!< STS length 16 is not recommended */
        DWT_STS_LEN_32 = 3,
        DWT_STS_LEN_64 = 7,
        DWT_STS_LEN_128 = 15,
        DWT_STS_LEN_256 = 31,
        DWT_STS_LEN_512 = 63,
        DWT_STS_LEN_1024 = 127,
        DWT_STS_LEN_2048 = 255
    } dwt_sts_lengths_e;
/**@}*/

#define GET_STS_LEN_IDX(sts_len) (__builtin_ctz((uint16_t)(sts_len) + 1U) - 1) /*!< It is used to find the index of the array sts_length_factors */


/*! @name Structure typedef: dwt_config_t
 *
 * Structure for setting device configuration via dwt_configure() function
 *
*@{*/
    typedef struct
    {
        uint8_t chan;                 //!< Channel number (5 or 9)
        uint16_t txPreambLength;      //!< DWT_PLEN_32..DWT_PLEN_4096
        dwt_pac_size_e rxPAC;         //!< Acquisition Chunk Size (Relates to RX preamble length)
        uint8_t txCode;               //!< TX preamble code (the code configures the PRF, e.g., 9 -> PRF of 64 MHz)
        uint8_t rxCode;               //!< RX preamble code (the code configures the PRF, e.g., 9 -> PRF of 64 MHz)
        dwt_sfd_type_e sfdType;       //!< SFD type (0 for short IEEE 8-bit standard, 1 for DW 8-bit, 2 for DW 16-bit, 3 for 4z BPRF)
        dwt_uwb_bit_rate_e dataRate;  //!< Data rate {DWT_BR_850K or DWT_BR_6M8}
        dwt_phr_mode_e phrMode;       //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
        dwt_phr_rate_e phrRate;       //!< PHR rate {0x0 - standard DWT_PHRRATE_STD, 0x1 - at datarate DWT_PHRRATE_DTA}
        uint16_t sfdTO;               //!< SFD timeout value (in symbols)
        dwt_sts_mode_e stsMode;       //!< STS mode (no STS, STS before PHR or STS after data)
        dwt_sts_lengths_e stsLength;  //!< STS length (the allowed values are listed in dwt_sts_lengths_e
        dwt_pdoa_mode_e pdoaMode;     //!< PDOA mode
#ifndef WIN32
    } __attribute__((packed)) dwt_config_t;
#else
    } dwt_config_t;
#endif // WIN32
/**@}*/

/*! @name Structure typedef: dwt_txconfig_t
 *
 * Structure for setting device TX configuration via dwt_configuretxrf() function
 *
*@{*/
    typedef struct
    {
        uint8_t PGdly;
        /*! TX POWER
        * 31:24     TX_CP_PWR
        * 23:16     TX_SHR_PWR
        * 15:8      TX_PHR_PWR
        * 7:0       TX_DATA_PWR
        */
        uint32_t power;
        uint16_t PGcount;
#ifndef WIN32
    } __attribute__((packed)) dwt_txconfig_t;
#else
    } dwt_txconfig_t;
#endif // WIN32
/**@}*/

/*! @name TDOA and PDOA results
 *
*@{*/
    typedef struct
    {
        int16_t tdoa;
        int16_t pdoa;
        int8_t fp_ok;
#ifndef WIN32
    } __attribute__((packed)) dwt_pdoa_tdoa_res_t;
#else
    } dwt_pdoa_tdoa_res_t;
#endif // WIN32
/**@}*/

/*! @name RX diagnostics
 *
*@{*/
    typedef struct
    {
        uint8_t ipatovRxTime[5]; //!< RX timestamp from Ipatov sequence
        uint8_t ipatovRxStatus;  //!< RX status info for Ipatov sequence
        uint16_t ipatovPOA;      //!< POA of Ipatov

        uint8_t stsRxTime[5];  //!< RX timestamp from STS
        uint16_t stsRxStatus;  //!< RX status info for STS
        uint16_t stsPOA;       //!< POA of STS block 1
        uint8_t sts2RxTime[5]; //!< RX timestamp from STS
        uint16_t sts2RxStatus; //!< RX status info for STS
        uint16_t sts2POA;      //!< POA of STS block 2

        uint8_t tdoa[6]; //!< TDOA from two STS RX timestamps
        int16_t pdoa;    //!< PDOA from two STS POAs signed int [1:-11] in radians

        int16_t xtalOffset; //!< Estimated crystal offset of remote device
        uint32_t ciaDiag1;  //!< Diagnostics common to both sequences

        uint32_t ipatovPeak;       //!< Index and amplitude of peak sample in Ipatov sequence CIR
        uint32_t ipatovPower;      //!< Channel area allows estimation of channel power for the Ipatov sequence
        uint32_t ipatovF1;         //!< F1 for Ipatov sequence
        uint32_t ipatovF2;         //!< F2 for Ipatov sequence
        uint32_t ipatovF3;         //!< F3 for Ipatov sequence
        uint16_t ipatovFpIndex;    //!< First path index for Ipatov sequence
        uint16_t ipatovAccumCount; //!< Number accumulated symbols for Ipatov sequence

        uint32_t stsPeak;       //!< Index and amplitude of peak sample in STS CIR
        uint32_t stsPower;      //!< Channel area allows estimation of channel power for the STS
        uint32_t stsF1;         //!< F1 for STS
        uint32_t stsF2;         //!< F2 for STS
        uint32_t stsF3;         //!< F3 for STS
        uint16_t stsFpIndex;    //!< First path index for STS
        uint16_t stsAccumCount; //!< Number accumulated symbols for STS

        uint32_t sts2Peak;       //!< Index and amplitude of peak sample in STS CIR
        uint32_t sts2Power;      //!< Channel area allows estimation of channel power for the STS
        uint32_t sts2F1;         //!< F1 for STS
        uint32_t sts2F2;         //!< F2 for STS
        uint32_t sts2F3;         //!< F3 for STS
        uint16_t sts2FpIndex;    //!< First path index for STS
        uint16_t sts2AccumCount; //!< Number accumulated symbols for STS

#ifndef WIN32
    } __attribute__((packed)) dwt_rxdiag_t;
#else
} dwt_rxdiag_t;
#endif // WIN32
/**@}*/

/*! @name CIR diagnostics
 *
*@{*/
    typedef struct
    {
        uint32_t power;      //!< Channel area allows estimation of channel power for the CIR sequence, [30:0].
        uint32_t F1;         //!< F1 for the CIR sequence, [21:0].
        uint32_t F2;         //!< F2 for the CIR sequence, [21:0].
        uint32_t F3;         //!< F3 for the CIR sequence, [21:0].
        uint32_t peakAmp;    //!< Amplitude of peak sample in the CIR (Q20.2 format)
        uint16_t peakIndex;  //!< Index of peak sample in the CIR
        uint16_t FpIndex;    //!< First path index for the CIR (Q10.6 format).
        uint16_t accumCount; //!< Number accumulated symbols for the CIR
        uint16_t EFpIndex;     //!< Early First path index (Q10.6 format).
        uint8_t EFpConfLevel; //!< Early First path Confidence level. (Q0.4 format).
        uint32_t FpThreshold; //!< Threshold used when calculating the location of the first path in the CIR.

#ifndef WIN32
    } __attribute__((packed)) dwt_cirdiags_t;
#else
} dwt_cirdiags_t;
#endif // WIN32
/**@}*/

/*! @name UWB IC NLOS DIAGNOSTIC TYPE
 *
*@{*/
    typedef enum
    {
        IPATOV = 0x0, //!< Select Ipatov Diagnostic
        STS1 = 0x1,   //!< Select STS1 Diagnostic
        STS2 = 0x2,   //!< Select STS2 Diagnostic
    } dwt_diag_type_e;
/**@}*/

/*! @name CIA diagnostics logging options
*@{*/
    typedef enum
    {
        DW_CIA_DIAG_LOG_MAX = 0x8, //!< CIA to copy to swinging set a maximum set of diagnostic registers in Double Buffer mode
        DW_CIA_DIAG_LOG_MID = 0x4, //!< CIA to copy to swinging set a medium set of diagnostic registers in Double Buffer mode
        DW_CIA_DIAG_LOG_MIN = 0x2, //!< CIA to copy to swinging set a minimal set of diagnostic registers in Double Buffer mode
        DW_CIA_DIAG_LOG_ALL = 0x1, //!< CIA to log all diagnostic registers
        DW_CIA_DIAG_LOG_OFF = 0x0, //!< CIA to log reduced set of diagnostic registers
    } dwt_cia_diag_log_conf_e;
/**@}*/

/*! Accumulator sizes (in the number of complex samples) */
#define DWT_CIR_LEN_STS      512  //!< Max number of STS CIR samples
#define DWT_CIR_LEN_IP_PRF16 992  //!< Max number of Ipatov CIR samples with PRF16
#define DWT_CIR_LEN_IP_PRF64 1016 //!< Max number of Ipatov CIR samples with PRF64
#define DWT_CIR_LEN_MAX      DWT_CIR_LEN_IP_PRF64 //!< Max number of CIR samples

#define PCODE_PRF16_START 1U
#define PCODE_PRF64_START 9U
#define PCODE_PRF64_END 24U

/*! Important definitions for CIR reading algorithm */
#define DWT_CIR_VALUE_NO_SIGN_18BIT_MASK 0x0003FFFFUL
#define DWT_CIR_SIGN_24BIT_EXTEND_32BIT_MASK 0xFFFC0000UL
/*! Read out by chunks of up to 16 complex samples i.e 16*(24bits+24bits) = 16*48 bytes */
#define CHUNK_CIR_NB_SAMP 16U

/*! @name This defines the CIR read mode (complex sample size)
 *
*@{*/
    typedef enum {
        DWT_CIR_READ_FULL = 0, //!< Full 48-bit complex samples
        DWT_CIR_READ_LO   = 1, //!< Reduced 32-bit complex samples: bits [15:0] for real/imag parts
        DWT_CIR_READ_MID  = 2, //!< Reduced 32-bit complex samples: bits [16:1] for real/imag parts
        DWT_CIR_READ_HI   = 3, //!< Reduced 32-bit complex samples: bits [17:2] for real/imag parts
    } dwt_cir_read_mode_e;
/**@}*/

/*! @name NLOS structs
 *
*@{*/
    typedef struct
    {
        uint32_t accumCount;          //!< The number of preamble symbols accumulated, or accumulated STS length.
        uint32_t F1;                  //!< The first path amplitude (point 1) magnitude value.
        uint32_t F2;                  //!< The first path amplitude (point 2) magnitude value.
        uint32_t F3;                  //!< The first path amplitude (point 3) magnitude value.
        uint32_t cir_power;           //!< The Channel Impulse Response power value.
        uint8_t D;                    //!< The DGC_DECISION, treated as an unsigned integer in range 0 to 7.
        dwt_diag_type_e diag_type;
    } dwt_nlos_alldiag_t;
/**@}*/

/*! @name NLOS FP and PP diags structs
 *
*@{*/
    typedef struct
    {
        uint32_t index_fp_u32;  //!< the first path index.
        uint32_t index_pp_u32;  //!< the peak path index
    } dwt_nlos_ipdiag_t;
/**@}*/

/*! @name DW3720 - enable/disable equaliser in the CIA
 *
*@{*/
    typedef enum
    {
        DWT_EQ_DISABLED = 0x0,
        DWT_EQ_ENABLED = 0x1,
    } dwt_eq_config_e;
/**@}*/

/*! @name Event counters
 *
*@{*/
    typedef struct
    {
        //!< All of the below are mapped to registers in DW3xxx
        uint16_t PHE;   //!< 12-bit number of received header error events
        uint16_t RSL;   //!< 12-bit number of received frame sync loss event events
        uint16_t CRCG;  //!< 12-bit number of good CRC received frame events
        uint16_t CRCB;  //!< 12-bit number of bad CRC (CRC error) received frame events
        uint8_t ARFE;   //!< 8-bit number of address filter error events
        uint8_t OVER;   //!< 8-bit number of receive buffer overrun events (used in double buffer mode)
        uint16_t SFDTO; //!< 12-bit number of SFD timeout events
        uint16_t PTO;   //!< 12-bit number of Preamble timeout events
        uint8_t RTO;    //!< 8-bit number of RX frame wait timeout events
        uint16_t TXF;   //!< 12-bit number of transmitted frame events
        uint8_t HPW;    //!< 8-bit half period warning events (when delayed RX/TX enable is used)
        uint8_t CRCE;   //!< 8-bit SPI CRC error events
        uint16_t PREJ;  //!< 12-bit number of Preamble rejection events
        uint16_t SFDD;  //!< 12-bit SFD detection events ... only DW3720
        uint8_t STSE;   //!< 8-bit STS error/warning events
#ifndef WIN32
    } __attribute__((packed)) dwt_deviceentcnts_t;
#else
} dwt_deviceentcnts_t;
#endif // WIN32
/**@}*/

    /* BEGIN: CHIP_SPECIFIC_SECTION DW3720 */

/*! @name Hosts for the SPI bus
 *
*@{*/
    typedef enum
    {
        DWT_HOST_SPI1 = 0, //!< Host using SPI1 interface
        DWT_HOST_SPI2      //!< Host using SPI2 interface
    } dwt_spi_host_e;
/**@}*/

#define SPI2MAVAIL_BIT_MASK 0x2 //!< bit 1 of 1a:01
#define SPI1MAVAIL_BIT_MASK 0x4 //!< bit 2 of 1a:01
/*! @name Hosts sleep configuration
 *
*@{*/
    typedef enum
    {
        HOST_EN_SLEEP = 0x00, //!< Host enable Sleep/Deepsleep
        HOST_DIS_SLEEP = 0x60 //!< Host disable Sleep/Deepsleep
    } dwt_host_sleep_en_e;
/**@}*/

    /* END: CHIP_SPECIFIC_SECTION DW3720 */

    /********************************************************************************************************************/
    /*                                                AES BLOCK                                                         */
    /********************************************************************************************************************/

    /*! enums below are defined in such a way as to allow a simple write to UWB IC AES configuration registers */

/*! @name MIC size definition
 *
*@{*/
    typedef enum
    {
        MIC_0 = 0,
        MIC_4,
        MIC_6,
        MIC_8,
        MIC_10,
        MIC_12,
        MIC_14,
        MIC_16
    } dwt_mic_size_e;
/**@}*/

/*! @name Key size definition
 *
*@{*/
    typedef enum
    {
        AES_KEY_128bit = 0,
        AES_KEY_192bit = 1,
        AES_KEY_256bit = 2
    } dwt_aes_key_size_e;
/**@}*/

/*! @name Load key from RAM selection
 *
*@{*/
    typedef enum
    {
        AES_KEY_No_Load = 0,
        AES_KEY_Load
    } dwt_aes_key_load_e;
/**@}*/

/*! @name AES key source, can be in RAM or registers
 *
*@{*/
    typedef enum
    {
        AES_KEY_Src_Register = 0, /*!< Use AES KEY from registers */
        AES_KEY_Src_RAMorOTP      /*! Use AES KEY from RAM or OTP (depending if AES_key_OTP set),
                                         AES_KEY_Load needs to be set as well */
    } dwt_aes_key_src_e;
/**@}*/

/*! @name AES encryption or decryption operation selection
 *
*@{*/
    typedef enum
    {
        AES_Encrypt = 0,
        AES_Decrypt
    } dwt_aes_mode_e;
/**@}*/

/*! @name AES source port selections, this defines the source port for encrypted/unencrypted data
 *
*@{*/
    typedef enum
    {
        AES_Src_Scratch = 0,
        AES_Src_Rx_buf_0,
        AES_Src_Rx_buf_1,
        AES_Src_Tx_buf
    } dwt_aes_src_port_e;
/**@}*/

/*! @name AES destination port selections, this defines the dest port for encrypted/unencrypted data
 *
*@{*/
    typedef enum
    {
        AES_Dst_Scratch = 0,
        AES_Dst_Rx_buf_0,
        AES_Dst_Rx_buf_1,
        AES_Dst_Tx_buf,
        AES_Dst_STS_key
    } dwt_aes_dst_port_e;
/**@}*/

/*! @name AES key structure, the storage for 128/192/256-bit key
 *
*@{*/
    typedef struct
    {
        uint32_t key0;
        uint32_t key1;
        uint32_t key2;
        uint32_t key3;
        uint32_t key4;
        uint32_t key5;
        uint32_t key6;
        uint32_t key7;
    } dwt_aes_key_t;
/**@}*/

/*! @name AES core type, select which core is used GCM or CCM
 *
*@{*/
    typedef enum
    {
        AES_core_type_GCM = 0, /*!< Core type GCM */
        AES_core_type_CCM      /*!< Core type CCM */
    } dwt_aes_core_type_e;
/**@}*/

/*! @name AES key OTP type, key used from OTP or from RAM
 *
*@{*/
    typedef enum
    {
        AES_key_RAM = 0, /*!< Use the AES key from RAM */
        AES_key_OTP      /*!< Use the AES key from the OTP memory, key_load needs to match -> needs to be set to @ref AES_KEY_Src_Ram */
    } dwt_aes_key_otp_type_e;
/**@}*/

/*! @name AES OTP key selection, which of the two OTP keys are used
 *
*@{*/
    typedef enum
    {
        AES_key_otp_sel_1st_128 = 0, /*!< Key first 128-bits */
        AES_key_otp_sel_2nd_128      /*!< Key second 128-bits */
    } dwt_aes_otp_sel_key_block_e;
/**@}*/

/*! @name AES configuration
 *
*@{*/
    typedef struct
    {
        dwt_aes_otp_sel_key_block_e aes_otp_sel_key_block; //!< Select the key from OTP memory, first 128-bits or 2nd 128-bits, @ref dwt_aes_otp_sel_key_block_e
        dwt_aes_key_otp_type_e aes_key_otp_type;           //!< Using KEY from the OTP memory or RAM, if this is set to AES_key_OTP, KEY from the OTP memory is used
        dwt_aes_core_type_e aes_core_type;                 //!< Core type GCM or CCM*
        dwt_mic_size_e mic;                                //!< Message integrity code size
        dwt_aes_key_src_e key_src;                         //!< Location of the key: either as programmed in registers(128-bit), in the RAM or in the OTP memory
        dwt_aes_key_load_e key_load;                       //!< Loads key from RAM or uses KEY from the registers
        uint8_t key_addr;                                  //!< Address offset of AES key when using AES key in RAM
        dwt_aes_key_size_e key_size;                       //!< AES key length configuration corresponding to AES_KEY_128/192/256bit
        dwt_aes_mode_e mode;                               //!< Operation type encrypt/decrypt
    } dwt_aes_config_t;
/**@}*/

/*! @name AES job
 *
*@{*/
    typedef struct
    {
        uint8_t *nonce;              //!< Pointer to the nonce
        uint8_t *header;             //!< Pointer to header (this is not encrypted/decrypted)
        uint8_t *payload;            //!< Pointer to payload (this is encrypted/decrypted)
        uint8_t header_len;          //!< Header size
        uint16_t payload_len;        //!< Payload size
        dwt_aes_src_port_e src_port; //!< Source port
        dwt_aes_dst_port_e dst_port; //!< Dest port
        dwt_aes_mode_e mode;         //!< Encryption or decryption
        uint8_t mic_size;            //!< Tag size;
    } dwt_aes_job_t;
/**@}*/

/*! @name STS key structure, the storage for 128-bit STS CP key
 *
*@{*/
    typedef struct
    {
        uint32_t key0;
        uint32_t key1;
        uint32_t key2;
        uint32_t key3;
#ifndef WIN32
    } __attribute__((packed)) dwt_sts_cp_key_t;
#else
} dwt_sts_cp_key_t;
#endif // WIN32
/**@}*/

/*! @name STS IV structure, the storage for 128-bit STS CP IV (nonce)
 *
*@{*/
    typedef struct
    {
        uint32_t iv0;
        uint32_t iv1;
        uint32_t iv2;
        uint32_t iv3;
#ifndef WIN32
    } __attribute__((packed)) dwt_sts_cp_iv_t;
#else
} dwt_sts_cp_iv_t;
#endif // WIN32
/**@}*/

#define ERROR_DATA_SIZE      (-1)
#define ERROR_WRONG_MODE     (-2)
#define ERROR_WRONG_MIC_SIZE (-3)
#define ERROR_PAYLOAD_SIZE   (-4)
#define MIC_ERROR            0xFFU
#define STS_LEN_128BIT       16U

/*! @name Double buffer state
 *
*@{*/
    typedef enum
    {
        DBL_BUF_STATE_EN = 0, //!< Double buffer enabled
        DBL_BUF_STATE_DIS     //!< Double buffer disabled
    } dwt_dbl_buff_state_e;
/**@}*/

/*! @name Double buffer mode
 *
*@{*/
    typedef enum
    {
        DBL_BUF_MODE_AUTO = 0, //!< Automatic - the receiver is re-enabled automatically
        DBL_BUF_MODE_MAN       //!< Manual - the receiver is re-enabled by the host after processing the RX event
    } dwt_dbl_buff_mode_e;
/**@}*/

/*! @name Double buffer configuration
 *
*@{*/
    typedef enum
    {
        DBL_BUFF_OFF = 0x0,
        DBL_BUFF_ACCESS_BUFFER_0 = 0x1,
        DBL_BUFF_ACCESS_BUFFER_1 = 0x3,
    } dwt_dbl_buff_conf_e;
/**@}*/

/*! @name PLL channel selection
 *
*@{*/
    typedef enum
    {
        DWT_CH5 = 5,      //!< Ch 5 configuration with PLL using 38.4 MHz crystal
        DWT_CH9 = 9,      //!< Ch 9 configuration with PLL using 38.4 MHz crystal
    } dwt_pll_ch_type_e;
/**@}*/

/*! @name ADC capture configuration
 *
*@{*/
    typedef struct
    {
        int8_t *buffer;               //!< Pointer to a buffer into which to read captured ADC results (-1,0,1)
        uint16_t length;              //!< Must be divisible by 16, number of ADC results (-1,0,1) requested (max is 2048x32 / 2) div by 2 because each pair of I-,I+ produces 1 result
        uint16_t sample_start_offset; //!< Must be divisible by 16, offset in the CIR from which to start reading ADC sample data
        uint8_t thresholds[4];        //!< Returns the ADC thresholds at time of capture, for I and Q
        uint8_t test_mode_wrap;       //!< Returns pointer to array of data of length 2*length (i and q samples)
    } dwt_capture_adc_t;
/**@}*/

/*! @name Enum for linear TX power control
 *
*@{*/
    typedef enum
    {
        DWT_DATA_INDEX = 0,
        DWT_PHR_INDEX = 1,
        DWT_SHR_INDEX = 2,
        DWT_STS_INDEX = 3,
        DWT_MAX_POWER_INDEX = 4
    } dwt_power_indexes_e;
/**@}*/

/*! @name TX power indices structure
 *
*@{*/
    typedef struct {
        uint8_t input[DWT_MAX_POWER_INDEX];
        uint8_t output[DWT_MAX_POWER_INDEX];
    } power_indexes_t;
/**@}*/

/*! @name TX power adjustment structure
 *
*@{*/
    typedef struct
    {
        uint32_t tx_power_setting;
        uint8_t pll_bias;
    } tx_adj_cfg_t;
/**@}*/

/*! @name TX power adjustment result structure
 *
*@{*/
    typedef struct
    {
        tx_adj_cfg_t tx_frame_cfg;
    } tx_adj_res_t;
/**@}*/


/*!
 * The default XTAL trim value for load capacitors of 2pF.
 * During the initialization the XTAL trim value can be read from the OTP memory and in case it is not present, the default would be used instead
 * */
#define DEFAULT_XTAL_TRIM 0x2EU

/*!
 * Max allowed value for XTAL trim
 * */
#ifdef AUTO_DW3300Q_DRIVER
#define XTAL_TRIM_BIT_MASK 0x7FU
#else
#define XTAL_TRIM_BIT_MASK 0x3FU
#endif

/*! @name XTAL temperature compensation set parameters structure
 *
*@{*/
    typedef struct
    {
        int8_t temperature;               //!< Pass in TEMP_INIT (-127) to use on chip temperature sensor
        uint8_t crystal_trim;             //!< Pass in 0 if you want to use the calibration value from the OTP memory
        int8_t crystal_trim_temperature;  //!< Temperature of the crystal for crystal_trim, if TEMP_INIT (-127) will assume 25C.
        int32_t crystal_alpha;            //!< * 2^22 scaled alpha value
        int32_t crystal_beta;             //!< * 2^22 scaled beta value
    } dwt_xtal_trim_t;
/**@}*/

/*! @name Debug register name/value structure
 *
*@{*/
    typedef struct {
        char* name;
        uint32_t address;
    } register_name_add_t;
/**@}*/

    /*! @name
        ------------------------------------------------------------------------------------------------------------------
        * @brief The dwt_probe_s structure is a structure assembling all the external structures and function
        * that must be defined externally
        * NB: In porting this to a particular microprocessor, the implementer needs to define the low
        * level abstract functions matching the selected hardware.
    *@{*/
    struct dwt_probe_s
    {
        /*! ------------------------------------------------------------------------------------------------------------------
            * @brief dw pointer to an externally defined dwchip_s.
            * if set to NULL then an internal structure will be used
            * the typical use case is to support multiple DW chip connection on the same board
            * NB: see dwchip_s structure definition for details in deca_interface.h
        */
        void *dw;
        /*! ------------------------------------------------------------------------------------------------------------------
            * @brief dw pointer to an externally defined dwt_spi_s structure
            * NB: see dwt_spi_s structure definition for details in deca_interface.h
        */
        void *spi;
        /*! ------------------------------------------------------------------------------------------------------------------
        * @brief  This function wakeup device by an IO pin. UWB IC SPI_CS or WAKEUP pins can be used for this.
        *         wakeup_device_with_io() which is external to this file and is platform dependant and it should be modified to
        *         toggle the correct pin depending on the HW/MCU connections with UWB IC.
        */
        void(*wakeup_device_with_io)(void);
        /*! ------------------------------------------------------------------------------------------------------------------
            * @brief dw pointer to an externally defined dwt_driver_s structure list
            * NB: see dwt_driver_s structure definition for details in deca_interface.h
        */
        struct dwt_driver_s **driver_list;
        /*! ------------------------------------------------------------------------------------------------------------------
        * @brief  Number of availabe DW drivers.
        */
        uint8_t dw_driver_num;
    };
/**@}*/

    /* Extern definition for the driver descriptors */
#ifndef WIN32
#ifdef USE_DRV_DW3000
    extern const struct dwt_driver_s dw3000_driver;
#endif
#ifdef USE_DRV_DW3720
    extern const struct dwt_driver_s dw3720_driver;
#endif
#endif /* WIN32 */

    /********************************************************************************************************************/
    /*                                                     API LIST                                                     */
    /********************************************************************************************************************/

/* Functions added for compatibility with QSOC.*/
#define dwt_configurerfport_override(x)
#define dwt_configurerfport(x,y)
#define dwt_rfsw_config_t int /* whatever built-in type as it just to compile. Used as dwt_configurerfport() parameter. */

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This finction probes the UWB device. It selects the correct UWB driver for DW3xxx or QM33xxx device from the list (@ref dw_chip_id_e).
     *
     * @param[out] probe_interf: Pointer to a @ref dwt_probe_s structure.
     *
     * @return @ref DWT_ERROR if no driver found or @ref DWT_SUCCESS if driver is found.
     */
    int32_t dwt_probe(struct dwt_probe_s *probe_interf);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function returns the version of the API
     *
     * @return version (@ref DRIVER_VERSION_HEX)
     */
    int32_t dwt_apiversion(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will update dw pointer to the dwchip_s structure used by the driver APIs.
     *        There is a static struct dwchip_s static_dw = { 0 }; in the deca_compat.c file and a pointer to it, *dw.
     *
     * @param[in] new_dw: pointer to dw dwchip_s structure instatiated by MCPS layer
     *
     * @return pointer to the old device structure dwchip_s, which can be restored when deleting MCPS instance
     *
     */
    struct dwchip_s* dwt_update_dw(struct dwchip_s* new_dw);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function returns the driver version of the API as a string
     *
     * @return version string (@ref DRIVER_VERSION_STR)
     */
    const char *dwt_version_string(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to return the read voltage (Vbat) value measured @ 3.0 V and recorded in OTP address 0x8 (VBAT_ADDRESS)
     *
     * @note dwt_initialise() must be called prior to this function so that it can return a relevant value.
     *
     * @return the 8-bit Vbat value as programmed into the OTP memory in the factory
     */
    uint8_t dwt_geticrefvolt(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to return the read temperature (Vtemp) value measured @ 22 C and recorded in OTP address 0x9 (VTEMP_ADDRESS)
     *
     * @note dwt_initialise() must be called prior to this function so that it can return a relevant value.
     *
     * @return the 8-bit Vtemp value as programmed into the OTP memory in the factory
     */
    uint8_t dwt_geticreftemp(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to return the read part ID of the device
     *
     * This function returns the part identifier as programmed in the factory during
     * device test and qualification.
     *
     * @note The part ID is read and cached in memory during initialisation, therefore, 
     * dwt_initialise() must be called prior to this function so that it can return a relevant value.
     *
     * @return the 32-bit part ID value as programmed into the OTP memory in the factory
     */
    uint32_t dwt_getpartid(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to return the read lot ID of the device
     *
     * @note The lot ID is read and cached in memory during initialisation, therefore,
     * dwt_initialise() must be called prior to this function so that it can return a relevant value.
     *
     * @return the 64-bit lot ID value as programmed into the OTP memory in the factory
     */
    uint64_t dwt_getlotid(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to return the read device type and revision information of the UWB chip
     *
     * @return the silicon device ID (one of @ref dw_chip_id_e values)
     */
    uint32_t dwt_readdevid(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to return the read OTP revision
     *
     * @note dwt_initialise() must be called prior to this function so that it can return a relevant value.
     *
     * @return the read OTP revision value
     */
    uint8_t dwt_otprevision(void);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief This function sets the temperature to be used for PLL calibration of the device.
    *        @parblock
    *        It sets the cal_temp in dwt_local_data_t structure. It is the temperature to use for automotive PLL cal.
    *        If set to @ref TEMP_INIT (-127) the dwt_configure() will attempt to measure the temperature using onboard sensor.
    *        If set to @ref TEMP_INIT (i.e., -127), the next time dwt_initialise() is called the onchip temperature sensor
    *        will be used to read the temperature and set it as the calibration temperature.
    *        @endparblock
    *
    * @param[in] temperature: Expected operating temperature in celcius
    *
    */
    void dwt_setpllcaltemperature(int8_t temperature);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief This function reads the temperature in celcius which is used for PLL calibrations
    *
    * @return the temperature in celcius that will be used by PLL calibrations
    */
    int8_t dwt_getpllcaltemperature(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function enables/disables the fine grain TX sequencing (enabled by default).
     *
     * This is used to activate/deactivate fine grain TX sequencing (active by default).
     * In some applications/use cases the fine grain TX sequencing needs to be disabled,
     * e.g., continuous wave mode or when driving an external PA.
     * 
     * @param[in] enable: Set to 1 to enable fine grain TX sequencing, 0 to disable it.
     *
     */
    void dwt_setfinegraintxseq(int32_t enable);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to enable GPIO for external LNA or PA functionality. HW dependent, consult the DW3xxx User Manual.
     *        This can also be used for debug as enabling TX and RX GPIOs is quite handy to monitor DW3xxx TRX activity.
     *
     * @note Enabling PA functionality requires that fine grain TX sequencing is deactivated. This can be done using
     *       dwt_setfinegraintxseq().
     *
     * @param[in] lna_pa: Bit field (only bits 0 and 1 are used).
     *                    Bit 0 if set will enable LNA functionality.
     *                    Bit 1 if set will enable PA functionality.
     *                    To disable LNA/PA set all the bits to 0.
     *
     */
    void dwt_setlnapamode(int32_t lna_pa);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to configure GPIO function
     *
     * @param[in] gpio_mask: The full mask of all of the GPIOs for which to change the mode. Typically built from @ref dwt_gpio_mask_e values.
     * @param[in] gpio_modes: The GPIO modes to set. Typically built from @ref dwt_gpio_pin_e values.
     *
     */
    void dwt_setgpiomode(uint32_t gpio_mask, uint32_t gpio_modes);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to configure the GPIOs as inputs or outputs. All are set to input (i.e. 1) by default.
     *
     * @param[in] in_out: If corresponding GPIO bit is set to 1 then it is input, otherwise it is set as an output,
     *                    GPIO 0 = bit 0, GPIO 1 = bit 1 etc...
     * @note On the DW3000 the GPIO5 and GPIO6 direction is reversed, 1 = output, 0 = input. 
     */
    void dwt_setgpiodir(uint16_t in_out);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the current GPIO configuration. All GPIOs are set to input (i.e. 1) by default.
     *
     * @param[out] in_out: Will return the current GPIO configuration, if GPIO is an input the bit will be set to 1, otherwise 0 means the GPIO is configured as an output,
     *                    GPIO 0 = bit 0, GPIO 1 = bit 1 etc...
     * @note On the DW3000 the GPIO5 and GPIO6 direction is reversed, 1 = output, 0 = input. 
     *
     */
     void dwt_getgpiodir(uint16_t* in_out);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to set output value on GPIOs that have been configured for output via dwt_setgpiodir() API
     *
     * @param[in] gpio: Should be one or multiple of @ref dwt_gpio_mask_e values
     * @param[in] value: Logic value for GPIO or GPIOs if multiple set at same time.
     *
     */
    void dwt_setgpiovalue(uint16_t gpio, int32_t value);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the raw value of the GPIO pins.
     *        It is presumed that functions such as dwt_setgpiomode(), dwt_setgpiovalue() and dwt_setgpiodir() are called before this function.
     *        GPIO 0 = bit 0, GPIO 1 = bit 1 etc...
     *
     * @return a value that holds the value read on the GPIO pins.
     */
    uint16_t dwt_readgpiovalue(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function initialises the UWB transceiver:
     * It performs the initially required device configurations and initializes static data belonging to the low-level driver.
     *
     * @note
     * 1. This function needs to be run before dwt_configuresleep(), also the SPI frequency has to be < 7MHz.
     * 2. It also reads and applies LDO and BIAS tune and crystal trim values from the OTP memory.
     * 3. It is assumed this function is called after a device reset or on power up of the UWB transceiver.
     *
     * @param[in] mode: A bit mask to alter the default initialisation flow.
     *                  - @ref DWT_READ_OTP_ALL        - reads Part ID, Lot ID, reference battery voltage and temprature from OTP.
     *                  - @ref DWT_READ_OTP_PLID_DIS   - don't read part ID or lot ID from OTP
     *                  - @ref DWT_READ_OTP_VTBAT_DIS  - don't read reference battery voltage from OTP 
     *                  - @ref DWT_READ_OTP_TMP_DIS    - don't read reference temperature from OTP
     *
     * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
     */
    int32_t dwt_initialise(int32_t mode);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function can place UWB radio into IDLE/IDLE_PLL or IDLE_RC mode when it is not actively in TX or RX.
     *
     * @param[in] state: See @ref dwt_idle_init_modes_e
     * @parblock
     *                DWT_DW_IDLE (1) to put UWB radio into IDLE/IDLE_PLL state
     *                DWT_DW_INIT (0) to put UWB radio into INIT_RC state
     *                DWT_DE_IDLE_RC (2) to put UWB radio into IDLE_RC state
     * @endparblock
     *
     * @return @ref DWT_SUCCESS for success, or @ref DWT_ERR_PLL_LOCK for error
     */
    int dwt_setdwstate(int state);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to enable GPIO clocks. The clocks are needed to ensure correct GPIO operation
     *
     */
    void dwt_enablegpioclocks(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @deprecated This function is now deprecated. Will be removed in the future.
     *           Use dwt_restore_common() and dwt_restore_txrx() instead.
     * 
     * @brief This function needs to be called after device is woken up from DEEPSLEEP/SLEEP state. It will restore the
     * configuration which has not been automatically restored from AON
     *
     * @param[in] restore_mask: See @ref dwt_restore_type_e for more info on types of restore
     *
     * @return @ref DWT_SUCCESS or error (@ref dwt_error_e DWT_ERR_RX_CAL*) if PGF/ADC calibration returns an error.
     *
     */
    int32_t dwt_restoreconfig(dwt_restore_type_e restore_mask);

    /*!
    * @brief This function needs to be called after device is woken up from DEEPSLEEP/SLEEP state, to restore the
    * configuration which has not been automatically restored from AON.
    * 
    */
    void dwt_restore_common(void);

    /*!
    * @brief This function needs to be called after device is woken up from DEEPSLEEP/SLEEP state, and after dwt_restore_common().
    * It restores the TX and RX configuration which has not been automatically restored from AON.
    * If no TX or RX is needed, this function does not need to be called.
    * If only RX is needed, use DWT_RESTORE_RX_ONLY_MODE in restore_mask.
    * If only TX is needed, use DWT_RESTORE_TX_ONLY_MODE in restore_mask.
    * If both TX and RX are needed, use DWT_RESTORE_TXRX_MODE in restore_mask.
    * If ADC calibration is needed, use DWT_FORCE_ADCOFFSET_CAL in restore_mask together with DWT_RESTORE_RX_ONLY_MODE 
    *   or DWT_RESTORE_TXRX_MODE.
    * 
    * @note Current guidelines recommend to force ADC cal only:
    *              - After every 10 deg drop when temperature <= -10 C
    *              - Once when temperature > -10 and < 20
    *              - Once when temperature >= 20
    *
    * @param[in] restore_mask: See @ref dwt_restore_type_e for more info on types of restore
    * 
    * @returns @ref DWT_SUCCESS for success, or @ref DWT_ERR_PLL_LOCK if PLL not locked.
    */
    int32_t dwt_restore_txrx(uint8_t restore_mask);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function configures STS mode: e.g., @ref DWT_STS_MODE_OFF, @ref DWT_STS_MODE_1 etc
     * The dwt_configure should be called prior to this to configure other parameters
     *
     * @param[in] stsMode: e.g., @ref DWT_STS_MODE_OFF, @ref DWT_STS_MODE_1 etc. See @ref dwt_sts_mode_e.
     *
     */
    void dwt_configurestsmode(uint8_t stsMode);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will configure the PDOA mode.
     *
     * @param[in] pdoaMode: PDOA mode, see @ref dwt_pdoa_mode_e
     *
     * @note The dwt_configure() or dwt_setrxcode() must be called prior to updating PDOA mode with this API. 
     *       To modify preamble length or STS length call dwt_configure() first or dwt_setstslength().
     *
     * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error.
     */
    int dwt_setpdoamode(dwt_pdoa_mode_e pdoaMode);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function provides the main API for the configuration of the
     * UWB transceiver and this low-level driver. The input is a pointer to the data structure
     * of type @ref dwt_config_t that holds all the configurable items.
     * The @ref dwt_config_t structure shows which ones are supported.
     *
     * @param[in] config: Pointer to the configuration structure @ref dwt_config_t, which contains the device configuration data.
     *
     * @note If the RX calibration routine fails the device receiver performance will be severely affected,
     * the application should reset device and try again
     *
     * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR (e.g., when PLL CAL fails / PLL fails to lock)
     */
    int32_t dwt_configure(dwt_config_t *config);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function provides the API for the configuration of the TX power
     * The input is the desired TX power to set.
     *
     * @param[in] power: TX power to configure, 32-bits.
     * 
     * TX_POWER register contains:
     * - [31:24]     TX_CP_PWR (STS power)
     * - [23:16]     TX_SHR_PWR
     * - [15:8 ]     TX_PHR_PWR
     * - [7:0]       TX_DATA_PWR
     * 
     * @note The TX power can also be cofigured with PGdly and PGcount See @ref dwt_txconfig_t via dwt_configuretxrf()
     *
     */
    void dwt_settxpower(uint32_t power);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function provides the API for the configuration of the TX spectrum
     * including the power and pulse generator delay. The input is a pointer to the data structure
     * of type @ref dwt_txconfig_t that holds all the configurable items.
     *
     * @param[in] config: Pointer to the @ref dwt_txconfig_t configuration structure, which contains the TX RF config data
     * @note If config->PGcount != 0 the the PG calibration (bandwith calibration) will run,
     *       otherwise PGdelay value will be used to set the bandwidth.
     *
     */
    void dwt_configuretxrf(dwt_txconfig_t *config);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function sets the default values of the lookup tables depending on the channel selected.
     *
     * @param[in] channel: Channel that the device will be receiving on.
     *
     */
    void dwt_configmrxlut(int32_t channel);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function configures the STS AES 128-bit KEY value.
     * The default value is [31:00]0xc9a375fa,
     *                      [63:32]0x8df43a20,
     *                      [95:64]0xb5e5a4ed,
     *                     [127:96]0x0738123b
     *
     * @param[in] pStsKey: The pointer to the structure of @ref dwt_sts_cp_key_t type, which holds the 128-bit AES KEY value to generate STS
     *
     */
    void dwt_configurestskey(dwt_sts_cp_key_t *pStsKey);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function configures the STS AES 128-bit initial value (IV). The default value is 1, i.e., UWB IC reset value is 1.
     *
     * @param[in] pStsIv: The pointer to the structure of @ref dwt_sts_cp_iv_t type, which holds the IV value to generate STS
     *
     */
    void dwt_configurestsiv(dwt_sts_cp_iv_t *pStsIv);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function re-loads the STS AES initial value (IV) and STS AES KEY into the STS AES block
     *
     */
    void dwt_configurestsloadiv(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This API function writes the receiver antenna delay (in device time units) to RX registers
     *
     * @param[in] rxAntennaDelay: This is the total (RX) antenna delay value, which
     *                         will be programmed into the RX register, and subtracted from the raw RX timestamp
     *
     */
    void dwt_setrxantennadelay(uint16_t rxAntennaDelay);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This API function reads the antenna delay (in time units) from the RX antenna delay register
     *
     * @return 16-bit RX antenna delay value which is currently programmed in CIA_CONF_ID register
     */
    uint16_t dwt_getrxantennadelay(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This API function writes the receiver antenna delay (in device time units) to TX registers
     *
     * @param[in] txAntennaDelay: This is the total (TX) antenna delay value, which
     *                         will be programmed into the TX register, and added to the raw TX timestamp
     *
     */
    void dwt_settxantennadelay(uint16_t txAntennaDelay);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief This API function reads the antenna delay (in time units) from the TX antenna delay register
    *
    * @return 16-bit TX antenna delay value which is currently programmed in TX_ANTD_ID register
    */
    uint16_t dwt_gettxantennadelay(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This API function writes the supplied TX data into the UWB radio TX buffer.
     * The input parameters are the data length in bytes and a pointer to those data bytes.
     *
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
    int32_t dwt_writetxdata(uint16_t txDataLength, uint8_t *txDataBytes, uint16_t txBufferOffset);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This API function configures the TX frame control register before the transmission of a frame
     *
     * @param[in] txFrameLength: This is the length of TX message (including the 2 byte CRC) - max is 1023
     *                              Note the standard PHR mode allows up to 127 bytes
     *                              if > 127 is programmed, @ref DWT_PHRMODE_EXT needs to be set in the phrMode configuration
     *                              see dwt_configure() function
     * @param[in] txBufferOffset: The offset in the tx buffer to start writing the data
     * @param[in] ranging: Set to 1 if this is a ranging frame, else can be set to 0.
     *
     */
    void dwt_writetxfctrl(uint16_t txFrameLength, uint16_t txBufferOffset, uint8_t ranging);

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
    * @param[in] preambleLength: The preamble length value in symbols.
    *
    * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
    */
    int32_t dwt_setplenfine(uint16_t preambleLength);

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
     * @param[in] pll_rx_prebuf_cfg: New "PLL RX Prebuffer Enable" Configuration.
     *    DWT_PLL_RX_PREBUF_DISABLE - Disable the DW RX PLL Pre-Buffers
     *    DWT_PLL_RX_PREBUF_ENABLE - Enable the DW RX PLL Pre-Buffers   
     *
     * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
     */
     int dwt_setpllrxprebufen(dwt_pll_prebuf_cfg_e pll_rx_prebuf_cfg);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This call initiates the transmission, input parameter indicates which TX mode is used see below
     *
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
    int32_t dwt_starttx(uint8_t mode);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This API function configures the reference time used for relative timing of delayed sending and reception.
     * The value is at a 8ns resolution.
     *
     * @param[in] reftime: The reference time (which together with DX_TIME or TX timestamp or RX timestamp time is used to define a
     * transmission time or delayed RX on time)
     *
     */
    void dwt_setreferencetrxtime(uint32_t reftime);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This API function configures the delayed transmit time or the delayed RX on time
     *
     * @param[in] starttime: The TX/RX start time (the 32-bits should be the high 32-bits of the system time at which to send the message,
     * or at which to turn on the receiver)
     *
     */
    void dwt_setdelayedtrxtime(uint32_t starttime);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the TX timestamp (adjusted with the programmed antenna delay)
     *
     * @param[out] timestamp: A pointer to a 5-byte buffer which will store the read TX timestamp.
     *                   The timestamp buffer will contain the value after the function call
     *
     */
    void dwt_readtxtimestamp(uint8_t *timestamp);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the high 32-bits of the TX timestamp (adjusted with the programmed antenna delay)
     *
     * @return High 32-bits of 40-bit TX timestamp
     */
    uint32_t dwt_readtxtimestamphi32(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the low 32-bits of the TX timestamp (adjusted with the programmed antenna delay)
     *
     * @return Low 32-bits of 40-bit TX timestamp
     */
    uint32_t dwt_readtxtimestamplo32(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the PDOA result (Phase Difference On Arrival).
     *        It is the phase difference between either the Ipatov and STS phase-of-arrival (POA), or
     *        the two STS POAs, depending on the PDOA mode of operation.
     *
     * @note To convert to degrees: float pdoa_deg = ((float)pdoa / (1 << 11)) * 180 / M_PI
     *
     * @return The PDOA result signed 16-bit number, ([1:-11] radian units)
     */
    int16_t dwt_readpdoa(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function is used to read the TDOA (Time Difference On Arrival). The TDOA value that is read from the
     * register is 41-bits in length. However, 6 bytes (or 48-bits) are read from the register. The remaining 7-bits at
     * the 'top' of the 6 bytes that are not part of the TDOA value should be set to zero and should not interfere with
     * rest of the 41-bit value. However, there is no harm in masking the returned value.
     *
     * @param[out] tdoa: time difference of arrival - buffer of 6 bytes that will be filled with TDOA value by calling this function
     *
     */
    void dwt_readtdoa(uint8_t *tdoa);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function is used to read the TDOA (Time Difference On Arrival) and the PDOA result (Phase Difference Of Arrival).
     * The device can measure 3 TDOA/PDOA results based on the configuration of the PDOA table.
     *
     * The TDOA value that is read from the register is 16-bits in length. It is a signed number, in device time units.
     *
     * The PDOA value that is read from the register is 15-bits in length. It is a signed number s[1:-11] (radians).
     * @note To convert to degrees: float pdoa_deg = ((float)pdoa / (1 << 11)) * 180 / M_PI
     *
     * @param[out] result: Pointer to dwt_pdoa_tdoa_res_t structure which into which PDOA, TDOA and FP_OK will be read
     * @param[in] index: Specifies which of 3 possible results are to be read (0, 1, 2)
     *
     */
    void dwt_read_tdoa_pdoa(dwt_pdoa_tdoa_res_t *result, int32_t index);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the RX timestamp (adjusted time of arrival)
     *
     * @param[out] timestamp: A pointer to a 5-byte buffer which will store the read RX timestamp.
     *                   The timestamp buffer will contain the value after the function call
     * @param[in] segment: Not used in and should be set as @ref DWT_COMPAT_NONE
     *
     */
    void dwt_readrxtimestamp(uint8_t *timestamp, dwt_ip_sts_segment_e segment);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the RX timestamp (unadjusted time of arrival)
     * @note The RX raw timestamp is read into the 5-byte array and the lowest byte is always 0x0.
     *
     * @param[out] timestamp: A pointer to a 5-byte buffer which will store the read RX timestamp.
     *                   The timestamp buffer will contain the value after the function call
     *
     */
    void dwt_readrxtimestampunadj(uint8_t *timestamp);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the RX timestamp (adjusted time of arrival) w.r.t. Ipatov CIR
     *
     * @param[out] timestamp: A pointer to a 5-byte buffer which will store the read RX timestamp.
     *                   The timestamp buffer will contain the value after the function call
     *
     */
    void dwt_readrxtimestamp_ipatov(uint8_t *timestamp);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the RX timestamp (adjusted time of arrival) w.r.t. STS CIR
     *
     * @param[out] timestamp: A pointer to a 5-byte buffer which will store the read RX timestamp.
     *                   The timestamp buffer will contain the value after the function call
     *
     */
    void dwt_readrxtimestamp_sts(uint8_t *timestamp);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the high 32-bits of the RX timestamp.
     *        The adjusted timestamp is read (adjusted with the programmed antenna delay)
     *
     * @note This should not be used when RX double buffer mode is enabled. Following APIs to read RX timestamp should be
     * used:  dwt_readrxtimestamp_ipatov() or dwt_readrxtimestamp_sts() or dwt_readrxtimestamp()
     *
     * @return High 32-bits of RX timestamp
     */
    uint32_t dwt_readrxtimestamphi32(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the low 32-bits of the RX timestamp.
     *        The adjusted timestamp is read (adjusted with the programmed antenna delay)
     *
     * @note This should not be used when RX double buffer mode is enabled. Following APIs to read RX timestamp should be
     * used:  dwt_readrxtimestamp_ipatov() or dwt_readrxtimestamp_sts() or dwt_readrxtimestamp()
     *
     * @param[in] segment: Not used in and should be set as @ref DWT_COMPAT_NONE
     *
     *
     * @return Low 32-bits of RX timestamp
     */
    uint32_t dwt_readrxtimestamplo32(dwt_ip_sts_segment_e segment);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the high 32-bits of the UWB IC system time
     *
     * @return High 32-bits of system time timestamp
     */
    uint32_t dwt_readsystimestamphi32(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the UWB IC system time
     *
     * @param[out] timestamp: A pointer to a 4-byte buffer which will store the read system time.
     *                   The timestamp buffer will contain the value after the function call
     *
     */
    void dwt_readsystime(uint8_t *timestamp);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to turn off the transceiver
     *
     */
    void dwt_forcetrxoff(void);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief This call turns on the receiver, can be immediate or delayed (depending on the mode parameter). In the case of a
    * "late" error the receiver will be turned on immediately unless the @ref DWT_IDLE_ON_DLY_ERR is not set.
    * The receiver will stay turned on, listening to any messages until
    * it either receives a good frame, an error (CRC, PHY header, Reed Solomon) or  it times out (SFD, Preamble or Frame).
    *
    * @param[in] mode: This can be one of the following values (@ref dwt_startrx_mode_e):
    *
    * DWT_START_RX_IMMEDIATE      0x00    Enable the receiver immediately
    * DWT_START_RX_DELAYED        0x01    Set up delayed RX, if "late" error triggers the RX will be enabled immediately
    * DWT_IDLE_ON_DLY_ERR         0x02    If delayed RX failed due to "late" error then if this
    *                                     flag is set the RX will not be re-enabled immediately, and device will be in IDLE when function exits
    * DWT_START_RX_DLY_REF        0x04    Enable the receiver at specified time (time in DREF_TIME register + any time in DX_TIME register)
    * DWT_START_RX_DLY_RS         0x08    Enable the receiver at specified time (time in RX_TIME_0 register + any time in DX_TIME register)
    * DWT_START_RX_DLY_TS         0x10    Enable the receiver at specified time (time in TX_TIME_LO register + any time in DX_TIME register)
    *
    * e.g., (DWT_START_RX_DELAYED | DWT_IDLE_ON_DLY_ERR) 0x03 used to disable re-enabling of receiver if delayed RX failed due to "late" error
    *
    * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error (e.g., a delayed receive enable will be too far in the future, the delayed time has passed)
    */
    int32_t dwt_rxenable(int32_t mode);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This enables/disables and configures the SNIFF mode.
     *
     * The SNIFF mode is a low-power reception mode where the receiver is sequenced on and off instead of being on all the time.
     * The time spent in each state (on/off) is specified through the parameters below.
     * See DW3xxx/QM33xxx User Manual section 4.5 "Low-Power SNIFF mode" for more details.
     *
     * @param[in] enable: Set to 1 to enable SNIFF mode, 0 to disable. When 0, all other parameters are not taken into account.
     * @param[in] timeOn: Duration of receiver ON phase, expressed in multiples of PAC size. The counter automatically adds 1 PAC
     *                 size to the value set. Min value that can be set is 1 (i.e., an ON time of 2 PAC size), max value is 15.
     * @param[in] timeOff: Duration of receiver OFF phase, expressed in multiples of 128/125 us (~1 us). Max value is 255.
     *
     */
    void dwt_setsniffmode(int32_t enable, uint8_t timeOn, uint8_t timeOff);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This call enables the double receive buffer mode
     *
     * @param[in] dbl_buff_state: Enum variable for enabling/disabling double buffering mode (see @ref dwt_dbl_buff_state_e)
     * @param[in] dbl_buff_mode: Enum variable for receiver re-enable (see @ref dwt_dbl_buff_mode_e)
     *
     */
    void dwt_setdblrxbuffmode(dwt_dbl_buff_state_e dbl_buff_state, dwt_dbl_buff_mode_e dbl_buff_mode);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This call signals to the chip that the specific RX buff is free (available) for writing the data of the the next
     *        received frame
     *
     */
    void dwt_signal_rx_buff_free(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This call enables RX timeout (SYS_STATUS_RFTO event)
     *
     * @param[in] rx_time: How long the receiver remains on from the RX enable command
     *                  The time parameter used here is in 1.0256 us (512/499.2MHz) units
     *                  If set to 0 the timeout is disabled.
     *
     */
    void dwt_setrxtimeout(uint32_t rx_time);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This call enables preamble timeout (SYS_STATUS_RXPTO event)
     *
     * @param[in]  timeout: Preamble detection timeout, expressed in multiples of PAC size. The counter automatically adds 1 PAC
     *                   size to the value set. Min value that can be set is 1 (i.e., a timeout of 2 PAC size).
     *                   Value 0 will disable the preamble timeout.
     *                   Value X >= 1 sets a timeout equal to (X + 1) * PAC
     *
     */
    void dwt_setpreambledetecttimeout(uint16_t timeout);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief Calibrates the local oscillator (LP OSC) as its frequency can vary between 15 kHz and 34 kHz depending on temp and voltage
     *
     * @note This function needs to be run before dwt_configuresleepcnt(), so that we know what the counter units are.
     *
     * @return The number of XTAL cycles per low-power oscillator cycle. LP OSC frequency = 38.4 MHz/return value
     */
    uint16_t dwt_calibratesleepcnt(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief Sets the sleep counter to new value, this function programs the high 16-bits of the 28-bit counter
     *
     * @note This function needs to be run before dwt_configuresleep(), also the SPI frequency has to be < 3MHz
     *
     * @param[in] sleepcnt: This it value of the sleep counter to program
     *
     */
    void dwt_configuresleepcnt(uint16_t sleepcnt);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief Configures the device for both DEEP_SLEEP and SLEEP modes, and on-wake mode
     * i.e., before entering the sleep, the device should be programmed for TX or RX, then upon "waking up" the TX/RX settings
     * will be preserved and the device can immediately perform the desired action TX/RX
     *
     * @note e.g., Tag operation - after deep sleep, the device needs to just load the TX buffer and send the frame
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
     *      DWT_SLP_CNT_RPT  0x40 - sleep counter repeats after expiration
     *      DWT_PRESRVE_SLP  0x20 - allows for SLEEP_EN bit to be "preserved", although it will self-clear on wake up
     *      DWT_WAKE_WK      0x10 - wake up on WAKEUP PIN
     *      DWT_WAKE_CS      0x8 - wake up on chip select
     *      DWT_BR_DET       0x4 - enable brownout detector during sleep/deep sleep
     *      DWT_SLEEP        0x2 - enable sleep
     *      DWT_SLP_EN       0x1 - enable sleep/deep sleep functionality
     *
     * @param[in] mode: Configures on-wake parameters
     * @param[in] wake: Configures wake up parameters
     *
     */
    void dwt_configuresleep(uint16_t mode, uint8_t wake);

    /*! ------------------------------------------------------------------------------------------------------------------
     *
     * @brief This function clears the AON configuration in UWB IC
     *
     */
    void dwt_clearaonconfig(void);

    /*!
     * @brief This function puts the device into deep sleep or sleep. dwt_configuresleep() should be called first
     * to configure the sleep and on-wake/wake-up parameters
     *
     * @param[in] idle_rc: If this is set to @ref DWT_DW_IDLE_RC, the auto INIT2IDLE bit will be cleared prior to going to sleep
     *                  thus after wakeup device will stay in the IDLE_RC state (PLL will not be enabled).
     *
     * @note With DW3000 devices, it is recommended to use @ref DWT_DW_IDLE_RC rather than @ref DWT_DW_IDLE to speed up wake up time.
     *
     */
    void dwt_entersleep(int32_t idle_rc);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief Sets or clears the auto TX to sleep bit. This means that after a frame transmission
     * the device will enter sleep or deep sleep mode. The dwt_configuresleep() function
     * needs to be called before this to configure the on-wake settings
     *
     * @note The IRQ line has to be low/inactive (i.e., no pending events) otherwise device will not enter sleep until the interrupt has been cleared.
     * @deprecated This API has been deprecated, might be removed in a future major release.
     *       Consider using the dwt_entersleepafter() function instead.
     *
     * @param[in] enable: Set to 1 to configure the device to enter sleep or deep sleep after TX, 0 disables the configuration
     *
     */
    void dwt_entersleepaftertx(int32_t enable);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief Sets or clears the auto TX and/or RX to sleep bits.
     *
     * This makes the device automatically enter deep sleep or sleep mode after a frame transmission and/or reception.
     * dwt_configuresleep() needs to be called before this to configure the sleep and on-wake/wake-up parameters.
     *
     * @note The IRQ line has to be low/inactive (i.e., no pending events)
     *
     * @param[in] event_mask: Bitmask to go to sleep after:
     *  - DWT_TX_COMPLETE to configure the device to enter sleep or deep sleep after TX
     *  - DWT_RX_COMPLETE to configure the device to enter sleep or deep sleep after RX
     *
     */
    void dwt_entersleepafter(int32_t event_mask);

#ifdef WIN32
    /*! ------------------------------------------------------------------------------------------------------------------
     * ********** @note in decatest only ****************
     *
     * @brief wake up the device from sleep mode using the SPI read,
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
     * @param[in] buff: This is a pointer to the dummy buffer which will be used in the SPI read transaction used for the WAKE UP of the device
     * @param[in] length: This is the length of the dummy buffer
     *
     * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
     */
    int32_t dwt_spicswakeup(uint8_t *buff, uint16_t length);
#endif

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function is used to register the different callbacks to be called when one or more of the corresponding events occur.
     * For example we can have a TX callback which is called when TX done event occurs following a transmission of a packet.
     *
     * @note Callbacks can be undefined (set to NULL). In this case, dwt_isr() will process the event as usual but the 'null'
     * callback will not be called.
     *
     * @param[in] callbacks: A pointer to a reference to the struct containing references for needed callback. See @ref dwt_callbacks_s
     *
     */
     void dwt_setcallbacks(dwt_callbacks_s *callbacks);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function checks if the IRQ line is active, this can used instead of the interrupt handler
     *
     * @return value is 1 if the SYS_STATUS_IRQS bit is set and 0 otherwise
     */
    uint8_t dwt_checkirq(void);

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
     * @return value is 1 if the IDLE_RC bit is set and 0 otherwise
     */
    uint8_t dwt_checkidlerc(void);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief This is the driver's general Interrupt Service Routine (ISR). It will process/report the following events:
    *          - RXFCG (through cbRxOk callback)
    *          - TXFRS (through cbTxDone callback)
    *          - RXRFTO/RXPTO (through cbRxTo callback)
    *          - RXPHE/RXFCE/RXRFSL/RXSFDTO/AFFREJ/LDEERR (through cbRxErr callback)
    *        For all events, corresponding interrupts are cleared and necessary resets are performed. In addition, in the RXFCG case,
    *        received frame information and frame control are read before calling the callback. If double buffering is activated, it
    *        will also toggle between reception buffers once the reception callback processing has ended.
    *
    *        ! This version of the ISR supports double buffering but does not support automatic RX re-enabling !
    *
    * @note  In PC based system using (Cheetah or ARM) USB to SPI converter there can be no interrupts, however we still need something
    *        to take the place of it and operate in a polled way. In an embedded system this function should be configured to be triggered
    *        on any of the interrupts described above.
    *
    */
    void dwt_isr(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function enables the specified events to trigger an interrupt.
     * The following events can be found in SYS_ENABLE_LO and SYS_ENABLE_HI registers.
     *
     *
     * @param[in] bitmask_lo: Sets the events in SYS_ENABLE_LO_ID register which will generate interrupt
     * @param[in] bitmask_hi: Sets the events in SYS_ENABLE_HI_ID register which will generate interrupt
     * @param[in] INT_options: If set to @ref DWT_ENABLE_INT additional interrupts as selected in the bitmask are enabled
     *                       If set to @ref DWT_ENABLE_INT_ONLY the interrupts in the bitmask are forced to selected state
     *                       i.e., the mask is written to the register directly.
     *                       Otherwise (if set to @ref DWT_DISABLE_INT) clear the interrupts as selected in the bitmask
     *
     */
    void dwt_setinterrupt(uint32_t bitmask_lo, uint32_t bitmask_hi, dwt_INT_options_e INT_options);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function is used to calculate 8-bit CRC. It uses 100000111 polynomial (i.e., P(x) = x^8+ x^2+ x^1+ x^0)
     *
     * @param[in] byteArray: Data to calculate CRC for
     * @param[in] flen     : Length of byteArray
     * @param[in] crcInit  : Initialisation value for CRC calculation
     *
     * @return 8-bit calculated CRC value
     */
    uint8_t dwt_generatecrc8(const uint8_t *byteArray, uint32_t flen, uint8_t crcInit);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to enable SPI CRC check in UWB IC
     *
     * @param[in] crc_mode: If set to @ref DWT_SPI_CRC_MODE_WR then SPI CRC checking will be performed in UWB IC on each SPI write
     *                   last byte of the SPI write transaction needs to be the 8-bit CRC, if it does not match
     *                   the one calculated by UWB IC SPI CRC ERROR event will be set in the status register (SYS_STATUS_SPICRC)
     *
     * @param[in] spireaderr_cb: This needs to contain the callback function pointer which will be called when SPI read error
     *                        is detected (when the UWB IC generated CRC does not match the one calculated by  dwt_generatecrc8()
     *                        following the SPI read transaction)
     *
     */
    void dwt_enablespicrccheck(dwt_spi_crc_mode_e crc_mode, dwt_spierrcb_t spireaderr_cb);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to enable the frame filtering. The default option is to
     * accept any data and ACK frames with correct destination address.
     *
     * @param[in] enabletype: A bitmask which enables/disables the frame filtering and configures 802.15.4 type
     *       - DWT_FF_ENABLE_802_15_4      0x2             - use 802.15.4 filtering rules
     *       - DWT_FF_DISABLE              0x0             - disable FF
     * @param[in] filtermode: A bitmask which configures the frame filtering options according to
     *       - DWT_FF_BEACON_EN            0x001           - beacon frames allowed
     *       - DWT_FF_DATA_EN              0x002           - data frames allowed
     *       - DWT_FF_ACK_EN               0x004           - ack frames allowed
     *       - DWT_FF_MAC_EN               0x008           - mac control frames allowed
     *       - DWT_FF_RSVD_EN              0x010           - reserved frame types allowed
     *       - DWT_FF_MULTI_EN             0x020           - multipurpose frames allowed
     *       - DWT_FF_FRAG_EN              0x040           - fragmented frame types allowed
     *       - DWT_FF_EXTEND_EN            0x080           - extended frame types allowed
     *       - DWT_FF_COORD_EN             0x100           - behave as coordinator (can receive frames with no dest address (PAN ID has to match))
     *       - DWT_FF_IMPBRCAST_EN         0x200           - allow MAC implicit broadcast
     *
     */
    void dwt_configureframefilter(uint16_t enabletype, uint16_t filtermode);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to set the PAN ID
     *        Used when frame filtering is enabled with dwt_configureframefilter()
     *
     * @param[in] panID: This is the network PAN ID
     *
     */
    void dwt_setpanid(uint16_t panID);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to set 16-bit (short) address
     *        Used when frame filtering is enabled with dwt_configureframefilter()
     *
     * @param[in] shortAddress: This sets the 16-bit short address
     *
     */
    void dwt_setaddress16(uint16_t shortAddress);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to set the EUI 64-bit (long) address
     *        Used when frame filtering is enabled with dwt_configureframefilter()
     *
     * @param[in] eui64: This is the pointer to a buffer that contains the 64-bit address
     *
     */
    void dwt_seteui(uint8_t *eui64);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to get the EUI 64-bit from the DW IC
     *
     * @param[in] eui64: This is the pointer to a buffer that will contain the read 64-bit EUI value
     *
     */
    void dwt_geteui(uint8_t *eui64);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This call enables the auto-ACK feature. If the responseDelayTime (parameter) is 0, the ACK will be sent a.s.a.p.,
     * otherwise it will be sent with a programmed delay (in symbols), max is 255.
     * @note Needs to have frame filtering enabled as well.
     *
     * @param[in] responseDelayTime: If non-zero the ACK is sent after this delay, max is 255.
     * @param[in] enable: Enables or disables the auto-ACK feature
     *
     */
    void dwt_enableautoack(uint8_t responseDelayTime, int32_t enable);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function is used to write a 16-bit address to a desired Low-Energy device (LE) address.
     * It is used for the frame pending to function when the correct bits are set in the frame filtering configuration
     * via the dwt_configureframefilter(). See dwt_configureframefilter() for more details.
     *
     * @param[in] addr: The address value to be written to the selected LE register
     * @param[in] leIndex: Low-Energy device (LE) address to write to, see @ref dwt_le_addresses_e
     *
     */
    void dwt_configure_le_address(uint16_t addr, int32_t leIndex);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This sets the receiver turn on delay time after a transmission of a frame.
     *
     * @param[in] rxDelayTime: The delay is in UWB microseconds, 20-bit value.
     *
     */
    void dwt_setrxaftertxdelay(uint32_t rxDelayTime);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function resets the UWB IC
     *
     * @note SPI rate must be <= 7MHz before a call to this function as the device will use FOSC/4 as part of internal reset
     *
     * @param[in] reset_semaphore: If set to 1 the semaphore will be also reset. (The semaphore is only valid for DW3720/QM33xxx device)
     *
     */
    void dwt_softreset(int32_t reset_semaphore);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the data from the RX buffer, from an offset location give by offset parameter
     *
     * @param[out] buffer: Pointer to the buffer into which the data will be read
     * @param[in] length: The length of data to read (in bytes)
     * @param[in] rxBufferOffset: The offset in the RX buffer from which to read the data
     *
     */
    void dwt_readrxdata(uint8_t *buffer, uint16_t length, uint16_t rxBufferOffset);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to write the data to the scratch buffer, to an offset location given by offset parameter.
     * The scratch buffer size is 128 bytes. The buffer can be used by the AES engine depending on the configuration of
     * destination and source ports:  @ref dwt_aes_src_port_e and @ref dwt_aes_dst_port_e
     *
     * @param[in] buffer: Pointer to the buffer which contains the data to write to the device
     * @param[in] length: The length of data to write (in bytes)
     * @param[in] bufferOffset: The offset in the scratch buffer to which to write the data
     *
     */
    void dwt_write_scratch_data(uint8_t *buffer, uint16_t length, uint16_t bufferOffset);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the data from the scratch buffer, from an offset location given by offset parameter.
     * The scratch buffer size is 128 bytes. The buffer can be used by the AES engine depending on the configuration of
     * destination and source ports:  @ref dwt_aes_src_port_e and @ref dwt_aes_dst_port_e
     *
     * @param[out] buffer: Pointer to the buffer into which the data will be read
     * @param[in]  length: The length of data to read (in bytes)
     * @param[in]  bufferOffset: The offset in the scratch buffer from which to read the data
     *
     */
    void dwt_read_scratch_data(uint8_t *buffer, uint16_t length, uint16_t bufferOffset);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the 18-bit data from the Accumulator (CIR) buffer, from an offset location give by offset parameter
     *        for 18-bit complex samples, each sample is 6 bytes (3 real and 3 imaginary)
     *
     *
     * @note Because of an internal memory access delay when reading the CIR the first octet output is a dummy octet
     *       that should be discarded. This is true no matter what sub-index the read begins at.
     *
     * @param[out] buffer: The buffer into which the data will be read
     * @param[in] len:  The length of data to read (in bytes)
     * @param[in] accOffset: The offset in the acc buffer from which to read the data, this is a complex sample index
     *                       e.g., to read 10 samples starting at sample 100
     *                       buffer would need to be >= 10*6 + 1, length is 61 (1 is for dummy), accOffset is 100
     *
     *
     *
     * @deprecated This function is now deprecated for new development. Plase use dwt_readcir() or dwt_readcir_48b()
     */
    void dwt_readaccdata(uint8_t *buffer, uint16_t len, uint16_t accOffset);

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
    int dwt_readcir(uint32_t *buffer, dwt_acc_idx_e cir_idx, uint16_t sample_offs,
                        uint16_t num_samples, dwt_cir_read_mode_e mode);

    /*!
     * This function reads the CIR/accumulator data in the compact 48-bit sample mode.
     *
     * This is used to read 48-bit complex samples from the CIR/Accumulator buffer, from an offset location
     * given by offset parameter. Each sample is 6 bytes (3 real and 3 imaginary).
     *
     * @note In the QM33xxx devices the @ref DWT_CIR_READ_FULL is already 48-bit. This function is added only for compatibility with QM35xxx devices
     *
     * @param[out] buffer: The buffer into which the data will be read NB: the buffer size needs to be >= num_samples*6 bytes
     * @param[in] acc_idx: Index of the accumulator to read data from
     * @param[in] sample_offs: The sample index offset within the selected accumulator to start reading from
     * @param[in] num_samples: The number of complex samples to read (each sample is a 48-bit complex value)
     *
     * @return DWT_SUCCESS or DWT_ERROR if wrong parameters were passed
     */
    int dwt_readcir_48b(uint8_t *buffer, dwt_acc_idx_e acc_idx, uint16_t sample_offs, uint16_t num_samples);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the crystal offset (relating to the frequency offset of the far UWB device compared to this one)
     *        @note The returned signed 16-bit number should be divided by 2^26 to get ppm offset.
     *              A positive value means the local (RX) clock is running slower than that of the remote (TX) device.
     *
     * @return The offset value, signed 16-bit number. (s[-15:-26])
     */
    int16_t dwt_readclockoffset(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the RX carrier integrator value (relating to the frequency offset of the TX node)
     *        @note This is a 21-bit signed quantity, the function sign extends the most significant bit, which is bit #20
     *        (numbering from bit zero) to return a 32-bit signed integer value.
     *        A positive value means the local (RX) clock is running slower than that of the remote (TX) device.
     *
     * @return The carrier integrator value, signed 32-bit number.
     */
    int32_t dwt_readcarrierintegrator(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function enables CIA diagnostic data. When turned on the following registers will be logged:
     * IP_TOA_LO, IP_TOA_HI, STS_TOA_LO, STS_TOA_HI, STS1_TOA_LO, STS1_TOA_HI, CIA_TDOA_0, CIA_TDOA_1_PDOA, CIA_DIAG_0, CIA_DIAG_1
     *
     * @param[in] enable_mask: Configures the CIA diagnostic logging in normal/single and double buffer (DB) modes.
     *                @ref DW_CIA_DIAG_LOG_MAX (0x8)   - CIA to copy to swinging set a maximum set of diagnostic registers in DB mode.
     *                @ref DW_CIA_DIAG_LOG_MID (0x4)   - CIA to copy to swinging set a medium set of diagnostic registers in DB mode.
     *                @ref DW_CIA_DIAG_LOG_MIN (0x2)   - CIA to copy to swinging set a minimal set of diagnostic registers in DB mode.
     *                @ref DW_CIA_DIAG_LOG_ALL (0x1)   - CIA to log all diagnostic registers.
     *                @ref DW_CIA_DIAG_LOG_OFF (0x0)   - CIA to log reduced set of diagnostic registers.
     *
     */
    void dwt_configciadiag(uint8_t enable_mask);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function reads the STS signal quality index
     *
     * @param[in] rxStsQualityIndex: The signed STS quality index value (int16_t).
     * @param[in] stsSegment       : Not used
     *
     * @return value >=0 for good and < 0 if bad STS quality.
     *
     * @note For the 64 MHz PRF if value is >= STSQUAL_THRESH_64_SH15 (60%) of the STS length then we can assume good STS reception.
     *       Otherwise the STS timestamp may not be accurate.
     */
    int32_t dwt_readstsquality(int16_t *rxStsQualityIndex, int32_t stsSegment);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function reads the STS status
     *
     * @param[in] stsStatus: The (uint8_t) STS status value.9 bits of this buffer are populated with various STS statuses. The
     *                    remaining 7 bits are ignored.
     * @param[in] sts_num: 0 for 1st STS, 1 for 2nd STS (2nd is only valid when PDOA Mode 3 is used)
     *
     * @return value 0 for good/valid STS < 0 if bad STS quality.
     */
    int32_t dwt_readstsstatus(uint16_t *stsStatus, int32_t sts_num);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function reads the RX signal quality diagnostic data
     *
     * @param[out] diagnostics: Diagnostic structure (@ref dwt_rxdiag_t) pointer,
     *                          this will contain the diagnostic data read from the UWB IC
     *
     */
    void dwt_readdiagnostics(dwt_rxdiag_t *diagnostics);

    /*! ---------------------------------------------------------------------------------------------------
    * @brief This function reads the CIA diagnostics for an individual CIR/accumulator
    *
    * @param[out] diag: Pointer to a CIR diagnostics structure for a particular CIR/accumulator
    * @param[in] acc_idx: CIR index (see @ref dwt_acc_idx_e)
    *
    * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error (if input parameters not correct)
    */
    int dwt_readdiagnostics_acc(dwt_cirdiags_t *diag, dwt_acc_idx_e acc_idx);

    /*! ---------------------------------------------------------------------------------------------------
    * @brief This API will return the RSSI  - UWB channel power
    *        This API must be called only after initializing and configuring the driver
    *        and receving some Rx data packets.
    *
    * @param[in] diag: Pointer to a CIR diagnostics structure for a particular CIR/accumulator
    * @param[in] acc_idx: CIR index (see @ref dwt_acc_idx_e)
    * @param[out] signal_strength: Channel power signal strength in q8.8 format (int16_t).
    *
    * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
    */
    int dwt_calculate_rssi(const dwt_cirdiags_t *diag, dwt_acc_idx_e acc_idx, int16_t *signal_strength);

    /*! ---------------------------------------------------------------------------------------------------
    * @brief This API will return the first path signal power
    *        This API must be called only after initializing and configuring the driver
    *        and receving some RX data packets.
    *
    * @param[in] diag: Pointer to a CIR diagnostics structure for a particular CIR/accumulator
    * @param[in] acc_idx: CIR index (see @ref dwt_acc_idx_e)
    * @param[out] signal_strength: First path signal strength in q8.8 format (int16_t).
    *
    * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
    */
    int dwt_calculate_first_path_power(const dwt_cirdiags_t *diag, dwt_acc_idx_e acc_idx, int16_t *signal_strength);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the DGC_DECISION index when RX_TUNING (DGC) is enabled, this value is used to adjust the
     *        RX level and first path FP level estimation
     *        See also dwt_calculate_rssi(), dwt_calculate_first_path_power()
     *
     * @return The index value to be used in RX level and FP level formulas
     */
    uint8_t dwt_get_dgcdecision(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to enable/disable the event counter in the IC
     *
     * @param[in] enable: 1 enables (and resets), 0 disables the event counters
     *
     */
    void dwt_configeventcounters(int32_t enable);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the event counters in the IC
     *
     * @param[out] counters: Pointer to the @ref dwt_deviceentcnts_t structure which will hold the read data
     *
     */
    void dwt_readeventcounters(dwt_deviceentcnts_t *counters);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read from AON memory
     *
     * @param[in] aon_address: This is the address of the memory location to read
     *
     * @return 8-bits read from given AON memory address
     */
    uint8_t dwt_aon_read(uint16_t aon_address);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to write to AON memory
     *
     * @param[in] aon_address: This is the address of the memory location to write
     * @param[in] aon_write_data: This is the data to write
     *
     */
    void dwt_aon_write(uint16_t aon_address, uint8_t aon_write_data);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the OTP data from given address into provided array
     *
     * @param[in] address: This is the OTP address to read from
     * @param[out] array: This is the pointer to the array into which to read the data
     * @param[in] length: This is the number of 32 bit words to read (array needs to be at least this length)
     *
     */
    void dwt_otpread(uint16_t address, uint32_t* array, uint8_t length);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to program 32-bit value into the devices OTP memory.
     *
     * @param[in] value: This is the 32-bit value to be programmed into OTP
     * @param[in] address: This is the 16-bit OTP address into which the 32-bit value is programmed
     *
     * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
     */
    int32_t dwt_otpwriteandverify(uint32_t value, uint16_t address);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to program 32-bit value into the device's OTP memory, it will not validate the word was written correctly
     *
     * @param[in] value: This is the 32-bit value to be programmed into OTP
     * @param[in] address: This is the 16-bit OTP address into which the 32-bit value is programmed
     *
     * @return @ref DWT_SUCCESS
     */
    int32_t dwt_otpwrite(uint32_t value, uint16_t address);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to set up TX/RX GPIOs which could be used to control LEDs
     * @note Not completely IC dependent, also needs a board with LEDS fitted on right I/O lines.
     *       This function enables GPIOs 2 and 3 which are connected to D1 and D2 on the DW3000 CSP shield / LED2 and LED3
     *       on the DWM3000EVB and QM33120MEVB, which are connected to LED3 and LED4 on EVB1000
     *
     * @param[in] mode: This is a bit field interpreted as follows:
     *          - bit 0: 1 to enable LEDs, 0 to disable them
     *          - bit 1: 1 to make LEDs blink once on init. Only valid if bit 0 is set (enable LEDs)
     *          - bit 2 to 7: reserved
     *
     */
    void dwt_setleds(uint8_t mode);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to adjust the crystal frequency
     *
     * @note dwt_initialise() must be called prior to this function in order to initialise the local data
     *
     * @param[in] value: Crystal trim value (in range 0x0 to 0x3F) 64 steps (~1.65ppm per step)
     *
     */
    void dwt_setxtaltrim(uint8_t value);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function returns the value of XTAL trim that has been applied during initialisation (dwt_initialise()). This can
     *        be either the value read in OTP memory or a default value.
     *
     * @note The value returned by this function is the initial value only! It is not updated on dwt_setxtaltrim() calls.
     *
     * @return The XTAL trim value set upon initialisation
     */
    uint8_t dwt_getxtaltrim(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function enables preamble test mode which sends a preamble pattern for duration specified by delay parameter.
     *        Once the delay expires, the device goes back to normal mode.
     *
     * @param[in] delay: This is the duration of the preamble transmission in us
     * @param[in] test_txpower: This is the TX power to be applied while transmitting preamble
     *
     */
    void dwt_send_test_preamble(uint16_t delay, uint32_t test_txpower);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function disables repeated frames from being generated.
     *
     */
    void dwt_stop_repeated_frames(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function enables repeated frames to be generated given a frame repetition rate.
     *
     * @param[in] framerepetitionrate: Value specifying the rate at which frames will be repeated.
     *                            If the value is less than the frame duration, the frames are sent
     *                            back-to-back.
     *
     */
    void dwt_repeated_frames(uint32_t framerepetitionrate);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will enable a repeated continuous waveform on the device
     *
     * @param[in] cw_enable: CW mode enable
     * @param[in] cw_mode_config: CW configuration mode.
     *
     */
    void dwt_repeated_cw(int32_t cw_enable, int32_t cw_mode_config);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function configures the UWB radio to transmit continuous wave (CW) signal at specific channel frequency
     *
     */
    void dwt_configcwmode(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function configures the UWB radio to continuous TX frame mode for regulatory approvals testing.
     *
     * @note dwt_configure() and dwt_configuretxrf() must be called before a call to this API
     *
     * @param[in] framerepetitionrate: This is a 32-bit value that is used to set the interval between transmissions.
     *  The minimum value is 2. The units are approximately 4 ns. (or more precisely 512/(499.2e6*256) seconds)).
     *
     */
    void dwt_configcontinuousframemode(uint32_t framerepetitionrate);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function stops the continuous TX frame mode of the UWB radio.
     *
     */
    void dwt_disablecontinuousframemode(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief this function stops the continuous wave mode of the UWB radio.
     *
     */
    void dwt_disablecontinuouswavemode(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief this function reads the raw battery voltage and temperature values of the UWB IC.
     * The values read here will be the current values sampled by UWB IC AtoD converters.
     *
     * @return reading  = (temp_raw<<8)|(vbat_raw)
     */
    uint16_t dwt_readtempvbat(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief  this function takes in a raw temperature value and applies the conversion factor
     * to give true temperature. The dwt_initialise needs to be called before call to this to
     * ensure pdw3xxxlocal->tempP contains the SAR_LTEMP value from OTP.
     *
     * @param[in] raw_temp: This is the 8-bit raw temperature value as read by dwt_readtempvbat() [15:8]
     *
     * @return temperature sensor value
     */
    float dwt_convertrawtemperature(uint8_t raw_temp);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief this function takes in a raw voltage value and applies the conversion factor
     * to give true voltage. The dwt_initialise needs to be called before call to this to
     * ensure pdw3xxxlocal->vBatP contains the SAR_LVBAT value from OTP
     *
     * @param[in] raw_voltage: This is the 8-bit raw voltage value as read by dwt_readtempvbat() [7:0]
     *
     * @return: voltage sensor value
     */
    float dwt_convertrawvoltage(uint8_t raw_voltage);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief this function reads the temperature of the UWB IC that was sampled
     * on waking from Sleep/Deepsleep. They are not current values, but read on last
     * wakeup if @ref DWT_RUNSAR bit is set in mode parameter of dwt_configuresleep()
     *
     * @return 8-bit raw temperature sensor value
     */
    uint8_t dwt_readwakeuptemp(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief this function reads the battery voltage of the UWB IC that was sampled
     * on waking from Sleep/Deepsleep. They are not current values, but read on last
     * wakeup if @ref DWT_RUNSAR bit is set in mode parameter of dwt_configuresleep()
     *
     * @return 8-bit raw battery voltage sensor value
     */
    uint8_t dwt_readwakeupvbat(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief Returns the PG delay value of the TX
     *
     * @return PG delay
     */
    uint8_t dwt_readpgdelay(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function determines the adjusted bandwidth setting (PG_DELAY bitfield setting)
     * of the UWB IC. The adjustment is a result of UWB IC's internal PG cal routine, given a target count value it will try to
     * find the PG delay which gives the closest count value to the given target count.
     * Manual sequencing of TX blocks and TX clocks need to be enabled for either channel 5 or 9.
     * This function presumes that the PLL is already on (device is in the IDLE state). Please configure the device to IDLE
     * state before calling this function, by calling dwt_configure(), dwt_setchannel() or dwt_setdwstate().
     *
     * @param[in] target_count: The PG count target to reach in order to correct the bandwidth
     *
     * @return The value that was written to the PG_DELAY register (when calibration completed, [5:0])
     */
    uint8_t dwt_calcbandwidthadj(uint16_t target_count);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function calculates the value in the pulse generator counter register (PGC_STATUS) for a given PG_DELAY
     * This is used to take a reference measurement, and the value recorded as the reference is used to adjust the
     * bandwidth of the device when the temperature changes. This function presumes that the PLL is already on (device is in the IDLE state).
     *
     * @param[in] pgdly: The PG_DELAY (max value 63) to set (to control bandwidth), and to find the corresponding count value for
     *
     * @return The count value calculated from the provided PG_DELAY value (from PGC_STATUS) - used as reference
     * for later bandwidth adjustments [11:0]
     */
    uint16_t dwt_calcpgcount(uint8_t pgdly);

    /********************************************************************************************************************/
    /*                                                AES BLOCK                                                         */
    /********************************************************************************************************************/

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function provides the API for the configuration of the AES block before its first usage.
     *
     * @param[in] pCfg: Pointer to the configuration structure, which contains the AES configuration data, see @ref dwt_aes_config_t
     *
     */
    void dwt_configure_aes(dwt_aes_config_t *pCfg);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief This gets the mic size in bytes and convert it to value to write in AES_CFG
    *
    * @param[in] mic_size_in_bytes: mic size in bytes.
    *
    * @return  @ref dwt_mic_size_e value
    */
    dwt_mic_size_e dwt_mic_size_from_bytes(uint8_t mic_size_in_bytes);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function provides the API for the configuration of the AES key before the first usage.
     *
     * @param[in] key: Pointer to the key which will be programmed to the AES_KEY register, see @ref dwt_aes_key_t
     *                 Note the AES_KEY register supports only 128-bit keys.
     *
     */
    void dwt_set_keyreg_128(dwt_aes_key_t *key);

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
     * @param[in] job: Pointer to AES job, contains data info and encryption info, see @ref dwt_aes_job_t
     * @param[in] core_type: Core type (CCM* or GCM), see @ref dwt_aes_core_type_e
     *
     * @return  AES_STS_ID status bits
     */
    int8_t dwt_do_aes(dwt_aes_job_t *job, dwt_aes_core_type_e core_type);

	/********************************************************************************************************************/
	/*             Declaration of platform-dependent lower level functions.                                             */
	/********************************************************************************************************************/

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief  This function wakeup device by an IO pin. SPI_CS or WAKEUP pins can be used for this, as
     *         configured by dwt_configuresleep(), @ref dwt_wkup_param_e
     *
     */
    void dwt_wakeup_ic(void);

    // ---------------------------------------------------------------------------
    //
    // NB: The purpose of the deca_mutex.c file is to provide for microprocessor interrupt enable/disable, this is used for
    //     controlling mutual exclusion from critical sections in the code where interrupts and background
    //     processing may interact.  The code using this is kept to a minimum and the disabling time is also
    //     kept to a minimum, so blanket interrupt disable may be the easiest way to provide this.  But at a
    //     minimum those interrupts coming from the Decawave device should be disabled/re-enabled by this activity.
    //
    //     In porting this to a particular microprocessor, the implementer may choose to use #defines here
    //     to map these calls transparently to the target system.  Alternatively the appropriate code may
    //     be embedded in the functions provided in the deca_irq.c file.
    //
    // ---------------------------------------------------------------------------

    typedef int32_t decaIrqStatus_t; // Type for remembering IRQ status

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function should disable interrupts. This is called at the start of a critical section
     * It returns the IRQ state before disable, this value is used to re-enable in decamutexoff call
     *
     * @note The body of this function is defined in deca_mutex.c and is platform specific
     *
     * @return the state of the UWB IC interrupt
     */
    decaIrqStatus_t decamutexon(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function should re-enable interrupts, or at least restore their state as returned(&saved) by decamutexon
     * This is called at the end of a critical section
     *
     * @note The body of this function is defined in deca_mutex.c and is platform specific
     *
     * @param[in] s: The state of the UWB IC interrupt as returned by decamutexon
     *
     * @return the state of the UWB IC interrupt
     */
    void decamutexoff(decaIrqStatus_t s);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief Wait for a given amount of time.
     * NB: The body of this function is defined in deca_sleep.c and is platform specific
     *
     * @param[in] time_ms: Time to wait in milliseconds
     *
     */
    void deca_sleep(unsigned int time_ms);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief Wait for a given amount of time.
     * NB: The body of this function is defined in deca_sleep.c and is platform specific
     *
     * @param[in] time_us: Time to wait in microseconds
     *
     */
    void deca_usleep(unsigned long time_us);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This reads the device ID and checks if it is the right one, i.e., if the driver matches the device.
     *
     * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
     */
    int32_t dwt_check_dev_id(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     *
     * @brief This function runs the PGF calibration. This is needed prior to reception.
     *
     * @note If the RX calibration routine fails the device receiver performance will be severely affected, the application should reset and try again
     *
     * @return result of PGF calibration (@ref dwt_error_e DWT_ERR_RX_CAL*)
     *
     */
    int32_t dwt_run_pgfcal(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function runs the PGF calibration. This is needed prior to reception.
     *
     * @note If the RX calibration routine fails the device receiver performance will be severely affected, the application should reset and try again
     *
     * @param[in] ldoen: If set to 1 the function will enable LDOs prior to calibration and disable afterwards.
     *
     * @return result of PGF calibration (@ref dwt_error_e DWT_ERR_RX_CAL*)
     *
     */
    int32_t dwt_pgf_cal(int32_t ldoen);

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
     * @return A value containing the value of the PLL status register (only bits [14:0] are valid)
     */
    uint32_t dwt_readpllstatus(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will re-calibrate and re-lock the PLL. If the cal/lock is successful @ref DWT_SUCCESS
     * will be returned otherwise @ref DWT_ERROR will be returned
     *
     * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
     */
    int32_t dwt_pll_cal(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function is used to control what RF port to use for TX/RX.
     *
     * @param[in] port_control: Enum value for enabling or disabling manual control and primary antenna, see @ref dwt_rf_port_ctrl_e
     *
     */
    void dwt_configure_rf_port(dwt_rf_port_ctrl_e  port_control);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function configures just the SFD type: e.g., IEEE 4a - 8, DW-8, DW-16, or IEEE 4z -8 (binary)
     * The dwt_configure() should be called prior to this to configure other parameters
     *
     * @param[in] sfdType: e.g, @ref DWT_SFD_IEEE_4A, @ref DWT_SFD_DW_8, @ref DWT_SFD_DW_16, @ref DWT_SFD_IEEE_4Z, see @ref dwt_sfd_type_e
     *
     */
    void dwt_configuresfdtype(dwt_sfd_type_e sfdType);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function just sets the TX preamble code: 1-8 PRF16 9-24 PRF64
     * The dwt_configure() should be called prior to this to configure other parameters
     *
     * @param[in] tx_code: 1-8 PRF16, 9-24 PRF64
     *
     */
    void dwt_settxcode(uint8_t tx_code);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function just sets the RX preamble code: 1-8 PRF16 9-24 PRF64
     * The dwt_configure() should be called prior to this to configure other parameters
     *
     * @param[in] rx_code: 1-8 PRF16, 9-24 PRF64
     *
     */
    void dwt_setrxcode(uint8_t rx_code);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief  this function allows read from the DW3xxx device 32-bit register
    *
    * @param[in] address: ID of the DW3xxx register
    *
    * @return the value of the 32-bit register
    */
    uint32_t dwt_read_reg(uint32_t address);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief  this function allows write to the DW3xxx device 32-bit register
    *
    * @param[in] address: ID of the DW3xxx register
    * @param[in] data: Value to write
    *
    */
    void dwt_write_reg(uint32_t address, uint32_t data);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function writes a value to the system status register (lower).
     *
     * @param[in] mask: mask value to send to the system status register (lower 32-bits).
     *               e.g., "SYS_STATUS_TXFRS_BIT_MASK" to clear the TX frame sent event.
     *
     */
    void dwt_writesysstatuslo(uint32_t mask);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function writes a value to the system status register (higher).
     *
     * @param[in] mask: Mask value to send to the system status register (higher bits).
     *               @note Be aware that the size of this register varies per device.
     *               DW3000 devices only require a 16-bit mask value typecast to 32-bit.
     *               DW3720 devices require a 32-bit mask value.
     *
     */
    void dwt_writesysstatushi(uint32_t mask);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function reads the current value of the system status register (lower 32 bits)
     *
     * @return A uint32_t value containing the value of the system status register (lower 32 bits)
     */
    uint32_t dwt_readsysstatuslo(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function reads the current value of the system status register (higher bits)
     *
     *
     * @return A uint32_t value containing the value of the system status register (higher bits)
     *        @note Be aware that the size of this register varies per device.
     *        DW3000 devices will return a 16-bit value of the register that is typecast to a 32-bit value.
     *        DW3720 devices will return a 'true' 32-bit value of the register.
     */
    uint32_t dwt_readsysstatushi(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function writes a value to the receiver double buffer status register.
     *
     * @param[in] mask: Mask value to send to the register.
     *               e.g., "RDB_STATUS_CLEAR_BUFF0_EVENTS" to clear the clear buffer 0 events.
     *
     */
    void dwt_writerdbstatus(uint8_t mask);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function reads the current value of the Receiver Double Buffer status register.
     *
     * @return The value of the Receiver Double Buffer status register.
     */
    uint8_t dwt_readrdbstatus(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will read the frame length of the last received frame.
     *        This function presumes that a good frame or packet has been received.
     *
     * @param[out] rng: This is an output, the parameter will have DWT_CB_DATA_RX_FLAG_RNG set if RNG bit is set in FINFO
     *
     * @return the number of octets in the received frame.
     */
    uint16_t dwt_getframelength(uint8_t *rng);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to read the value stored in CIA_ADJUST_ID register
     *
     * @return value stored in CIA_ADJUST_ID register
     */
    uint32_t dwt_readpdoaoffset(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will set the value to the CIA_ADJUST_ID register.
     *
     * @param[in] offset: The offset value to be written into the CIA_ADJUST_ID register.
     *
     */
    void dwt_setpdoaoffset(uint16_t offset);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function can set GPIO output to high (1) or low (0) which can then be used to signal e.g., WiFi chip to
     * turn off or on. This can be used in devices with multiple radios to minimise co-existence interference.
     *
     * @param[in] enable: Specifies if to enable or disable WiFi co-ex functionality on GPIO5 (or GPIO4)
     *                       depending if coex_io_swap param is set to 1 or 0
     * @param[in] coex_io_swap: When set to 0, GPIO5 is used as co-ex out, otherwise GPIO4 is used
     *
     * @return event counts from both timers: @ref DWT_TIMER0 events in bits [7:0], @ref DWT_TIMER1 events in bits [15:8]
     */
    void dwt_wifi_coex_set(dwt_wifi_coex_e enable, int32_t coex_io_swap);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will reset the internal system time counter. The counter will be momentarily reset to 0,
     * and then will continue counting as normal. The system time/counter is only available when device is in
     * IDLE or TX/RX states.
     *
     */
    void dwt_reset_system_counter(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function is used to configure the device for OSTR mode (One Shot Timebase Reset mode), this will
     * prime the device to reset the internal system time counter on SYNC pulse / SYNC pin toggle.
     * For more information on this operation please consult the device User Manual.
     *
     * @param[in] enable:    Set to 1 to enable OSTR mode and 0 to disable
     * @param[in] wait_time: When a counter running on the 38.4 MHz external clock and initiated on the rising edge
     *                       of the SYNC signal equals the WAIT programmed value, the UWB IC timebase counter will be reset.
     *
     * @note At the time the SYNC signal is asserted, the clock PLL dividers generating the UWB IC 125 MHz system clock are reset,
     * to ensure that a deterministic phase relationship exists between the system clock and the asynchronous 38.4 MHz external clock.
     * For this reason, the WAIT value programmed will dictate the phase relationship and should be chosen to give the
     * desired phase relationship, as given by WAIT modulo 4. A WAIT value of 33 decimal is recommended,
     * but if a different value is chosen it should be chosen so that WAIT modulo 4 is equal to 1, i.e., 29, 37, and so on.
     *
     */
    void dwt_config_ostr_mode(uint8_t enable, uint16_t wait_time);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function is used for specific customer hardware/modules where antenna selection switch is connected
     *        to GPIO6 and GPIO7. This function configures the GPIOs to give particular antenna selecton.
     *
     * @param[in] antenna_config: Configures GPIO6 and/or GPIO7 to give specific antenna selection
     *      bitfield configuration:
     *      Bit 0: Set GPIO6 as output
     *      Bit 1: Value to apply for GPIO6 (0/1)
     *      Bit 2: Set GPIO7 as output
     *      Bit 3: Value to apply for GPIO7  (0/1)
     *
     */
    void dwt_configure_and_set_antenna_selection_gpio(uint8_t antenna_config);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will read Ipatov, STS1 and STS2 diagnostics registers from the device, which can help in
     *        determining if packet has been received in LOS (line-of-sight) or NLOS (non-line-of-sight) condition.
     *        To help determine/estimate NLOS condition either Ipatov, STS1 or STS2 can be used, (or all three).
     *
     * @note  CIA diagnostics need to be enabled with @ref DW_CIA_DIAG_LOG_ALL, otherwise the diagnostic registers read will be 0,
     *        please see dwt_configciadiag().
     *
     * @param[out] all_diag: This is the pointer to the dwt_nlos_alldiag_t structure into which to read the data.
     *
     * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
     */
    int dwt_nlos_alldiag(dwt_nlos_alldiag_t *all_diag);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will read the Ipatov diagnostic registers to get the first path (FP) and peak path (PP) index value.
     *        This function is used when signal power is low to determine the signal type (LOS or NLOS). Hence only
     *        Ipatov diagnostic registers are used to determine the signal type.
     *
     * @param[out] index: This is the pointer to the dwt_nlos_ipdiag_t structure into which to read the data.
     *
     */
    void dwt_nlos_ipdiag(dwt_nlos_ipdiag_t *index);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief This is used to read the value stored in CTR_DBG_ID register, these are the low 32-bits of the STS IV counter.
    *
    * @return value stored in CTR_DBG_ID register
    */
    uint32_t dwt_readctrdbg(void);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief This is used to read the value stored in DGC_DBG_ID register.
    *
    * @return value stored in DGC_DBG_ID register
    */
    uint32_t dwt_readdgcdbg(void);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief This function reads the CIA version in this device.
    *
    */
    uint32_t dwt_readCIAversion(void);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief This is used to return base address of ACC_MEM_ID register (CIR base address)
    *
    * @return address of ACC_MEM_ID register
    */
    uint32_t dwt_getcirregaddress(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This API enables returns a list of register name/value pairs, to enable debug output / logging in external applications
     * e.g., DecaRanging application
     *
     * @return Pointer to registers structure @ref register_name_add_t
     */
    register_name_add_t* dwt_get_reg_names(void);

    /*! 
     * @brief This function calculates the adjusted Tx power setting by applying a boost over a reference Tx power setting.
     * The reference Tx power setting should correspond to a 1 ms frame (or 0 dB boost).
     * The boost (power increase) to be applied should be provided in unit of 0.1dB.
     * For example, for a 125 us packet, a theoretical boost of 9 dB can be applied, compared to a packet of 1 ms.
     * A boost of '90' should be provided as input parameter and will be applied over the reference Tx power setting for a 1 ms frame.
     *
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
    int32_t dwt_adjust_tx_power(uint16_t boost, uint32_t ref_tx_power, uint8_t channel, uint32_t* adj_tx_power, uint16_t* applied_boost);

   /*! 
    * @brief This API is used to calculate a transmit power configuration in a linear manner (step of 0.25 dB)
    *
    *
    * @param[in] channel:   The channel for which the linear tx power setting must be calculated
    * @param[in] p_indexes: Pointer to an object containing two members:
    * @parblock      
    *                       - in : The inputs indexes for which tx power configuration must be calculated.
    *                          This is an array of size 4 allowing to set individual indexes for each section of a frame.
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
    *                       - out: output tx power indexes corresponding to the indexes that were actually applied.
    *                         These may differ from the input indexes in the two cases below:
    *                             1. The input indexes correspond to different PLLCFG for different section of frame. This is not
    *                              supported by the IC. In such condition, all indexes will belong to the same tx configuration state. The state
    *                              to be used is the one corresponding to the highest required power (lower index)
    *
    *                              2. The input index are greater than the maximum value supported for the (channel, SOC) current
    *                              configuration. In which case, the maximum index supported is returned.
    * @endparblock
    * @param[out] p_res: Pointer to an object containing the output configuration corresponding to input index.
    * @parblock
    *                     The object contain two members:
    *                     - tx_frame_cfg: the power configuration to use during frame transmission
    *
    *                     A configuration is a combination of two parameters:
    *                     - uint32_t tx_power_setting: Tx power setting to be written to register TX_POWER_ID
    *                     - uint8_t pll_bias: PLL bias trim configuration to be written to register PLL_COMMON_ID
    * @endparblock
    * @return
    * @ref DWT_SUCCESS if an adjusted tx power setting could be calculated. In this case, the configuration to be applied is return through
    * p_res parameter.
    * @ref DWT_ERROR if the API could not calculate a valid configuration.
    *
    */
    int32_t dwt_calculate_linear_tx_power(uint32_t channel, power_indexes_t *p_indexes, tx_adj_res_t *p_res);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief This API is used to convert a transmit power value into its corresponding Tx power index.
    *
    * @param[in] channel: The channel for which the Tx power index must be calculated
    * @param[in] tx_power: The Transmit power to convert
    * @param[out] tx_power_idx: Pointer to the returned TX power index
    *
    * @return
    * @ref DWT_SUCCESS if the conversion succeeded
    * @ref DWT_ERROR if the API could not calculate a valid configuration.
    */
    int dwt_convert_tx_power_to_index(uint32_t channel, uint8_t tx_power, uint8_t *tx_power_idx);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief This function will set the PLL bias trim value in PLL_COMMON register.
    *
    * @note If the RX is required, to provide optimal performance, the PLL bias trim should be set to the default value.
    * The @ref dwt_rxenable API and @ref dwt_isr will do this automatically. 
    * If an external ISR is used and the RX is required after a TX, the application should set the PLL bias trim to the default 
    * value manually.
    * Use @ref DWT_DEF_PLLBIASTRIM to restore the default value.
    * 
    * @param[in] pll_bias_trim: PLL bias trim.
    *
    */
    void dwt_setpllbiastrim(uint8_t pll_bias_trim);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief This function sets the ISR configuration flags.
    *
    * @param[in] flags: ISR configuration flags (see dwt_isr_flags_e)
    *
    */
    void dwt_configureisr(dwt_isr_flags_e flags);
    /* BEGIN: CHIP_SPECIFIC_SECTION DW3720 */

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function enables the specified double RX buffer to trigger an interrupt.
     * The following events can be found in RDB_STAT_EN_ID registers.
     *
     * @param[in] bitmask: Sets the events in RDB_STAT_EN_ID register which will generate interrupt
     * @param[in] INT_options: If set to @ref DWT_ENABLE_INT additional interrupts as selected in the bitmask are enabled
     *                       If set to @ref DWT_ENABLE_INT_ONLY the interrupts in the bitmask are forced to selected state
     *                       i.e., the mask is written to the register directly.
     *                       Otherwise (if set to @ref DWT_DISABLE_INT) the interrupts as selected in the bitmask are cleared
     *
     */
    void dwt_setinterrupt_db(uint8_t bitmask, dwt_INT_options_e INT_options);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief Request access to the device registers, using the dual SPI semaphore request command. If the semaphore is available,
     * the semaphore will be given.
     *
     * @note dwt_ds_sema_status() should be used to get semaphore status value.
     *
     */
    void dwt_ds_sema_request(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief Release the semaphore that was taken by this host
     *
     */
    void dwt_ds_sema_release(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This can be used by host on the SPI2 to force taking of the semaphore. Take semaphore even if it is not available.
     *        This does not apply to host on SPI1, only host on SPI2 can force taking of the semaphore.
     *
     */
    void dwt_ds_sema_force(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief Reports the semaphore status low byte.
     *
     * @return Semaphore value
     */
    uint8_t dwt_ds_sema_status(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief Reports the semaphore status high byte.
     *
     * @return Semaphore value
     */
    uint8_t dwt_ds_sema_status_hi(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief With this API each host can prevent the device going into Sleep/Deepsleep state.
     * By default it is possible for either host to place the device into Sleep/Deepsleep. This may not be desirable,
     * thus a host once it is granted access can set a SLEEP_DISABLE bit in the register
     * to prevent the other host from putting the device to sleep once it gives up its access.
     *
     * @param[in] host_sleep_en: @ref HOST_EN_SLEEP clears the bit allowing the the device to go to sleep.
     *                        @ref HOST_DIS_SLEEP sets the bit to prevent the device from going to sleep. See @ref dwt_host_sleep_en_e.
     *
     */
    void dwt_ds_en_sleep(dwt_host_sleep_en_e host_sleep_en);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief With this API the host on either SPI1 or SPI2 can enable/disable whether the interrupt is raised upon
    * SPI1MAVAIL or SPI2MAVAIL event.
    *
    * @param[in] spi_num: Should be set to either @ref DWT_HOST_SPI1 or @ref DWT_HOST_SPI2, see @ref dwt_spi_host_e
    * @param[in] int_set: Should be set to either @ref DWT_ENABLE_INT or @ref DWT_DISABLE_INT, see @ref dwt_INT_options_e
    *
    * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error (if input parameters not correct)
    */
    int32_t dwt_ds_setinterrupt_SPIxavailable(dwt_spi_host_e spi_num, dwt_INT_options_e int_set);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function enables or disables the equaliser block within in the CIA. The equaliser should be used when
     * receiving from devices which transmit using a Symmetric Root Raised Cosine pulse shape. The equaliser will adjust
     * the CIR to give improved receive timestamp results. Normally, this is left disabled (the default value), which
     * gives the best receive timestamp performance when interworking with devices (like this UWB IC) that use the
     * IEEE 802.15.4z recommended minimum precursor pulse shape.
     *
     * @param[in] en: @ref DWT_EQ_ENABLED or @ref DWT_EQ_DISABLED, enables/disables the equaliser block
     *
     */
    void dwt_enable_disable_eq(uint8_t en);
    /* END: CHIP_SPECIFIC_SECTION DW3720 */

    /* BEGIN: CHIP_SPECIFIC_SECTION DW3720 */
    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will reset the timers block. It will reset both timers. It can be used to stop a timer running
     * in repeat mode.
     *
     */
    void dwt_timers_reset(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will read the timers' event counts. When reading from this register the values will be reset/cleared,
     * thus the host needs to read both timers' event counts the events relating to @ref DWT_TIMER0 are in bits [7:0] and events
     * relating to @ref DWT_TIMER1 in bits [15:8].
     *
     * @return event counts from both timers: @ref DWT_TIMER0 events in bits [7:0], @ref DWT_TIMER1 events in bits [15:8]
     */
    uint16_t dwt_timers_read_and_clear_events(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function configures selected timer (@ref DWT_TIMER0 or @ref DWT_TIMER0) as per configuration structure
     *
     * @param[in] tim_cfg: Pointer to timer configuration structure, see @ref dwt_timer_cfg_t
     *
     */
    void dwt_configure_timer(dwt_timer_cfg_t *tim_cfg);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function configures the GPIOs (4 and 5) for COEX_OUT
     *
     * @param[in] timer_coexout: Configure if timer controls the COEX_OUT
     * @param[in] coex_swap: Configures if the COEX_OUT is on GPIO4 or GPIO5, when set to 1 the GPIO4 will be COEX_OUT
     *
     */
    void dwt_configure_wificoex_gpio(uint8_t timer_coexout, uint8_t coex_swap);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function sets timer expiration period, it is a 22-bit number
     *
     * @param[in] timer_name: Specify which timer period to set: @ref DWT_TIMER0 or @ref DWT_TIMER1, see @ref dwt_timers_e
     * @param[in] expiration: Expiry count - e.g., if units are XTAL/64 (1.66 us) then setting 1024 ~= 1.7 ms period
     *
     */
    void dwt_set_timer_expiration(dwt_timers_e timer_name, uint32_t expiration);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function enables the timer. In order to enable, the timer enable bit [0] for TIMER0 or [1] for TIMER1
     * needs to transition from 0->1.
     *
     * @param[in] timer_name: Specifies which timer to enable, see @ref dwt_timers_e
     *
     */
    void dwt_timer_enable(dwt_timers_e timer_name);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This API enables "Fixed STS" function. The fixed STS function means that the same STS will be sent in each packet.
     * And also in the receiver, when the receiver is enabled the STS will be reset. Thus transmitter and the receiver will be in sync.
     *
     * @param[in] set: Set to 1 to set FIXED STS and 0 to disable
     *
     */
    void dwt_set_fixedsts(uint8_t set);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This API sets the alternative pulse shape according to ARIB.
     *
     * @param[in] set_alternate: Set to 1 to enable the alternate pulse shape and 0 to restore default shape.
     *
     */
    void dwt_set_alternative_pulse_shape(uint8_t set_alternate);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will configure the channel number.
     *
     * @param[in] ch: Channel number (5 or 9)
     *
     * @return @ref DWT_SUCCESS for success, or @ref DWT_ERR_PLL_LOCK for error
     */
    int32_t dwt_setchannel(dwt_pll_ch_type_e ch);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will set the STS length. The STS length is specified in blocks. A block is 8 us duration,
     *        0 value specifies 8 us duration, the max value is 255 == 2048 us.
     *
     *        @note dwt_configure() must be called prior to this to configure the PRF
     *        To set the PRF dwt_settxcode() is used.
     *
     *
     * @param[in] stsblocks: Number of STS 8us blocks (0 == 8us, 1 == 16us, etc)
     *
     */
    void dwt_setstslength(uint8_t stsblocks);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This is used to enable/disable the FCS generation in transmitter and checking in receiver
     *
     * @param[in] enable: @ref dwt_fcs_mode_e, This is used to enable/disable FCS generation in TX and check in RX
     *
     */
    void dwt_configtxrxfcs(uint8_t enable);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will configure the PHR mode and rate.
     * @note Normally the PHR mode and PHR rate are configured with dwt_configure() with the rest of the receiver configurations.
     *
     * @param[in] phrMode: PHR mode {0x0 - standard @ref DWT_PHRMODE_STD, 0x1 - extended frames @ref DWT_PHRMODE_EXT}
     * @param[in] phrRate: PHR rate {0x0 - standard @ref DWT_PHRRATE_STD, 0x1 - at datarate @ref DWT_PHRRATE_DTA}
     *
     * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
     */
    int32_t dwt_setphr(dwt_phr_mode_e phrMode, dwt_phr_rate_e phrRate);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will configure the data rate.
     * @note Normally the data rate is configured with dwt_configure() with the rest of the receiver configurations.
     *
     * @param[in] bitRate: Data rate see @ref dwt_uwb_bit_rate_e, depends on PRF
     *
     * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error
     */
    int32_t dwt_setdatarate(dwt_uwb_bit_rate_e bitRate);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will configure the Ipatov Preamble Acquisition Chunk (PAC) size.
     * @note Normally the PAC is configured with dwt_configure() with the rest of the receiver configurations.
     *
     * @param[in] rxPAC: Acquisition Chunk Size (Relates to the reception of RX preamble length), please see dwt_pac_size_e
     *
     * @return @ref DWT_SUCCESS
     */
    int32_t dwt_setrxpac(dwt_pac_size_e rxPAC);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will configure the SFD timeout value.
     * @note Normally the SFD timeout is configured with dwt_configure() with the rest of the receiver configurations.
     *
     * @param[in] sfdTO: SFD timeout value (in symbols)
     *
     * @return @ref DWT_SUCCESS
     */
    int32_t dwt_setsfdtimeout(uint16_t sfdTO);

    /* @brief This function disables integrated power supply (IPS) and needs to be called prior to
     *        the device going to sleep or when an OTP low power mode is required.
     *
     * @note Synopsys have released an errata against the OTP power macro, in relation to low power
     *        SLEEP/RETENTION modes where the VDD and VCC are removed.
     *        On exit from low-power mode this is called to restore/enable the OTP IPS for normal OTP use (RD/WR).
     *
     * @param[in] mode: If set to 1 this will configure OTP for low-power
     *
     */
    void dwt_disable_OTP_IPS(int32_t mode);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function runs the PLL calibration using a SW method. It allows user to pass various parameters as below.
     *
     * @param[in] chan: (5 = channel 5, 9 = channel 9)
     * @param[in] coarse_code: VCO coarse tune code
     * @param[in] sleep: delay to wait in microseconds between each step
     * @param[in] steps: the max number of steps over which to run PLL cal
     * @param[in] temperature: the current temperature (temp at which the cal is run), it is only used for channel 5
     *            If @ref TEMP_INIT is passed it will read temperature from the device using dwt_readtempvbat()
     *
     * @return Number of steps taken to lock the PLL or @ref DWT_ERR_PLL_LOCK
     */
    uint8_t dwt_pll_chx_auto_cal(int32_t chan, uint32_t coarse_code, uint16_t sleep, uint8_t steps, int8_t temperature);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function sets the crystal trim based on temperature, and crystal temperature
     *        characteristics, if you pass in a temperature of @ref TEMP_INIT (-127), the functions will also read
     *        onchip temperature sensors to determine the temperature, the crystal temperature
     *        could be different.
     * @parblock
     *        If a crystal temperature of @ref TEMP_INIT (-127) is passed, the function will assume 25C.
     *        If a crystal trim of 0 is passed, the function will use the calibration value from OTP.
     *
     *        This is to compensate for crystal temperature versus frequency curve e.g.,
     *
     * @verbatim
     *  Freq Hi
     *         *                               /
     *         *       __-__                  /
     *         *     /       \               /
     *         *   /           \            /
     *         *  /             \          /
     *         * /               \       /
     *         *                   -___-
     *         *
     *  Freq Lo  -40 -20   0  20  40  60  80 100 120
     * @endverbatim
     *
     * @endparblock
     * @param[in] params: The based-on parameters to set the new crystal trim.
     * @param[in] xtaltrim: Newly programmed crystal trim value
     *
     * @return @ref DWT_SUCCESS for success, or @ref DWT_ERROR for error on invalid parameters.
     */
    int32_t dwt_xtal_temperature_compensation(dwt_xtal_trim_t *params, uint8_t *xtaltrim);

    /*! ------------------------------------------------------------------------------------------------------------------
    * @brief This function captures the ADC samples on receiving a signal.
    *
    * @param[out] capture_adc: This is the pointer to the structure (@ref dwt_capture_adc_t) into which to read the data.
    *
    */
    void dwt_capture_adc_samples(dwt_capture_adc_t *capture_adc);

    /*! ---------------------------------------------------------------------------------------------------
     * @brief This function reads the captured ADC samples.
     *        This needs to be called after @ref dwt_capture_adc_samples()
     *        The test block will write to 255 (0xff) locations in the CIR memory (Ipatov).
     *        Each clock cycle produces 36 ADC bits so 8 clocks will fill a memory word.
     *        The ADC capture memory pointer increments twice each write, until the 255 limit is reached - the pointer will be at 0x1fe.
     *        If wrapped mode is used the poiner will wrap around after reaching the 255 limit.
     *        There are 36*8*255=73440 ADC bits logged (18-bit I and 18 bit Q ... but that is actually 16-bit I and 16-bit Q ... as the top two bits are 0).
     *        16-bit I and 16-bit Q are saved into 2 24-bit words: [0] = 0x00NNPP, [1] = 0x00NNPP,
     *        [0]: NN = I-, PP = I+, [1]: NN = Q-, PP = Q+,
     *        (18360 I samples and 18360 Q samples).
     *
     * @param[out] capture_adc: This is the pointer to the @ref dwt_capture_adc_t structure into which to read the data.
     *
     */
    void dwt_read_adc_samples(dwt_capture_adc_t *capture_adc);

    /* END: CHIP_SPECIFIC_SECTION DW3720 */

    /*! @name Device Data for DW3720 and DW3000 Transceiver control
    *@{*/
    typedef struct
    {
        uint64_t lotID;                         // IC Lot ID - read during initialisation
        uint32_t partID;                        // IC Part ID - read during initialisation
        uint8_t bias_tune;                      // Bias tune code - currently only used in DW3000
        dwt_dgc_load_location dgc_otp_set;      // Flag to check if DGC values are programmed in OTP
        uint8_t vBatP;                          // IC V bat read during production and stored in OTP (Vmeas @ 3.0V)
        uint8_t tempP;                          // IC temp read during production and stored in OTP (Tmeas @ 22C)
        int8_t  temperature;                    // Temperature to use for PLL cal, if set to TEMP_INIT (-127) dwt_configure() will attempt to measure the temperature using onboard sensor
        uint8_t vdddig_otp;                     // Value of VDDDIG in OTP
        uint8_t vdddig_current;                 // Value of VDDDIG in AON (currently configured)
        uint8_t longFrames;                     // Flag in non-standard long frame mode
        uint8_t otprev;                         // OTP revision number (read during initialisation)
        uint8_t init_xtrim;                     // Initial XTAL trim value read from OTP (or defaulted to mid-range if OTP not programmed)
        uint8_t dblbuffon;                      // Double RX buffer mode and DB status flag
        uint8_t channel;                        // Current channel the PLL is configured for
        uint16_t sleep_mode;                    // Used for automatic reloading of LDO tunes and other configs from OTP at wake-up
        int16_t ststhreshold;                   // Threshold for deciding if received STS is good or bad
        dwt_spi_crc_mode_e spicrc;              // Use SPI CRC when this flag is true
        uint8_t stsconfig;                      // STS configuration mode
        uint8_t cia_diagnostic;                 // CIA dignostic logging level
        dwt_cb_data_t cbData;                   // Callback data structure
        uint8_t sys_cfg_dis_fce_bit_flag;       // Cached value of the SYS_CFG_DIS_FCE_BIT in the SYS_CFG_ID register
        dwt_pdoa_mode_e pdoaMode;               // Cached value of the PDOA mode
        dwt_sts_lengths_e stsLength;            // Current STS length
        uint32_t adc_zero_thresholds;           // Value of ADC zero thresholds
        uint32_t otp_ldo_tune_lo;               // LDO tune low value
        dwt_pll_prebuf_cfg_e pll_rx_prebuf_cfg; // PLL RX prebuf configuration
        uint32_t coarse_code_pll_cal_ch5;       // Coarse code value used for PLL calibration, first read from OTP and update if necessary during PLL calibration for channel 5
        uint32_t coarse_code_pll_cal_ch9;       // Coarse code value used for PLL calibration, first read from OTP and update if necessary during PLL calibration for channel 9
        uint8_t pll_bias_trim;                  // PLL bias trim value
    }dwt_local_data_t;
    /**@}*/
#ifdef __cplusplus
}
#endif

#endif /* DECA_DEVICE_API_H */
