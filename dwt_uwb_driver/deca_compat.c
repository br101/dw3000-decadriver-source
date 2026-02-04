/**
 * @file:     deca_compat.c
 *
 * @brief     This file is a wrapper which sits on top of the low-level driver file, it will call either DW3000 or DW3720
 *            APIs depending which device was discovered by the dwt_probe() function.
 *
 * @note Requirements to the application:
 * Call the dwt_probe() at the start of the application to select the correct driver.
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 */
#include "deca_device_api.h"
#include "deca_interface.h"
#include "deca_version.h"
#include "deca_private.h"

/*! The device ID regiser address, common to all QM33xxx/DW3xxx devices */
#define DW3XXX_DEVICE_ID (0x0)

#ifdef DWT_ENABLE_CRC
// Static CRC-8 ATM implementation.
// clang-format off
static const uint8_t crcTable[256] = {
        0x00U, 0x07U, 0x0EU, 0x09U, 0x1CU, 0x1BU, 0x12U, 0x15U, 0x38U, 0x3FU, 0x36U, 0x31U, 0x24U, 0x23U, 0x2AU, 0x2DU,
        0x70U, 0x77U, 0x7EU, 0x79U, 0x6CU, 0x6BU, 0x62U, 0x65U, 0x48U, 0x4FU, 0x46U, 0x41U, 0x54U, 0x53U, 0x5AU, 0x5DU,
        0xE0U, 0xE7U, 0xEEU, 0xE9U, 0xFCU, 0xFBU, 0xF2U, 0xF5U, 0xD8U, 0xDFU, 0xD6U, 0xD1U, 0xC4U, 0xC3U, 0xCAU, 0xCDU,
        0x90U, 0x97U, 0x9EU, 0x99U, 0x8CU, 0x8BU, 0x82U, 0x85U, 0xA8U, 0xAFU, 0xA6U, 0xA1U, 0xB4U, 0xB3U, 0xBAU, 0xBDU,
        0xC7U, 0xC0U, 0xC9U, 0xCEU, 0xDBU, 0xDCU, 0xD5U, 0xD2U, 0xFFU, 0xF8U, 0xF1U, 0xF6U, 0xE3U, 0xE4U, 0xEDU, 0xEAU,
        0xB7U, 0xB0U, 0xB9U, 0xBEU, 0xABU, 0xACU, 0xA5U, 0xA2U, 0x8FU, 0x88U, 0x81U, 0x86U, 0x93U, 0x94U, 0x9DU, 0x9AU,
        0x27U, 0x20U, 0x29U, 0x2EU, 0x3BU, 0x3CU, 0x35U, 0x32U, 0x1FU, 0x18U, 0x11U, 0x16U, 0x03U, 0x04U, 0x0DU, 0x0AU,
        0x57U, 0x50U, 0x59U, 0x5EU, 0x4BU, 0x4CU, 0x45U, 0x42U, 0x6FU, 0x68U, 0x61U, 0x66U, 0x73U, 0x74U, 0x7DU, 0x7AU,
        0x89U, 0x8EU, 0x87U, 0x80U, 0x95U, 0x92U, 0x9BU, 0x9CU, 0xB1U, 0xB6U, 0xBFU, 0xB8U, 0xADU, 0xAAU, 0xA3U, 0xA4U,
        0xF9U, 0xFEU, 0xF7U, 0xF0U, 0xE5U, 0xE2U, 0xEBU, 0xECU, 0xC1U, 0xC6U, 0xCFU, 0xC8U, 0xDDU, 0xDAU, 0xD3U, 0xD4U,
        0x69U, 0x6EU, 0x67U, 0x60U, 0x75U, 0x72U, 0x7BU, 0x7CU, 0x51U, 0x56U, 0x5FU, 0x58U, 0x4DU, 0x4AU, 0x43U, 0x44U,
        0x19U, 0x1EU, 0x17U, 0x10U, 0x05U, 0x02U, 0x0BU, 0x0CU, 0x21U, 0x26U, 0x2FU, 0x28U, 0x3DU, 0x3AU, 0x33U, 0x34U,
        0x4EU, 0x49U, 0x40U, 0x47U, 0x52U, 0x55U, 0x5CU, 0x5BU, 0x76U, 0x71U, 0x78U, 0x7FU, 0x6AU, 0x6DU, 0x64U, 0x63U,
        0x3EU, 0x39U, 0x30U, 0x37U, 0x22U, 0x25U, 0x2CU, 0x2BU, 0x06U, 0x01U, 0x08U, 0x0FU, 0x1AU, 0x1DU, 0x14U, 0x13U,
        0xAEU, 0xA9U, 0xA0U, 0xA7U, 0xB2U, 0xB5U, 0xBCU, 0xBBU, 0x96U, 0x91U, 0x98U, 0x9FU, 0x8AU, 0x8DU, 0x84U, 0x83U,
        0xDEU, 0xD9U, 0xD0U, 0xD7U, 0xC2U, 0xC5U, 0xCCU, 0xCBU, 0xE6U, 0xE1U, 0xE8U, 0xEFU, 0xFAU, 0xFDU, 0xF4U, 0xF3U
};
// clang-format on
#endif // DWT_ENABLE_CRC

/*! Use statically allocated struct: to make driver compatible with legacy implementations: 1 chip <-> 1 driver */
static struct dwchip_s static_dw = { 0 };

static struct dwchip_s *dw; //! <pointer to the local driver structure

#ifdef WIN32
extern const struct dwt_driver_s dw3000_driver;
extern const struct dwt_driver_s dw3720_driver;
static const struct dwt_driver_s* tmp_ptr[] = { &dw3000_driver, &dw3720_driver };
#endif // WIN32

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function selects the correct UWB device driver from the list
 *
 * @parameter[in] probe_interf: A pointer to a structure @ref dwt_probe_s
 *
 * @return ret, @ref DWT_ERROR if no driver found or invalid probe_interface. @ref DWT_SUCCESS if driver is found.
 */
int32_t dwt_probe(struct dwt_probe_s *probe_interf)
{
    dwt_error_e ret = DWT_ERROR;
    uint32_t devId;
    uint8_t buf[sizeof(uint32_t)];
    uint8_t addr;

    if(probe_interf != NULL)
    {
        if(probe_interf->dw == NULL)
        {
            dw = &static_dw;
        }
        else
        {
            dw = (struct dwchip_s*)probe_interf->dw;
        }
        dw->SPI = (struct dwt_spi_s*)probe_interf->spi;
        dw->wakeup_device_with_io = probe_interf->wakeup_device_with_io;

        dw->wakeup_device_with_io();

        // Device ID address is common in between all DW chips
        addr = (uint8_t)DW3XXX_DEVICE_ID;

        (void)dw->SPI->readfromspi(sizeof(uint8_t), &addr, sizeof(buf), buf);
        devId = (((uint32_t)buf[3] << 24UL) | ((uint32_t)buf[2] << 16UL) | ((uint32_t)buf[1] << 8UL) | (uint32_t)buf[0]);

#ifdef WIN32
        // Find which DW device the host is connected to and assign correct low-level driver structure
        for(uint8_t i = 0U; i < 2U; i++)
        {
            if ((devId & tmp_ptr[i]->devmatch) == (tmp_ptr[i]->devid & tmp_ptr[i]->devmatch))
            {
                dw->dwt_driver = tmp_ptr[i];
                ret = DWT_SUCCESS;
                break;
            }
        }

#else
        struct dwt_driver_s **driver_list = probe_interf->driver_list;
        for (uint8_t i = 0U; i < probe_interf->dw_driver_num; i++)
        {
            if ((devId & driver_list[i]->devmatch) == (driver_list[i]->devid & driver_list[i]->devmatch))
            {
                dw->dwt_driver = driver_list[i];
                ret = DWT_SUCCESS;
                break;
            }
        }
#endif
    }

    return (int32_t)ret;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will update dw pointer to the @ref dwchip_s structure used by the driver APIs.
 *        There is a static struct dwchip_s static_dw = { 0 }; in the deca_compat.c file and a pointer to it, *dw.
 *
 * @param[in] new_dw: pointer to dw @ref dwchip_s structure instatiated by MCPS layer
 *
 * @return pointer to the old device structure dwchip_s, which can be restored when deleting MCPS instance
 *
 */
struct dwchip_s *dwt_update_dw(struct dwchip_s *new_dw)
{
    struct dwchip_s *old_dw = dw;
    dw = new_dw;
    return old_dw;
}

/********************************************************************************************************************/
/*                                  API wrapper functions                                                           */
/********************************************************************************************************************/

int32_t dwt_apiversion(void)
{
    return (int32_t)dw->dwt_driver->vernum;
}

const char *dwt_version_string(void)
{
    return DRIVER_VERSION_STR;
}

uint8_t dwt_geticrefvolt(void)
{
    uint8_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_GETICREFVOLT, 0, (void *)&tmp);
    return tmp;
}

uint8_t dwt_geticreftemp(void)
{
    uint8_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_GETICREFTEMP, 0, (void *)&tmp);
    return tmp;
}

uint32_t dwt_getpartid(void)
{
    uint32_t tmp = 0UL;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_GETPARTID, 0, (void *)&tmp);
    return tmp;
}

uint64_t dwt_getlotid(void)
{
    uint64_t tmp = 0UL;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_GETLOTID, 0, (void *)&tmp);
    return tmp;
}

uint32_t dwt_readdevid(void)
{
    uint32_t tmp = 0UL;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READ_REG, DW3XXX_DEVICE_ID, (void *)&tmp);
    return tmp;
}

uint8_t dwt_otprevision(void)
{
    uint8_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_OTPREVISION, 0, (void *)&tmp);
    return tmp;
}

void dwt_setpllcaltemperature(int8_t temperature)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETPLLCALTEMP, temperature, NULL);
}

int8_t dwt_getpllcaltemperature(void)
{
    int8_t tmp = 0;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_GETPLLCALTEMP, 0, (void *)&tmp);
    return tmp;
}

void dwt_setfinegraintxseq(int32_t enable)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETFINEGRAINTXSEQ, enable, NULL);
}

void dwt_setlnapamode(int32_t lna_pa)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETLNAPAMODE, lna_pa, NULL);
}

void dwt_setgpiomode(uint32_t gpio_mask, uint32_t gpio_modes)
{
    struct dwt_set_gpio_mode_s tmp = { gpio_mask, gpio_modes };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETGPIOMODE, 0, (void *)&tmp);
}

void dwt_setgpiodir(uint16_t in_out)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETGPIODIR, 0, (void *)&in_out);
}

void dwt_getgpiodir(uint16_t* in_out)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_GETGPIODIR, 0, (void *)in_out);
}

void dwt_setgpiovalue(uint16_t gpio, int32_t value)
{
    struct dwt_set_gpio_value_s tmp = { gpio, value };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETGPIOVALUE, 0, (void *)&tmp);
}

uint16_t dwt_readgpiovalue(void)
{
    uint16_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READGPIOVALUE, 0, (void *)&tmp);
    return tmp;
}

int32_t dwt_initialise(int32_t mode)
{
    return dw->dwt_driver->dwt_ops->initialize(dw, mode);
}

int dwt_setdwstate(int state)
{
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETDWSTATE, state, NULL);
}

void dwt_enablegpioclocks(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_ENABLEGPIOCLOCKS, 0, NULL);
}

int32_t dwt_restoreconfig(dwt_restore_type_e restore_mask)
{
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_RESTORECONFIG, (int32_t)restore_mask, NULL);
}

void dwt_restore_common(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_RESTORECOMMON, 0, NULL);
}

int32_t dwt_restore_txrx(uint8_t restore_mask)
{
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_RESTORETXRX, (uint8_t)restore_mask, NULL);
}

void dwt_configurestsmode(uint8_t stsMode)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGURESTSMODE, 0, (void *)&stsMode);
}

int32_t dwt_configure(dwt_config_t *config)
{
    return dw->dwt_driver->dwt_ops->configure(dw, config);
}

void dwt_settxpower(uint32_t power)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SET_TXPOWER, 0, (void *)&power);
}

void dwt_configuretxrf(dwt_txconfig_t *config)
{
    dw->dwt_driver->dwt_ops->configure_tx_rf(dw, config);
}

void dwt_configurestsloadiv(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGURESTSLOADIV, 0, NULL);
}

void dwt_configmrxlut(int32_t channel)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGMRXLUT, channel, NULL);
}

void dwt_configurestskey(dwt_sts_cp_key_t *pStsKey)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGURESTSKEY, 0, pStsKey);
}

void dwt_configurestsiv(dwt_sts_cp_iv_t *pStsIv)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGURESTSIV, 0, pStsIv);
}

void dwt_setrxantennadelay(uint16_t rxAntennaDelay)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETRXANTENNADELAY, 0, (void *)&rxAntennaDelay);
}

uint16_t dwt_getrxantennadelay(void)
{
    uint16_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_GETRXANTENNADELAY, 0, (void*)&tmp);
    return tmp;
}

void dwt_settxantennadelay(uint16_t txAntennaDelay)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETTXANTENNADELAY, 0, (void *)&txAntennaDelay);
}

uint16_t dwt_gettxantennadelay(void)
{
    uint16_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_GETTXANTENNADELAY, 0, (void*)&tmp);
    return tmp;
}

int32_t dwt_writetxdata(uint16_t txDataLength, uint8_t *txDataBytes, uint16_t txBufferOffset)
{
    return dw->dwt_driver->dwt_ops->write_tx_data(dw, txDataLength, txDataBytes, txBufferOffset);
}

void dwt_writetxfctrl(uint16_t txFrameLength, uint16_t txBufferOffset, uint8_t ranging)
{
    dw->dwt_driver->dwt_ops->write_tx_fctrl(dw, txFrameLength, txBufferOffset, ranging);
}

int32_t dwt_setplenfine(uint16_t preambleLength)
{
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETPLENFINE, 0, (void *)&preambleLength);
}

int dwt_setpllrxprebufen(dwt_pll_prebuf_cfg_e pll_rx_prebuf_cfg)
{
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETPLLRXPREBUFEN, 0, (void *)&pll_rx_prebuf_cfg);
}

int32_t dwt_starttx(uint8_t mode)
{
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_STARTTX, 0, (void *)&mode);
}

void dwt_setreferencetrxtime(uint32_t reftime)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETREFERENCETRXTIME, 0, (void *)&reftime);
}

void dwt_setdelayedtrxtime(uint32_t starttime)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETDELAYEDTRXTIME, 0, (void *)&starttime);
}

uint8_t dwt_get_dgcdecision(void)
{
    uint8_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_GETDGCDECISION, 0, (void *)&tmp);
    return tmp;
}

void dwt_readtxtimestamp(uint8_t *timestamp)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READTXTIMESTAMP, 0, (void *)timestamp);
}

uint32_t dwt_readtxtimestamphi32(void)
{
    uint32_t tmp = 0UL;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READTXTIMESTAMPHI32, 0, (void *)&tmp);
    return tmp;
}

uint32_t dwt_readtxtimestamplo32(void)
{
    uint32_t tmp = 0UL;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READTXTIMESTAMPLO32, 0, (void *)&tmp);
    return tmp;
}

int16_t dwt_readpdoa(void)
{
    int16_t tmp = 0;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READPDOA, 0, (void *)&tmp);
    return tmp;
}

void dwt_readtdoa(uint8_t *tdoa)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READTDOA, 0, (void *)tdoa);
}

void dwt_read_tdoa_pdoa(dwt_pdoa_tdoa_res_t *result, int32_t index)
{
    (void) index;
    uint8_t rd_tdoa[6];
    uint16_t tdoa_shifted;
    dwt_readtdoa(rd_tdoa);
    tdoa_shifted = ((uint16_t)rd_tdoa[0] + ((uint16_t)rd_tdoa[1] << 8U));
    result->tdoa = (int16_t)tdoa_shifted;
    result->pdoa = dwt_readpdoa();
}

void dwt_readrxtimestamp(uint8_t *timestamp, dwt_ip_sts_segment_e segment)
{
    (void) segment; // not used
    dw->dwt_driver->dwt_ops->read_rx_timestamp(dw, timestamp);
}

void dwt_readrxtimestampunadj(uint8_t *timestamp)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READRXTIMESTAMPUNADJ, 0, (void *)timestamp);
}

void dwt_readrxtimestamp_ipatov(uint8_t *timestamp)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READRXTIMESTAMP_IPATOV, 0, (void *)timestamp);
}

void dwt_readrxtimestamp_sts(uint8_t *timestamp)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READRXTIMESTAMP_STS, 0, (void *)timestamp);
}

uint32_t dwt_readrxtimestamphi32(void)
{
    uint32_t tmp = 0UL;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READRXTIMESTAMPHI32, 0, (void *)&tmp);
    return tmp;
}

uint32_t dwt_readrxtimestamplo32(dwt_ip_sts_segment_e segment)
{
    uint32_t tmp = 0UL;
    (void) segment; //not used
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READRXTIMESTAMPLO32, 0, (void *)&tmp);
    return tmp;
}

uint32_t dwt_readsystimestamphi32(void)
{
    uint32_t tmp = 0UL;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READSYSTIMESTAMPHI32, 0, (void *)&tmp);
    return tmp;
}

void dwt_readsystime(uint8_t *timestamp)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READSYSTIME, 0, (void *)timestamp);
}

void dwt_forcetrxoff(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_FORCETRXOFF, 0, NULL);
}

int32_t dwt_rxenable(int32_t mode)
{
    return dw->dwt_driver->dwt_ops->rx_enable(dw, mode);
}

void dwt_setsniffmode(int32_t enable, uint8_t timeOn, uint8_t timeOff)
{
    struct dwt_set_sniff_mode_s tmp = { enable, timeOn, timeOff };

    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETSNIFFMODE, 0, (void *)&tmp);
}

void dwt_setdblrxbuffmode(dwt_dbl_buff_state_e dbl_buff_state, dwt_dbl_buff_mode_e dbl_buff_mode)
{
    struct dwt_set_dbl_rx_buff_mode_s tmp = { dbl_buff_state, dbl_buff_mode };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETDBLRXBUFFMODE, 0, (void *)&tmp);
}

void dwt_signal_rx_buff_free(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SIGNALRXBUFFFREE, 0, NULL);
}

void dwt_setrxtimeout(uint32_t rx_time)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETRXTIMEOUT, 0, (void *)&rx_time);
}

void dwt_setpreambledetecttimeout(uint16_t timeout)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETPREAMBLEDETECTTIMEOUT, 0, (void *)&timeout);
}

uint16_t dwt_calibratesleepcnt(void)
{
    uint16_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CALIBRATESLEEPCNT, 0, (void *)&tmp);
    return tmp;
}

void dwt_configuresleepcnt(uint16_t sleepcnt)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGURESLEEPCNT, 0, (void *)&sleepcnt);
}

void dwt_configuresleep(uint16_t mode, uint8_t wake)
{
    struct dwt_configure_sleep_s tmp = { mode, wake };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGURESLEEP, 0, (void *)&tmp);
}

void dwt_clearaonconfig(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CLEARAONCONFIG, 0, NULL);
}

void dwt_entersleep(int32_t idle_rc)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_ENTERSLEEP, idle_rc, NULL);
}

void dwt_entersleepaftertx(int32_t enable)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_ENTERSLEEPAFTERTX, enable, NULL);
}

void dwt_entersleepafter(int32_t event_mask)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_ENTERSLEEPAFTER, event_mask, NULL);
}

void dwt_setcallbacks(dwt_callbacks_s *callbacks)
{
    dw->callbacks = *callbacks;
}

uint8_t dwt_checkirq(void)
{
    uint8_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CHECKIRQ, 0, (void *)&tmp);
    return tmp;
}

uint8_t dwt_checkidlerc(void)
{
    uint8_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CHECKIDLERC, 0, (void *)&tmp);
    return tmp;
}

void dwt_isr(void)
{
    /* It is possible that an interrupt is triggered before by SPI_RDY when 
        dwt_probe is not called yet and dw will be NULL. */
    if(dw == NULL)
    {
        return;
    }
    
    dw->dwt_driver->dwt_ops->isr(dw);
}

void dwt_setinterrupt(uint32_t bitmask_lo, uint32_t bitmask_hi, dwt_INT_options_e INT_options)
{
    dw->dwt_driver->dwt_ops->set_interrupt(dw, bitmask_lo, bitmask_hi, INT_options);
}

void dwt_setpanid(uint16_t panID)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETPANID, 0, (void *)&panID);
}

void dwt_setaddress16(uint16_t shortAddress)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETADDRESS16, 0, (void *)&shortAddress);
}

void dwt_seteui(uint8_t *eui64)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETEUI, 0, (void *)eui64);
}

void dwt_geteui(uint8_t *eui64)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_GETEUI, 0, (void *)eui64);
}

uint8_t dwt_aon_read(uint16_t aon_address)
{
    struct dwt_aon_read_s tmp = { 0U, aon_address };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_AONREAD, 0, (void *)&tmp);
    return tmp.ret_val;
}

void dwt_aon_write(uint16_t aon_address, uint8_t aon_write_data)
{
    struct dwt_aon_write_s tmp = { aon_address, aon_write_data };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_AONWRITE, 0, (void *)&tmp);
}

void dwt_configureframefilter(uint16_t enabletype, uint16_t filtermode)
{
    struct dwt_configure_ff_s tmp = { enabletype, filtermode };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGUREFRAMEFILTER, 0, (void *)&tmp);
}

uint8_t dwt_generatecrc8(const uint8_t *byteArray, uint32_t flen, uint8_t crcInit)
{
#ifdef DWT_ENABLE_CRC
    uint8_t data;

    /* Divide the message by the polynomial, a byte at a time. */
    for (uint32_t byte = 0UL; byte < flen; ++byte)
    {
        data = byteArray[byte] ^ crcInit;
        crcInit = crcTable[data]; // ^ (crcRemainderInit << 8);
    }
#endif
    /* The final remainder is the CRC. */
    return (crcInit);
}

void dwt_enablespicrccheck(dwt_spi_crc_mode_e crc_mode, dwt_spierrcb_t spireaderr_cb)
{
    struct dwt_enable_spi_crc_check_s tmp = { crc_mode, spireaderr_cb };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_ENABLESPICRCCHECK, 0, (void *)&tmp);
}

void dwt_enableautoack(uint8_t responseDelayTime, int32_t enable)
{
    struct dwt_enable_auto_ack_s tmp = { responseDelayTime, enable };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_ENABLEAUTOACK, 0, (void *)&tmp);
}

void dwt_setrxaftertxdelay(uint32_t rxDelayTime)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETRXAFTERTXDELAY, 0, (void *)&rxDelayTime);
}

void dwt_softreset(int32_t reset_semaphore)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SOFTRESET, 0, (void*)&reset_semaphore);
}

void dwt_readrxdata(uint8_t *buffer, uint16_t length, uint16_t rxBufferOffset)
{
    dw->dwt_driver->dwt_ops->read_rx_data(dw, buffer, length, rxBufferOffset);
}

void dwt_write_scratch_data(uint8_t *buffer, uint16_t length, uint16_t bufferOffset)
{
    //!!Check later if needs range protection.
    struct dwt_rw_data_s rd = { buffer, length, bufferOffset };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_WRITESCRATCHDATA, 0, &rd);
}

void dwt_read_scratch_data(uint8_t *buffer, uint16_t length, uint16_t bufferOffset)
{
    //!!Check later if needs range protection.
    struct dwt_rw_data_s rd = { buffer, length, bufferOffset };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READSCRATCHDATA, 0, &rd);
}

void dwt_readaccdata(uint8_t *buffer, uint16_t len, uint16_t accOffset)
{
    dw->dwt_driver->dwt_ops->read_acc_data(dw, buffer, len, accOffset);
}

int dwt_readcir(uint32_t *buffer, dwt_acc_idx_e cir_idx, uint16_t sample_offs,
                    uint16_t num_samples, dwt_cir_read_mode_e mode)
{
    return dw->dwt_driver->dwt_ops->read_cir( dw , buffer, cir_idx, sample_offs , num_samples , mode );
}

int dwt_readcir_48b(uint8_t *buffer, dwt_acc_idx_e acc_idx, uint16_t sample_offs, uint16_t num_samples){
    // In the QM33 devices the DWT_CIR_READ_FULL is already 48-bit. This function is added only for compatibility with QM35 devices
    return dw->dwt_driver->dwt_ops->read_cir( dw , (uint32_t*)(void*)buffer, acc_idx, sample_offs , num_samples , DWT_CIR_READ_FULL );
}

int16_t dwt_readclockoffset(void)
{
    int16_t tmp = 0;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READCLOCKOFFSET, 0, (void *)&tmp);
    return tmp;
}

int32_t dwt_readcarrierintegrator(void)
{
    int32_t tmp = 0;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READCARRIERINTEGRATOR, 0, (void *)&tmp);
    return tmp;
}

void dwt_configciadiag(uint8_t enable_mask)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGCIADIAG, 0, (void *)&enable_mask);
}

int32_t dwt_readstsquality(int16_t *rxStsQualityIndex, int32_t stsSegment)
{
    (void) stsSegment; // not used
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READSTSQUALITY, 0, (void *)rxStsQualityIndex);
}

int32_t dwt_readstsstatus(uint16_t *stsStatus, int32_t sts_num)
{
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READSTSSTATUS, sts_num, (void *)stsStatus);
}

void dwt_readdiagnostics(dwt_rxdiag_t *diagnostics)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READDIAGNOSTICS, 0, (void *)diagnostics);
}

void dwt_configeventcounters(int32_t enable)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGEVENTCOUNTERS, enable, NULL);
}

void dwt_readeventcounters(dwt_deviceentcnts_t *counters)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READEVENTCOUNTERS, 0, (void *)counters);
}

void dwt_otpread(uint16_t address, uint32_t* array, uint8_t length)
{
    struct dwt_otp_read_s rd = { address, array, length };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_OTPREAD, 0, &rd);
}

int32_t dwt_otpwriteandverify(uint32_t value, uint16_t address)
{
    struct dwt_opt_write_and_verify_s tmp = { value, address };
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_OTPWRITEANDVERIFY, 0, (void *)&tmp);
}

int32_t dwt_otpwrite(uint32_t value, uint16_t address)
{
    struct dwt_opt_write_and_verify_s tmp = { value, address };
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_OTPWRITE, 0, (void *)&tmp);
}

void dwt_setleds(uint8_t mode)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETLEDS, 0, (void *)&mode);
}

void dwt_setxtaltrim(uint8_t value)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETXTALTRIM, 0, (void *)&value);
}

uint8_t dwt_getxtaltrim(void)
{
    uint8_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_GETXTALTRIM, 0, (void *)&tmp);
    return tmp;
}

void dwt_stop_repeated_frames(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_STOPREPEATEDFRAMES, 0, NULL);
}

void dwt_repeated_frames(uint32_t framerepetitionrate)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_REPEATEDFRAMES, 0, (void *)&framerepetitionrate);
}

void dwt_send_test_preamble(uint16_t delay, uint32_t test_txpower)
{
    struct dwt_repeated_p_s tmp = { delay, test_txpower };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_REPEATEDPREAMBLE, 0, (void *)&tmp);
}

void dwt_repeated_cw(int32_t cw_enable, int32_t cw_mode_config)
{
    struct dwt_repeated_cw_s tmp = { cw_enable, cw_mode_config };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_REPEATEDCW, 0, (void *)&tmp);
}

void dwt_configcwmode(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGCWMODE, 0, NULL);
}

void dwt_configcontinuousframemode(uint32_t framerepetitionrate)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGCONTINUOUSFRAMEMODE, 0, (void *)&framerepetitionrate);
}

void dwt_disablecontinuousframemode(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_DISABLECONTINUOUSFRAMEMODE, 0, NULL);
}

void dwt_disablecontinuouswavemode(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_DISABLECONTINUOUSWAVEMODE, 0, NULL);
}

uint16_t dwt_readtempvbat(void)
{
    uint16_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READTEMPVBAT, 0, (void *)&tmp);
    return tmp;
}

float dwt_convertrawtemperature(uint8_t raw_temp)
{
    struct dwt_convert_raw_temp_s tmp = { 0.0f, raw_temp };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONVERTRAWTEMP, 0, (void *)&tmp);
    return tmp.result;
}

float dwt_convertrawvoltage(uint8_t raw_voltage)
{
    struct dwt_convert_raw_volt_s tmp = { 0.0f, raw_voltage };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONVERTRAWVBAT, 0, (void *)&tmp);
    return tmp.result;
}

uint8_t dwt_readwakeuptemp(void)
{
    uint8_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READWAKEUPTEMP, 0, (void *)&tmp);
    return tmp;
}

uint8_t dwt_readwakeupvbat(void)
{
    uint8_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READWAKEUPVBAT, 0, (void *)&tmp);
    return tmp;
}

uint8_t dwt_readpgdelay(void)
{
    uint8_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READPGDELAY, 0, (void*)&tmp);
    return tmp;
}

uint8_t dwt_calcbandwidthadj(uint16_t target_count)
{
    struct dwt_calc_bandwidth_adj_s tmp = { 0, target_count };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CALCBANDWIDTHADJ, 0, (void *)&tmp);
    return tmp.result;
}

uint16_t dwt_calcpgcount(uint8_t pgdly)
{
    struct dwt_calc_pg_count_s tmp = { 0U, pgdly };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CALCPGCOUNT, 0, (void *)&tmp);
    return tmp.result;
}

/********************************************************************************************************************/
/*                                                AES BLOCK                                                         */
/********************************************************************************************************************/

void dwt_set_keyreg_128(dwt_aes_key_t *key)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETKEYREG128, 0, (void *)key);
}

void dwt_configure_aes(dwt_aes_config_t *pCfg)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGUREAES, 0, (void *)pCfg);
}

dwt_mic_size_e dwt_mic_size_from_bytes(uint8_t mic_size_in_bytes)
{
    struct dwt_mic_size_from_bytes_s tmp = { MIC_0, mic_size_in_bytes };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_MICSIZEFROMBYTES, 0, (void *)&tmp);
    return tmp.result;
}

int8_t dwt_do_aes(dwt_aes_job_t *job, dwt_aes_core_type_e core_type)
{
    struct dwt_do_aes_s tmp = { 0, job, core_type };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_DOAES, 0, (void *)&tmp);
    return tmp.result;
}

int32_t dwt_check_dev_id(void)
{
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CHECKDEVID, 0, NULL);
}

int32_t dwt_run_pgfcal(void)
{
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_RUNPGFCAL, 0, NULL);
}

int32_t dwt_pgf_cal(int32_t ldoen)
{
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_PGF_CAL, ldoen, NULL);
}

uint32_t dwt_readpllstatus(void)
{
    return (uint32_t)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_PLL_STATUS, 0, NULL);
}

int32_t dwt_pll_cal(void)
{
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_PLL_CAL, 0, NULL);
}

void dwt_configure_rf_port(dwt_rf_port_ctrl_e port_control)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGURE_RF_PORT, (int)port_control, NULL);
}

void dwt_configure_le_address(uint16_t addr, int32_t leIndex)
{
    struct dwt_configure_le_address_s tmp = { addr, leIndex };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGURELEADDRESS, addr, (void *)&tmp);
}

void dwt_configuresfdtype(dwt_sfd_type_e sfdType)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGURESFDTYPE, 0, (void *)&sfdType);
}

void dwt_settxcode(uint8_t tx_code)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETTXCODE, 0, (void *)&tx_code);
}

void dwt_setrxcode(uint8_t rx_code)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETRXCODE, 0, (void *)&rx_code);
}

uint32_t dwt_read_reg(uint32_t address)
{
    uint32_t tmp32 = 0UL;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READ_REG, (int)address, (void *)&tmp32);
    return tmp32;
}

void dwt_write_reg(uint32_t address, uint32_t data)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_WRITE_REG, address, (void *)(uint32_t *)data);
}

void dwt_writesysstatuslo(uint32_t mask)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_WRITESYSSTATUSLO, 0, (void *)&mask);
}

void dwt_writesysstatushi(uint32_t mask)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_WRITESYSSTATUSHI, 0, (void *)&mask);
}

uint32_t dwt_readsysstatuslo(void)
{
    uint32_t tmp = 0UL;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READSYSSTATUSLO, 0, (void *)&tmp);
    return tmp;
}

uint32_t dwt_readsysstatushi(void)
{
    uint32_t tmp = 0UL;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READSYSSTATUSHI, 0, (void *)&tmp);
    return tmp;
}

void dwt_writerdbstatus(uint8_t mask)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_WRITERDBSTATUS, 0, (void *)&mask);
}

uint8_t dwt_readrdbstatus(void)
{
    uint8_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READRDBSTATUS, 0, (void *)&tmp);
    return tmp;
}

uint16_t dwt_getframelength(uint8_t *rng)
{
    struct dwt_getframelength_s tmp = { 0, 0 };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_GETFRAMELENGTH, 0, (void *)&tmp);
    *rng = tmp.rng_bit;
    return tmp.frame_len;
}

uint32_t dwt_readpdoaoffset(void)
{
    uint32_t tmp = 0UL;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READPDOAOFFSET, 0, (void *)&tmp);
    return tmp;
}

void dwt_setpdoaoffset(uint16_t offset)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETPDOAOFFSET, 0, (void *)&offset);
}

void dwt_setinterrupt_db(uint8_t bitmask, dwt_INT_options_e INT_options)
{
    struct dwt_set_interrupt_db_s tmp = { bitmask, INT_options };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETINTERUPTDB, 0, (void *)&tmp);
}

void dwt_ds_sema_request(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_DSSEMAREQUEST, 0, NULL);
}

void dwt_ds_sema_release(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_DSSEMARELEASE, 0, NULL);
}

void dwt_ds_sema_force(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_DSSEMAFORCE, 0, NULL);
}

uint8_t dwt_ds_sema_status(void)
{
    uint8_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_DSSEMASTATUS, 0, (void *)&tmp);
    return tmp;
}

uint8_t dwt_ds_sema_status_hi(void)
{
    uint8_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_DSSEMASTATUS, 1, (void*)&tmp);
    return tmp;
}

void dwt_ds_en_sleep(dwt_host_sleep_en_e host_sleep_en)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_DSENSLEEP, 0, (void *)&host_sleep_en);
}

int32_t dwt_ds_setinterrupt_SPIxavailable(dwt_spi_host_e spi_num, dwt_INT_options_e int_set)
{
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_DSSETINT_SPIAVAIL, (int) spi_num, (void*)&int_set);
}

void dwt_enable_disable_eq(uint8_t en)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_ENABLEDISABLEEQ, 0, (void *)&en);
}

void dwt_timers_reset(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_TIMERSRST, 0, NULL);
}

uint16_t dwt_timers_read_and_clear_events(void)
{
    uint16_t tmp = 0U;
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_TIMERSRSTCLR, 0, (void *)&tmp);
    return tmp;
}

void dwt_configure_timer(dwt_timer_cfg_t *tim_cfg)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONFIGTIMER, 0, (void *)tim_cfg);
}

void dwt_configure_wificoex_gpio(uint8_t timer_coexout, uint8_t coex_swap)
{
    struct dwt_cfg_wifi_coex_s tmp = { timer_coexout, coex_swap };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CFGWIFICOEXGPIO, 0, (void *)&tmp);
}

void dwt_configure_and_set_antenna_selection_gpio(uint8_t antenna_config)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CFGANTSEL, 0, (void *)&antenna_config);
}

void dwt_set_timer_expiration(dwt_timers_e timer_name, uint32_t expiration)
{
    struct dwt_timer_exp_s tmp = { timer_name, expiration };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_TIMEREXPIRATION, 0, (void *)&tmp);
}

void dwt_timer_enable(dwt_timers_e timer_name)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_TIMERENABLE, 0, (void *)&timer_name);
}

void dwt_wifi_coex_set(dwt_wifi_coex_e enable, int32_t coex_io_swap)
{
    struct dwt_cfg_wifi_coex_set_s tmp = { enable, coex_io_swap };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CFGWIFICOEXSET, 0, (void *)&tmp);
}

void dwt_reset_system_counter(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_RSTSYSTEMCNT, 0, NULL);
}

void dwt_config_ostr_mode(uint8_t enable, uint16_t wait_time)
{
    struct dwt_ostr_mode_s tmp = { enable, wait_time };
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CFGOSTRMODE, 0, (void *)&tmp);
}

void dwt_set_fixedsts(uint8_t set)
{
	(void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETFIXEDSTS, 0, (void *)&set);
}

uint32_t dwt_readctrdbg(void)
{
	uint32_t tmp = 0UL;
	(void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READCTRDBG, 0, (void*)&tmp);
	return tmp;
}

uint32_t dwt_readdgcdbg(void)
{
	uint32_t tmp = 0UL;
	(void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READDGCDBG, 0, (void*)&tmp);
	return tmp;
}

uint32_t dwt_readCIAversion(void)
{
	uint32_t tmp = 0UL;
	(void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CIA_VERSION, 0, (void*)&tmp);
	return tmp;
}

uint32_t dwt_getcirregaddress(void)
{
	uint32_t tmp = 0UL;
	(void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_GET_CIR_REGADD, 0, (void*)&tmp);
	return tmp;
}

register_name_add_t* dwt_get_reg_names(void)
{
    return (register_name_add_t*) dw->dwt_driver->dwt_ops->dbg_fn(dw, DWT_DBG_REGS, 0, NULL);
}

void dwt_set_alternative_pulse_shape(uint8_t set_alternate)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SET_ALT_PULSE_SHAPE, 0, (void *)&set_alternate);
}

int dwt_nlos_alldiag(dwt_nlos_alldiag_t *all_diag)
{
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_NLOS_ALLDIAG, 0, (void *)all_diag);
}

void dwt_nlos_ipdiag(dwt_nlos_ipdiag_t *index)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_NLOS_IPDIAG, 0, (void *)index);
}

int32_t dwt_adjust_tx_power(uint16_t boost, uint32_t ref_tx_power, uint8_t channel, uint32_t* adj_tx_power, uint16_t* applied_boost)
{
    struct dwt_adj_tx_power_s tmp = { 0, boost, ref_tx_power, channel, adj_tx_power, applied_boost};
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_ADJ_TXPOWER, 0, (void *)&tmp);
    *adj_tx_power = *tmp.adj_tx_power;
    *applied_boost = *tmp.applied_boost;
    return tmp.result;
}

int32_t dwt_calculate_linear_tx_power(uint32_t channel, power_indexes_t *p_indexes, tx_adj_res_t *p_res)
{
    struct dwt_calculate_linear_tx_power_s tmp = {0, channel, p_indexes, p_res};
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_LINEAR_TXPOWER, 0, (void *)&tmp);
    *p_indexes = *tmp.txp_indexes;
    *p_res = *tmp.txp_res;
    return tmp.result;
}

int dwt_convert_tx_power_to_index(uint32_t channel, uint8_t tx_power, uint8_t *tx_power_idx)
{
	struct dwt_convert_tx_power_to_index_s tmp = {0, channel, tx_power, tx_power_idx};
	int ret = dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CONVERT_TXPOWER_TO_IDX, 0, (void *)&tmp);
	if (ret != 0)
    {
		return (int)DWT_ERROR;
    }
	return tmp.result;
}

void dwt_setpllbiastrim(uint8_t pll_bias_trim)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SET_PLLBIASTRIM, 0, (void *)&pll_bias_trim);
}

int32_t dwt_setchannel(dwt_pll_ch_type_e ch)
{
    return dw->dwt_driver->dwt_mcps_ops->set_channel(dw, ch);
}

void dwt_setstslength(uint8_t stsblocks)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SET_STS_LEN, 0, (void *)&stsblocks);
}

int32_t dwt_setphr(dwt_phr_mode_e phrMode, dwt_phr_rate_e phrRate)
{
    struct dwt_set_phr_s tmp = {phrMode, phrRate};
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SET_PHR, 0, (void *)&tmp);

    return (int32_t)DWT_SUCCESS;
}

int32_t dwt_setdatarate(dwt_uwb_bit_rate_e bitRate)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SET_DATARATE, 0, (void *)&bitRate);

    return (int32_t)DWT_SUCCESS;
}

int32_t dwt_setrxpac(dwt_pac_size_e rxPAC)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SET_PAC, 0, (void *)&rxPAC);

    return (int32_t)DWT_SUCCESS;
}

int32_t dwt_setsfdtimeout(uint16_t sfdTO)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SET_SFDTO, 0, (void *)&sfdTO);

    return (int32_t)DWT_SUCCESS;
}

void dwt_disable_OTP_IPS(int32_t mode)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_DIS_OTP_IPS, mode, NULL);
}

uint8_t dwt_pll_chx_auto_cal(int32_t chan, uint32_t coarse_code, uint16_t sleep, uint8_t steps, int8_t temperature)
{
    struct dwt_set_pll_cal_s tmp = {coarse_code, sleep, steps, temperature};
    return ((uint8_t) dw->dwt_driver->dwt_ops->ioctl(dw, DWT_PLL_AUTO_CAL, chan, (void *)&tmp));
}

int32_t dwt_xtal_temperature_compensation(dwt_xtal_trim_t *params, uint8_t *xtaltrim)
{
	struct dwt_set_xtal_cal_s tmp = {params, xtaltrim};
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_XTAL_AUTO_TRIM, 0,  (void *)&tmp);
}

void dwt_capture_adc_samples(dwt_capture_adc_t *capture_adc)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CAPTURE_ADC, 0, (void *)capture_adc);
}

void dwt_read_adc_samples(dwt_capture_adc_t *capture_adc)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READ_ADC_SAMPLES, 0, (void *)capture_adc);
}

void dwt_configtxrxfcs(uint8_t enable)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SET_FCS_MODE, 0, (void *)&enable);
}

int dwt_calculate_rssi(const dwt_cirdiags_t *diag, dwt_acc_idx_e acc_idx, int16_t *signal_strength)
{
    struct dwt_calculate_rssi_s tmp = {diag, acc_idx, signal_strength};
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CALCULATE_RSSI, 0,  (void *)&tmp);
}

int dwt_calculate_first_path_power(const dwt_cirdiags_t *diag, dwt_acc_idx_e acc_idx, int16_t *signal_strength)
{
    struct dwt_calculate_rssi_s tmp = {diag, acc_idx, signal_strength};
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_CALCULATE_FIRST_PATH_POWER, 0,  (void *)&tmp);
}

int dwt_readdiagnostics_acc(dwt_cirdiags_t *diag, dwt_acc_idx_e acc_idx)
{
    struct dwt_readdiagnostics_acc_s tmp = {diag, acc_idx};
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_READDIAGNOSTICS_ACC, 0,  (void *)&tmp);
}

int dwt_setpdoamode(dwt_pdoa_mode_e pdoaMode)
{
    return (dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SETPDOAMODE, (int32_t)pdoaMode, NULL));
}

void dwt_configureisr(dwt_isr_flags_e flags)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SET_ISR_FLAGS, (int32_t)flags, NULL);
}

/********************************************************************************************************************/
/*             Declaration of platform-dependent lower level functions.                                             */
/********************************************************************************************************************/

void dwt_wakeup_ic(void)
{
    (void)dw->dwt_driver->dwt_ops->ioctl(dw, DWT_WAKEUP, 0, NULL);
}

#ifdef WIN32

int32_t dwt_spicswakeup(uint8_t *buff, uint16_t length)
{
    struct dwt_spi_cs_wakeup_s tmp = { buff, length };
    return dw->dwt_driver->dwt_ops->ioctl(dw, DWT_SPICSWAKEUP, 0, (void *)&tmp);
}
#endif

/********************************************************************************************************************/
/*                                           Private Functions                                                      */
/********************************************************************************************************************/

void dwt_writetodevice(uint32_t regFileID, uint16_t index, uint16_t length, uint8_t *buffer)
{
    dw->dwt_driver->dwt_mcps_ops->write_to_device(dw, regFileID, index, length, buffer);
}

void dwt_readfromdevice(uint32_t regFileID, uint16_t index, uint16_t length, uint8_t *buffer)
{
    dw->dwt_driver->dwt_mcps_ops->read_from_device(dw, regFileID, index, length, buffer);
}
