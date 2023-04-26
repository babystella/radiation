/*
 * c12137_comm.c
 *
 * RadiationDetector_Linux
 *
 *
 *
 *
 */

#include <errno.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <usb.h>

#include "c12137_comm.h"

/* Define */
#define BULK_HEADER (8)
#define BULK_DATA (1048)
#define BULK_SIZE (8 + 1048)

#define ENDPOINT_BULK (2)

#define REQ_RESET_REQUEST (0x01)
#define REQ_READ_EEPROM (0x04)
#define REQ_RADIATION_LIMIT (0x07)
#define REQ_ENERFY_THRESHOLD (0x0C)
#define REQ_I2C_RXD_REQUEST (0x22)
#define REQ_SEIVERT_UPDATE_CYCLE (0x38)
#define REQ_CLEAR_BULK_BUFFER (0xF0)

#define I2C_REQ_TEMP_A (3) // Temperature information Analog

/* Prototype function */
static double VoltToCelcius(unsigned short volt);

/**
 *  LM94021
 *  0-50deg (1034-760mV)
 *  ADC 0-1250mV/16bit
 *
 *  V - V1 = ((V2 - V1) / (T2 - T1)) * (T - T1)
 *  T = T1 + ((V - V1) * (T2 - T1)) / (V2 - V1)
 *  T = 0 + ((v * 1250 / 65535 - 1034) * (50 - 0)) / (760 - 1034)
 *    = -0.00348 * v + 188.686
 */
static double VoltToCelcius(unsigned short volt)
{
    double t = -0.00348 * (double)volt + 188.686;

    return t;
}

/**
 * @fn RdmUsb_ResetRequest
 * @brief
 * @param[in] dh Deice handle
 * @param[in] level
 * @retval 0 Success
 * @retval n Error
 */
int RdmUsb_ResetRequest(usb_dev_handle *dh,
                        unsigned short level)
{
    int res = 0;

    res = usb_control_msg(dh,
                          USB_RECIP_OTHER | USB_TYPE_VENDOR | USB_ENDPOINT_OUT,
                          REQ_RESET_REQUEST,
                          level, 0x00, NULL, 0, -1);

    usleep(100000); // 100ms wait

    if (res < 0)
    {
        return RDMUSB_UNSUCCESS;
    }

    return RDMUSB_SUCCESS;
}
/**
 * @fn RdmUsb_GetEnergyThreshold
 * @brief:w
 *
 * @param[in] dh Deice handle
 * @param[out] data
 * @retval 0 Success
 * @retval n Error
 */
int RdmUsb_GetEnergyThreshold(usb_dev_handle *dh,
                              unsigned short *data)
{
    int res = 0;
    unsigned short buf = 0;

    res = usb_control_msg(dh,
                          USB_RECIP_OTHER | USB_TYPE_VENDOR | USB_ENDPOINT_IN,
                          REQ_ENERFY_THRESHOLD,
                          0x00, 0x00, (char *)&buf, 2, -1);

    if (res > 0)
    {
        *data = (buf >> 8) + (buf << 8);
        res = 0;
    }
    else
    {
        return RDMUSB_UNSUCCESS;
    }

    return RDMUSB_SUCCESS;
}

/**
 * @fn RdmUsb_SetEnergyThreshold
 * @brief
 * @param[in] dh Deice handle
 * @param[in] data
 * @retval 0 Success
 * @retval n Error
 */
int RdmUsb_SetEnergyThreshold(usb_dev_handle *dh,
                              unsigned short data)
{
    int res = 0;

    res = usb_control_msg(dh,
                          USB_RECIP_OTHER | USB_TYPE_VENDOR | USB_ENDPOINT_OUT,
                          REQ_ENERFY_THRESHOLD,
                          data, 0x00, NULL, 0, -1);

    if (res < 0)
    {
        return RDMUSB_UNSUCCESS;
    }

    return RDMUSB_SUCCESS;
}

/**
 * @fn RdmUsb_GetRadiationLimit
 * @brief
 * @param[in] dh Deice handle
 * @param[in] area
 * @param[out] data
 * @retval 0 Success
 * @retval n Error
 */
int RdmUsb_GetRadiationLimit(usb_dev_handle *dh,
                             unsigned short area,
                             unsigned short *data)
{
    int res = 0;
    unsigned short buf = 0;

    res = usb_control_msg(dh,
                          USB_RECIP_OTHER | USB_TYPE_VENDOR | USB_ENDPOINT_IN,
                          REQ_RADIATION_LIMIT,
                          0x00, area, (char *)&buf, 2, -1);

    if (res > 0)
    {
        *data = (buf >> 8) + (buf << 8);
    }
    else
    {
        return RDMUSB_UNSUCCESS;
    }

    return RDMUSB_SUCCESS;
}

/**
 * @fn RdmUsb_SetRadiationLimit
 * @brief
 * @param[in] dh Deice handle
 * @param[in] data
 * @retval 0 Success
 * @retval n Error
 */
int RdmUsb_SetRadiationLimit(usb_dev_handle *dh,
                             unsigned short area,
                             unsigned short data)
{
    int res = 0;

    res = usb_control_msg(dh,
                          USB_RECIP_OTHER | USB_TYPE_VENDOR | USB_ENDPOINT_OUT,
                          REQ_RADIATION_LIMIT,
                          data, area, NULL, 0, -1);

    if (res < 0)
    {
        return RDMUSB_UNSUCCESS;
    }

    return RDMUSB_SUCCESS;
}

/**
 * @fn RdmUsb_GetDataAndTemperture
 * @brief
 * @param[in] dh Deice handle
 * @param[out] index
 * @param[out] size
 * @param[out] data
 * @param[out] Temperature
 * @retval 0 Success
 * @retval n Error
 */
int RdmUsb_GetDataAndTemperture(
    usb_dev_handle *dh,
    unsigned short *index,
    unsigned short *size,
    unsigned short *data,
    double *temperature)
{
    /**
     * Data format
     *    buffer[0] : fixed value (0x5A5A)
     *    buffer[1] : fixed value (0x5A5A)
     *    buffer[2] : data size (0 - 1000)
     *    buffer[3] : fixed value (0x0000)
     *    buffer[4] : index (increment counter every 100 ms)
     *    buffer[5] : temperature (adc value - digit)
     *    buffer[6] : fixed value(0x0000)
     *    buffer[7] : debug area
     *    buffer[8-1007] : data area
     *    buffer[1008-1055] : fixed value (0x0000)
     */

    const int BULK_READ_SIZE = BULK_SIZE * sizeof(unsigned short);

    int res = 0;

    int result;
    unsigned short buffer[BULK_SIZE];
    int i;
    unsigned short val;

    res = usb_bulk_read(dh, ENDPOINT_BULK, (char *)buffer,
                        BULK_READ_SIZE, 100);

    if (res != BULK_READ_SIZE)
    {
        return RDMUSB_UNSUCCESS;
    }

    // Header Byte swap (Endian)
    for (i = 0; i < BULK_HEADER; i++)
    {
        val = buffer[i];
        buffer[i] = (val << 8) + (val >> 8);
    }

    if (buffer[0] != 0x5A5A || buffer[1] != 0x5A5A)
    {
        return RDMUSB_PACKET_ERROR;
    }

    // Packet index
    *index = buffer[4];

    // Data (ADC value)
    *size = buffer[2];
    memcpy(data, &buffer[8], buffer[2] * sizeof(unsigned short));

    // Data (Temperature LM94021 GS0=0 GS1=0)
    *temperature = VoltToCelcius(buffer[5]);

    return RDMUSB_SUCCESS;
}

/**
 * @fn RdmUsb_ClearBulkBuffer
 * @brief
 * @param[in] dh Deice handle
 * @retval 0 Success
 * @retval n Error
 */
int RdmUsb_ClearBulkBuffer(usb_dev_handle *dh)
{
    int res = 0;

    res = usb_control_msg(dh,
                          USB_RECIP_OTHER | USB_TYPE_VENDOR | USB_ENDPOINT_OUT,
                          REQ_CLEAR_BULK_BUFFER,
                          0x00, 0x00, NULL, 0, -1);

    if (res < 0)
    {
        return RDMUSB_UNSUCCESS;
    }

    return RDMUSB_SUCCESS;
}

/**
 * @fn RdmUsb_ReadEeprom
 * @brief
 * @param[in] dh Deice handle
 * @param[in] address
 * @param[out] data
 * @retval 0 Success
 * @retval n Error
 */
int RdmUsb_ReadEeprom(
    usb_dev_handle *dh,
    unsigned short address,
    unsigned short *data)
{
    int res = 0;
    unsigned short buf = 0;

    res = usb_control_msg(dh,
                          USB_RECIP_OTHER | USB_TYPE_VENDOR | USB_ENDPOINT_IN,
                          REQ_READ_EEPROM,
                          address, 0x02, (char *)&buf, 2, -1);

    if (res > 0)
    {
        *data = (buf >> 8) + (buf << 8);
    }
    else
    {
        return RDMUSB_UNSUCCESS;
    }

    return RDMUSB_SUCCESS;
}

/**
 * @fn RdmUsb_ReadEeprom
 * @brief
 * @param[in] dh Deice handle
 * @param[out] celcius
 * @param[out] digit
 * @retval 0 Success
 * @retval n Error
 */
int RdmUsb_HvpsGetTemperature(
    usb_dev_handle *dh,
    double *celcius,
    unsigned short *digit)
{
    int res = 0;
    unsigned short buf = 0;

    unsigned short value = 0;
    double tval = 255;

    res = usb_control_msg(dh,
                          USB_RECIP_OTHER | USB_TYPE_VENDOR | USB_ENDPOINT_OUT,
                          REQ_I2C_RXD_REQUEST,
                          I2C_REQ_TEMP_A, 0x00, NULL, 0, -1);

    if (res < 0)
    {
        return RDMUSB_UNSUCCESS;
    }

    usleep(100000); // 100ms wait

    res = usb_control_msg(dh,
                          USB_RECIP_OTHER | USB_TYPE_VENDOR | USB_ENDPOINT_IN,
                          REQ_I2C_RXD_REQUEST,
                          0x00, 0x00, (char *)&buf, 2, -1);

    if (res > 0)
    {
        // Temperature coefficient conversion
        double t1 = 0;
        double v1 = 1034;
        double t2 = 50;
        double v2 = 760;
        double range = 1250; // 1250mV

        value = (buf >> 8) + (buf << 8);
        *digit = value;
        tval = (double)value * range / 65535;
        *celcius = t1 + ((tval - v1) * (t2 - t1)) / (v2 - v1);
    }
    else
    {
        return RDMUSB_UNSUCCESS;
    }

    return RDMUSB_SUCCESS;
}
