/*
 * c12137_comm.h
 *
 * RadiationDetector_Linux
 *
 *
 *
 *
 */

#include <usb.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>


/* Define */
#define USB_VENDOR 0x0661
#define USB_PRODUCT 0x2917

#define TIMEOUT (5 * 1000)

// EEPROM Address
#define EEPROM_COMP_LEVEL (0x0A)   // default = 30 keV
#define EEPROM_ENERGY_LOWER (0x0C) // default = 30 keV
#define EEPROM_ENERGY_UPPER (0x0E) // default = 2000 keV
#define EEPROM_CONVERT_USV (0x10)  // default = 1000

// RDMUSB Return value list
#define RDMUSB_SUCCESS (0)
#define RDMUSB_INVALID_HANDLE (1)
#define RDMUSB_UNSUCCESS (2)
#define RDMUSB_INVALID_VALUE (3)
#define RDMUSB_NOT_UPDATED (4)
#define RDMUSB_PACKET_ERROR (5)

/* Prototype function */
int RdmUsb_ResetRequest(usb_dev_handle *dh,
                        unsigned short level);

int RdmUsb_GetEnergyThreshold(usb_dev_handle *dh,
                              unsigned short *data);
int RdmUsb_SetEnergyThreshold(usb_dev_handle *dh,
                              unsigned short data);

int RdmUsb_GetRadiationLimit(usb_dev_handle *dh,
                             unsigned short area,
                             unsigned short *data);
int RdmUsb_SetRadiationLimit(usb_dev_handle *dh,
                             unsigned short area,
                             unsigned short data);

int RdmUsb_GetDataAndTemperture(
    usb_dev_handle *dh,
    unsigned short *index,
    unsigned short *size,
    unsigned short *data,
    double *temperature);

int RdmUsb_ClearBulkBuffer(usb_dev_handle *dh);

int RdmUsb_ReadEeprom(
    usb_dev_handle *dh,
    unsigned short address,
    unsigned short *data);

int RdmUsb_HvpsGetTemperature(
    usb_dev_handle *dh,
    double *celcius,
    unsigned short *digit);
