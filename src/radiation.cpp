#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include "std_msgs/UInt32.h"
#include <errno.h>
#include <vector>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <map>
#include <unistd.h>
#include <algorithm>
#include <usb.h>
#include "radiation/sievert.h"
#include "radiation/keV.h"

#include "c12137_comm.h"

/* Define */
#define APP_VERSION "1.0.0" // Application version

#define GE_FUNCTION_C12137 "gef_C12137.csv"       // G(E) Function file name C12137.csv
#define GE_FUNCTION_C12137_01 "gef_C12137-01.csv" // G(E) Function file name C12137-01.csv
#define GE_FUNCTION_FILENAME GE_FUNCTION_C12137   // Select G(E) Function file name

#define GE_FUNCTION_TABLE_SIZE (4096) // G(E) Function table size
#define DATA_BUFFER_SIZE (1000)       // Get data buffer size

#define ARGUMENTS_PARAM_NUM_MAX (8) // Argument number
#define ARGUMENTS_BUF_MAX (256)     // Argument buffer size

#define ARGUMENTS_TARTGET_VERSION "--version" // Target application version argument
#define ARGUMENTS_TARTGET_HELP "--help"       // Target help argument

#define ARGUMENTS_TARTGET_RESET "--reset"                              // Target Reset module argument
#define ARGUMENTS_TARTGET_GET_ENERGY_THRESHOLD "--g_eng_th"            // Target Get gamma-ray energy lower limit argument
#define ARGUMENTS_TARTGET_SET_ENERGY_THRESHOLD "--s_eng_th"            // Target Set gamma-ray energy lower limit argument
#define ARGUMENTS_TARTGET_GET_RADIATION_UPPER_LIMIT "--g_rd_up"        // Target Get upper limits of effective range of air dose argument
#define ARGUMENTS_TARTGET_SET_RADIATION_UPPER_LIMIT "--s_rd_up"        // Target Set upper limits of effective range of air dose argument
#define ARGUMENTS_TARTGET_GET_RADIATION_LOWER_LIMIT "--g_rd_lo"        // Target Get lower limits of effective range of air dose argument
#define ARGUMENTS_TARTGET_SET_RADIATION_LOWER_LIMIT "--s_rd_lo"        // Target Set lower limits of effective range of air dose argument
#define ARGUMENTS_TARTGET_GET_DATA_AND_TEMPERATURE "--g_data_and_temp" // Target Get Data and temperature argument
#define ARGUMENTS_TARTGET_GET_SIEVERT "--g_sievert"                    // Target Get sievert argument
#define ARGUMENTS_TARTGET_GET_INTERNAL_TEMPERATURE "--g_int_temp"      // Target Get internal temperature argument
#define ARGUMENTS_TARTGET_READ_EEPROM "--r_eeprom"                     // Target Read EEPROM argument




std::vector<int> energycounts; 



/* Prototype function */
void abort_handler(int signal);

// Command argument check
static int checkCommandArguments(int *pArgc, char *pArgv[],
                                 char *pCommandRequestStr,
                                 char commandParamStrList[][ARGUMENTS_BUF_MAX],
                                 int *commandParamCnt);

static struct usb_bus *USB_init();
static struct usb_device *USB_find(struct usb_bus *busses, struct usb_device *dev);
static struct usb_dev_handle *USB_open(struct usb_device *dev);
static void USB_close(struct usb_dev_handle *dh);
static void USB_altinterface(struct usb_dev_handle *dh, int tyep);

static int Load_GE_FunctionFile(char *path);
static unsigned short ConvkeVtoIndex(unsigned short valuen);
static unsigned short ConvIndextokeV(unsigned short value);

// Reset module
static int ProcResetModule(struct usb_dev_handle *dh);

// Get gamma-ray energy lower limit
static int ProcGetEnergyThreshold(struct usb_dev_handle *dh);

// Set gamma-ray energy lower limit
static int ProcSetEnergyThreshold(struct usb_dev_handle *dh,
                                  unsigned short threshold);

// Get upper limits of effective range of air dose
static int ProcGetRadiationUpperLimit(struct usb_dev_handle *dh);

// Set upper limits of effective range of air dose
static int ProcSetRadiationUpperLimit(struct usb_dev_handle *dh,
                                      unsigned short upper);

// Get lower limits of effective range of air dose
static int ProcGetRadiationLowerLimit(struct usb_dev_handle *dh);

// Set lower limits of effective range of air dose
static int ProcSetRadiationLowerLimit(struct usb_dev_handle *dh,
                                      unsigned short lower);

// Acquisition request of acquired data
static int ProcGetDacDataAndTemperature(struct usb_dev_handle *dh);

// Get internal temperature
static int ProcGetInternalTemperature(struct usb_dev_handle *dh);

// Get sievert
static int ProcGetSievert(struct usb_dev_handle *dh,
                          double second,
                          int times, 
                          ros::Publisher &sv_pub 
                          );

// Read EEPROM
static int ProcReadEeprom(struct usb_dev_handle *dh,
                          unsigned short address);

/* volatile variable */
volatile sig_atomic_t IsAbortReq = 0; // Abaort request

/* static variable */
static double GeArray[2][GE_FUNCTION_TABLE_SIZE] = {0}; // GE Array
static double GeHistogramIndex[GE_FUNCTION_TABLE_SIZE] = {0}; //Hist index Array
static int GeHistogramCount [GE_FUNCTION_TABLE_SIZE] ={0}; //Hist count Array

static char IsExitReq = 0; // Exit request

/* Abort handler */
void abort_handler(int signal)
{
    printf("Abort request.\n");

    IsAbortReq = 1;
}

/* Command argument check processing */
static int checkCommandArguments(int *pArgc, char *pArgv[],
                                 char *pCommandRequestStr,
                                 char commandParamStrList[][ARGUMENTS_BUF_MAX],
                                 int *commandParamCnt)
{
    const int ARG_CNT_MAX = *pArgc;

    int arg_idx;

    *commandParamCnt = 0;

    if (ARG_CNT_MAX < 2)
    {
        // Argument shortage
        return -1;
    }

    // Request
    strcpy(pCommandRequestStr, pArgv[1]);

    for (arg_idx = 2; arg_idx < ARG_CNT_MAX; arg_idx++)
    {
        // Parameter
        strcpy(&commandParamStrList[arg_idx - 2][0], pArgv[arg_idx]);
        *commandParamCnt += 1;
    }

    return 0;
}

/* Initialize USB */
static struct usb_bus *USB_init()
{
    usb_init();
    usb_find_busses();
    usb_find_devices();

    return (usb_get_busses());
}

/* Find USB device */
static struct usb_device *USB_find(struct usb_bus *busses, struct usb_device *dev)
{
    struct usb_bus *bus;

    for (bus = busses; bus; bus = bus->next)
    {
        for (dev = bus->devices; dev; dev = dev->next)
        {
            if ((dev->descriptor.idVendor == USB_VENDOR) && (dev->descriptor.idProduct == USB_PRODUCT))
            {
                return (dev);
            }
        }
    }

    return (NULL);
}

/* Open USB device */
static struct usb_dev_handle *USB_open(struct usb_device *dev)
{
    struct usb_dev_handle *udev = NULL;

    udev = usb_open(dev);
    if ((udev = usb_open(dev)) == NULL)
    {
        fprintf(stderr, "usb_open Error.(%s)\n", usb_strerror());
        exit(1);
    }

    if (usb_set_configuration(udev, dev->config->bConfigurationValue) < 0)
    {
        if (usb_detach_kernel_driver_np(udev,
                                        dev->config->interface->altsetting->bInterfaceNumber) < 0)
        {
            fprintf(stderr, "usb_set_configuration Error.\n");
            fprintf(stderr, "usb_detach_kernel_driver_np Error.(%s)\n",
                    usb_strerror());
        }
    }

    if (usb_claim_interface(udev, dev->config->interface->altsetting->bInterfaceNumber) < 0)
    {
        if (usb_detach_kernel_driver_np(udev,
                                        dev->config->interface->altsetting->bInterfaceNumber) < 0)
        {
            fprintf(stderr, "usb_claim_interface Error.\n");
            fprintf(stderr, "usb_detach_kernel_driver_np Error.(%s)\n",
                    usb_strerror());
        }
    }

    if (usb_claim_interface(udev, dev->config->interface->altsetting->bInterfaceNumber) < 0)
    {
        fprintf(stderr, "usb_claim_interface Error.(%s)\n", usb_strerror());
    }

    return (udev);
}

/* Close USB device */
static void USB_close(struct usb_dev_handle *dh)
{
    if (usb_release_interface(dh, 0))
    {
        fprintf(stderr, "usb_release_interface() failed. (%s)\n",
                usb_strerror());
    }

    if (usb_close(dh) < 0)
    {
        fprintf(stderr, "usb_close Error.(%s)\n", usb_strerror());
    }
}

/* Set alternative USB interface */
static void USB_altinterface(struct usb_dev_handle *dh, int tyep)
{
    if (usb_set_altinterface(dh, tyep) < 0)
    {
        fprintf(stderr, "Failed to set altinterface %d: %s\n", 1,
                usb_strerror());
        USB_close(dh);
    }
}

/* Load G(E) function file */
static int Load_GE_FunctionFile(char *path)
{
    // FILE *fp, *ptr_test;
    FILE *fp;
    char str[64];
    int i;
    char *ptr;

    fp = fopen(path, "r");
    if (fp == NULL)
    {
        printf("%s file not open\n", path);
        return -1;
    }

    for (i = 0; i < GE_FUNCTION_TABLE_SIZE; i++)
    {
        fgets(str, 64, fp);
        ptr = strtok(str, ",");
        GeArray[0][i] = atof(ptr);
        GeHistogramIndex[i] = atof(ptr);
        ptr = strtok(NULL, ",");
        GeArray[1][i] = atof(ptr);
    }

    fclose(fp);

    return 0;
}

/* Convert keV to Index */
static unsigned short ConvkeVtoIndex(unsigned short value)
{
    unsigned short res = 0, n = 0;
    double new_value = 0.0;

    for (n = 0; n < GE_FUNCTION_TABLE_SIZE; n++)
    {
        new_value = GeArray[0][n];

        if (new_value >= (double)value)
        {
            res = n;
            break;
        }
    }

    return res;
}

/* Convert Index to keV */
static unsigned short ConvIndextokeV(unsigned short value)
{
    unsigned short res = 0;

    if ((value >= 0) && (value < GE_FUNCTION_TABLE_SIZE))
    {
        res = (unsigned short)(GeArray[0][value]);
    }
    else
    {
        res = 0;
    }

    return res;
}

// Reset module
static int ProcResetModule(struct usb_dev_handle *dh)
{
    const unsigned short MIN = 0;
    const unsigned short MAX = 2;

    int res = 0;
    unsigned short level = 0;

    if (level > MAX)
    {
        printf("Module reset failed.(Parameter error)\n");

        return -1;
    }

    res = RdmUsb_ResetRequest(dh, level);
    if (res != RDMUSB_SUCCESS)
    {
        printf("Module reset failed.\n");

        return -1;
    }

    printf("Module reset.\n");

    return 0;
    ;
}

/* Get gamma-ray energy lower limit */
static int ProcGetEnergyThreshold(struct usb_dev_handle *dh)
{
    int res = 0;
    unsigned short data = 0;

    res = RdmUsb_GetEnergyThreshold(dh, &data);
    if (res != RDMUSB_SUCCESS)
    {
        printf("Get threshold failed.\n");

        return -1;
    }

    printf("Get threshold = %d\n", ConvIndextokeV(data));

    return 0;
}

/* Set gamma-ray energy lower limit */
static int ProcSetEnergyThreshold(struct usb_dev_handle *dh,
                                  unsigned short threshold)
{
    const unsigned short MIN = 0;
    const unsigned short MAX = 4095;

    int res = 0;

    if (threshold > MAX)
    {
        printf("Set threshold failed.(Parameter error)\n");

        return -1;
    }

    res = RdmUsb_SetEnergyThreshold(dh, ConvkeVtoIndex(threshold));
    if (res != RDMUSB_SUCCESS)
    {
        printf("Set threshold failed.\n");

        return -1;
    }

    printf("Set threshold = %d\n", threshold);

    return 0;
}

/* Get upper limits of effective range of air dose */
static int ProcGetRadiationUpperLimit(struct usb_dev_handle *dh)
{
    int res = 0;
    unsigned short data = 0;

    res = RdmUsb_GetRadiationLimit(dh, 1, &data);
    if (res != RDMUSB_SUCCESS)
    {
        printf("Get limit upper failed.\n");

        return -1;
    }

    printf("Get limit upper = %d\n", data);

    return 0;
}

/* Set upper limits of effective range of air dose */
static int ProcSetRadiationUpperLimit(struct usb_dev_handle *dh,
                                      unsigned short upper)
{
    const unsigned short MIN = 30;
    const unsigned short MAX = 2000;

    int res = 0;

    if (upper < MIN || upper > MAX)
    {
        printf("Set limit upper failed.(Parameter error)\n");

        return -1;
    }

    res = RdmUsb_SetRadiationLimit(dh, 1, upper);
    if (res != RDMUSB_SUCCESS)
    {
        printf("Set limit upper failed.\n");

        return -1;
    }

    printf("Set limit upper = %d\n", upper);

    return 0;
}

/* Get lower limits of effective range of air dose */
static int ProcGetRadiationLowerLimit(struct usb_dev_handle *dh)
{
    int res = 0;
    unsigned short data = 0;

    res = RdmUsb_GetRadiationLimit(dh, 0, &data);
    if (res != RDMUSB_SUCCESS)
    {
        printf("Get limit lower failed.\n");

        return -1;
    }

    printf("Get limit lower = %d\n", data);

    return 0;
}

/* Set lower limits of effective range of air dose */
static int ProcSetRadiationLowerLimit(struct usb_dev_handle *dh,
                                      unsigned short lower)
{
    const unsigned short MIN = 30;
    const unsigned short MAX = 2000;

    int res = 0;

    if (lower < MIN || lower > MAX)
    {
        printf("Set limit lower failed.(Parameter error)\n");

        return -1;
    }

    res = RdmUsb_SetRadiationLimit(dh, 0, lower);
    if (res != RDMUSB_SUCCESS)
    {
        printf("Set limit lower failed.\n");

        return -1;
    }

    printf("Set limit lower = %d\n", lower);

    return 0;
}

/* Acquisition request of acquired data */
static int ProcGetDacDataAndTemperature(struct usb_dev_handle *dh)
{
    const int RETRY_MAX = 5;

    char busy = 1;
    int count = 0, t = 0;
    int res = 0;
    int err_retry_cnt = 0;
    unsigned short old_index = 0, new_index = 0;
    unsigned short size = 0;
    unsigned short buffer[DATA_BUFFER_SIZE] = {0};
    double temperature = 0;

    /* Clear bulk buffer */
    res = RdmUsb_ClearBulkBuffer(dh);
    if (res != RDMUSB_SUCCESS)
    {
        printf("Clear bulk buffer failed.\n");

        return -1;
    }

    while (busy == 1)
    {
        if (IsAbortReq)
        {
            return 0;
        }

        memset(buffer, 0, DATA_BUFFER_SIZE);
        res = RdmUsb_GetDataAndTemperture(dh, &new_index, &size, buffer, &temperature);
        if (res != RDMUSB_SUCCESS)
        {
            if (res == RDMUSB_PACKET_ERROR)
            {
                printf("Packet error.\n");

                // Retry after wait
                usleep(20000); // 20ms wait

                if (err_retry_cnt >= RETRY_MAX)
                {
                    printf("Retry limit.\n");

                    return -1;
                }

                err_retry_cnt++;

                continue;
            }
            else
            {
                printf("Get data and temperature failed.\n");

                return -1;
            }
        }

        err_retry_cnt = 0;

        busy = 0;
    }

    if (busy == 0)
    {
        // new_index
        // size
        // buffer
        // temperature

        printf("Index value:%d\n", new_index);
        printf("Data count:%d\n", size);
        for (int n = 0; n < size; n++)
        {
            printf("Data[%d]:%d\n", n, buffer[n]);
        }
        printf("Temp. %.1lf℃\n", temperature);
    }

    return 0;
}

/* Get internal temperature */
static int ProcGetInternalTemperature(struct usb_dev_handle *dh)
{
    int res = 0;
    double temperature = 0;
    unsigned short data = 0;

    res = RdmUsb_HvpsGetTemperature(dh, &temperature, &data);
    if (res != RDMUSB_SUCCESS)
    {
        printf("Get internal Temp. failed.\n");

        return -1;
    }

    printf("Get internal Temp. %.1lf℃\n", temperature);

    return 0;
}

/* Get sievert and energy*/
static int ProcGetSievert(struct usb_dev_handle *dh,
                          double second,
                          int times, 
                          ros::Publisher &sv_pub 
                          )



{
    const int RETRY_MAX = 5;

    // std::string counts_filename = "counts_";
    // std::string sievert_filename = "sievert_";

    char busy = 1;
    FILE *ftcounts, *ftemp;
    int count = 0, t = 0;
    int res = 0;
    int err_retry_cnt = 0;
    unsigned short old_index = 0, new_index = 0;
    unsigned short size = 0;
    unsigned short buffer[DATA_BUFFER_SIZE] = {0};
    unsigned short sv[GE_FUNCTION_TABLE_SIZE] = {0};
    double temperature = 0;

    unsigned short threshold = 0;
    unsigned short area_lower = 0;
    unsigned short area_upper = 0;



    ////////////////////////    FILE    ////////////////////////////////   


    char* base_filename2 = "sievert.csv";
    char* base_filename3 = "counts.csv";


    // Check if the base filename already exists

    FILE* base_file2 = fopen(base_filename2, "r");
    if (base_file2 != NULL)
    {
        // If the base filename exists, find the next available sequence number
        fclose(base_file2);
        int sequence_number = 1;
        char new_filename[20];
        while (true)
        {
            sprintf(new_filename, "sievert_%d.csv", sequence_number);
            ftemp = fopen(new_filename, "r");
            if (ftemp == NULL)
            {
                // If the new filename doesn't exist, use it to save the new CSV file
                ftemp = fopen(new_filename, "w");
                break;
            }
            else
            {
                // If the new filename already exists, increment the sequence number and try again
                sequence_number++;
            }
        }
    }
    else
    {
        // If the base filename doesn't exist, use it to save the new CSV file
        ftemp = fopen(base_filename2, "w");

    }




    FILE* base_file3 = fopen(base_filename3, "r");
    if (base_file3 != NULL)
    {
                // If the base filename exists, find the next available sequence number
        fclose(base_file3);
        int sequence_number = 1;
        char new_filename[20];
        while (true)
        {
                    
            sprintf(new_filename, "counts_%d.csv", sequence_number);
            ftcounts = fopen(new_filename, "r");
            if (ftcounts == NULL)
            {
                        // If the new filename doesn't exist, use it to save the new CSV file
                ftcounts = fopen(new_filename, "w");
                break;
            }
            else
            {
                        // If the new filename already exists, increment the sequence number and try again
                sequence_number++;
            }
        }
    }
    else
    {
                // If the base filename doesn't exist, use it to save the new CSV file
        ftcounts = fopen(base_filename3, "w");

    }

    fprintf(ftcounts, "Energy (Kev2I), counts\n");
    fprintf(ftemp, "Time, Sievert (uSv/h) \n");


    /* Clear bulk buffer */
    res = RdmUsb_ClearBulkBuffer(dh);
    if (res != RDMUSB_SUCCESS)
    {
        printf("Clear bulk buffer failed.\n");

        return -1;
    }

    // Threshold
    res = RdmUsb_GetEnergyThreshold(dh, &threshold);
    if (res == RDMUSB_SUCCESS)
    {
        printf("Threshold = %d\n", ConvIndextokeV(threshold));
    }
    // Limit lower
    res = RdmUsb_GetRadiationLimit(dh, 0, &area_lower);
    if (res == RDMUSB_SUCCESS)
    {
        printf("Limit lower = %d\n", area_lower);
    }
    // Limit upper
    res = RdmUsb_GetRadiationLimit(dh, 1, &area_upper);
    if (res == RDMUSB_SUCCESS)
    {
        printf("Limit upper = %d\n", area_upper);
    }

    for (t = 0; t < times; t++)
    {
        if (IsAbortReq)
        {
            return 0;
        }

        memset(sv, 0, sizeof(unsigned short) * GE_FUNCTION_TABLE_SIZE);

        while (busy == 1)
        {
            if (IsAbortReq)
            {
                return 0;
            }

            memset(buffer, 0, DATA_BUFFER_SIZE);
            res = RdmUsb_GetDataAndTemperture(dh, &new_index, &size, buffer, &temperature);
            if (res != RDMUSB_SUCCESS)
            {
                if (res == RDMUSB_PACKET_ERROR)
                {
                    printf("Packet error.\n");
                }
                else
                {
                    printf("Get data and temperature failed.\n");
                }

                usleep(5000); // 5ms wait

                res = RdmUsb_ClearBulkBuffer(dh);
                if (res != RDMUSB_SUCCESS)
                {
                    printf("Clear bulk buffer failed.\n");

                    return -1;
                }

                if (err_retry_cnt >= RETRY_MAX)
                {
                    printf("Retry limit.\n");

                    return -1;
                }

                err_retry_cnt++;

                // Retry after wait
                usleep(20000); // 20ms wait

                continue;
            }

            err_retry_cnt = 0;

            if (old_index == new_index)
            {
                // Retry after wait
                usleep(20000); // 20ms wait

                continue;
            }

            old_index = new_index;
            for (int i = 0; i < size; i++)
            {
                int address = (buffer[i] >> 4) & 0x0FFF;

                if (address > ConvkeVtoIndex(area_lower) && address < ConvkeVtoIndex(area_upper))
                {
                    sv[address] += 1;
                }
            }
            count++;

            if (count > second * 10 - 1)
            {
                count = 0;
                busy = 0;
            }

            usleep(20000); // 20ms wait
        }

        if (busy == 0)
        {
            double sv_val = 0.0;
            double norm_sv = 0.0;
            busy = 1;


            for (int n = 0; n < size; n++)
            {

                int keV_Index = (buffer[n] >> 4) & 0x0FFF;

                if (keV_Index > ConvkeVtoIndex(area_lower) && keV_Index < ConvkeVtoIndex(area_upper))
                {
                  GeHistogramCount [keV_Index] += 1;
                }
                
                printf("Kev2I[%d]:%d\n", n, keV_Index);

            }


            for (int i = 0; i < GE_FUNCTION_TABLE_SIZE; i++)
            {
                sv_val += (double)sv[i] * GeArray[1][i];
            }

            /* pSv -> uSv/h */
            sv_val = sv_val * 3600 / second / 1E+6;
            printf("[%5d]sievert = %0.3f uSv/h\n", t, sv_val);
            
            
            fprintf(ftemp, "%d, %f\n", t,sv_val);
            
            norm_sv = 1/(1+exp((-10)*(sv_val-0.5)));
            
            // std_msgs::Float32 sv_digit;
            radiation::sievert sv_msg;
            sv_msg.sv = sv_val;
            
       

            // set the message timestamp
            sv_msg.header.stamp = ros::Time::now();
            
            
            sv_pub.publish(sv_msg);


            ros::spinOnce();

        }
    }
  

    /////////////////////   wright in counts    /////////////////////////////

    for (int i = 0; i < GE_FUNCTION_TABLE_SIZE; i++) 
    {      
        fprintf(ftcounts, "%f,%d\n", GeHistogramIndex[i], GeHistogramCount[i]);
    }


    printf("ended succesfully\n");
    fclose(ftemp);
    fclose(ftcounts);
    return 0;
}








/* Read EEPROM */
static int ProcReadEeprom(struct usb_dev_handle *dh,
                          unsigned short address)
{
    int res = 0;
    unsigned short data = 0;

    if (address != EEPROM_COMP_LEVEL &&
        address != EEPROM_ENERGY_LOWER &&
        address != EEPROM_ENERGY_UPPER &&
        address != EEPROM_CONVERT_USV)
    {
        printf("Read EEPROM failed.(Parameter error)\n");

        return -1;
    }

    res = RdmUsb_ReadEeprom(dh, address, &data);
    if (res != RDMUSB_SUCCESS)
    {
        printf("Read EEPROM failed.\n");

        return -1;
    }

    switch (address)
    {
    case EEPROM_COMP_LEVEL:
        printf("Read threshold(EEPROM) = %d\n", data);
        break;

    case EEPROM_ENERGY_LOWER:
        printf("Read limit lower(EEPROM) = %d\n", data);
        break;

    case EEPROM_ENERGY_UPPER:
        printf("Read limit upper(EEPROM) = %d\n", data);
        break;

    case EEPROM_CONVERT_USV:
        printf("Read calibration coefficient for acquired radiation dose(EEPROM) = %d\n", data);
        break;

    default:
        printf("Read EEPROM failed.(Parameter error)\n");

        return -1;
    }

    return 0;
}

/* main function */
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "radaition"); // Initialize the node with a name
    ros::start();
    ros::NodeHandle nh; // Create a NodeHandle instance
  
    ROS_INFO_STREAM("radiation detection start " << nh.getNamespace());

    ros::Publisher sievert_pub = nh.advertise<radiation::sievert>("/radiation/sievert", 20);

    // ros::Publisher kev_pub = nh.advertise<radiation::keV>("/radiation/keV", 20);



    static char command_request_str[ARGUMENTS_BUF_MAX] = {0};
    static char command_param_str_list[ARGUMENTS_PARAM_NUM_MAX][ARGUMENTS_BUF_MAX] = {0};
    static int command_param_cnt = 0;

    struct usb_bus *bus;
    struct usb_device *dev;
    struct usb_dev_handle *dh;

    int cmd_err = 1;
    int res = 0;

    /* Initialize variable */
    memset(command_request_str, 0, ARGUMENTS_BUF_MAX);
    memset(command_param_str_list, 0, ARGUMENTS_PARAM_NUM_MAX * ARGUMENTS_BUF_MAX);
    command_param_cnt = 0;

    /* Signal define */
    if (signal(SIGINT, abort_handler) == SIG_ERR)
    {
        exit(999);
    }





    /* Command check */
    cmd_err = checkCommandArguments(&argc, argv,
                                    command_request_str,
                                    command_param_str_list, &command_param_cnt);

    if (cmd_err == 0)
    {
        // Target confirmation
        if (strcmp(command_request_str, ARGUMENTS_TARTGET_VERSION) == 0)
        {
            //	--version : Version display
            printf("Application Version: %s\n", APP_VERSION);
        }
        else if (strcmp(command_request_str, ARGUMENTS_TARTGET_HELP) == 0)
        {
            //	--help : Help (Usage) display
            printf("%s\n", "");
            printf("%s\n", "Usage:  RadiationDetectorLinux [REQUEST] [PARAM1] [PARAM2]...");
            printf("%s\n", "[REQUEST]");
            printf("%s\n", "--version           : application version display");
            printf("%s\n", "--help              : help display");
            printf("%s\n", "");
            printf("%s\n", "--reset             : Reset module");
            printf("%s\n", "--g_eng_th          : Get gamma-ray energy lower limit");
            printf("%s\n", "--s_eng_th          : Set gamma-ray energy lower limit(PARAM1:value)");
            printf("%s\n", "                    :  value");
            printf("%s\n", "                    :   0[keV] - 4095[keV]");
            printf("%s\n", "--g_rd_up           : Get upper limits of effective range of air dose");
            printf("%s\n", "--s_rd_up           : Set upper limits of effective range of air dose(PARAM1:value)");
            printf("%s\n", "                    :  value");
            printf("%s\n", "                    :   30[keV] - 2000[keV]");
            printf("%s\n", "--g_rd_lo           : Get lower limits of effective range of air dose");
            printf("%s\n", "--s_rd_lo           : Set lower limits of effective range of air dose(PARAM1:value)");
            printf("%s\n", "                    :  value");
            printf("%s\n", "                    :   30[keV] - 2000[keV]");
            printf("%s\n", "--g_data_and_temp   : Get Data and temperature");
            printf("%s\n", "--g_sievert         : Get sievert(PARAM1:second,PARAM2:time)");
            printf("%s\n", "                    :  second");
            printf("%s\n", "                    :   update cycle[s]");
            printf("%s\n", "                    :  time");
            printf("%s\n", "                    :   total time[s]");
            printf("%s\n", "--g_int_temp        : Get internal temperature");
            printf("%s\n", "--r_eeprom          : Read EEPROM(PARAM1:address(HEX))");
            printf("%s\n", "                    :  address");
            printf("%s\n", "                    :   0Ah:Comparator threshold value (Default value: 30 keV)");
            printf("%s\n", "                    :       A threshold value used as the default value at startup is stored.");
            printf("%s\n", "                    :   0Ch:Lower limit of energy range (Default value: 30 keV)");
            printf("%s\n", "                    :       This sets the gamma-ray energy range that will be valid when");
            printf("%s\n", "                    :       converting to Sv/h.");
            printf("%s\n", "                    :   0Eh:Higher limit of energy range (Default value: 2000 keV)");
            printf("%s\n", "                    :       This sets the gamma-ray energy range that will be valid when");
            printf("%s\n", "                    :       converting to Sv/h.");
            printf("%s\n", "                    :   10h:Calibration coefficient for acquired radiation dose (Default value: 1000 (1x))");
            printf("%s\n", "                    :       Coefficient for correcting the acquired radiation dose rate that has ");
            printf("%s\n", "                    :       changed over time.");
            printf("%s\n", "");
        }
        else
        {
            /* USB Initialize */
            bus = USB_init();
            dev = USB_find(bus, dev);
            if (dev == NULL)
            {
                fprintf(stderr, "Device not found\n");
                exit(1);
            }
            printf("USB initialization success\n");

            /* Device Open */
            dh = USB_open(dev);
            if (dh == NULL)
            {
                fprintf(stderr, "Device open failure\n");
                exit(2);
            }
            printf("Device Open success\n");

            /* Load G(E) function file */
            res = Load_GE_FunctionFile(GE_FUNCTION_FILENAME);
            if (res != 0)
            {
                exit(3);
            }

            // Target confirmation
            if (strcmp(command_request_str, ARGUMENTS_TARTGET_RESET) == 0)
            {
                //	--reset : Reset module
                if (ProcResetModule(dh) == 0)
                {
                    // Success
                }
            }
            // Target confirmation
            else if (strcmp(command_request_str, ARGUMENTS_TARTGET_GET_ENERGY_THRESHOLD) == 0)
            {
                //	--g_eng_th : Get gamma-ray energy lower limit
                if (ProcGetEnergyThreshold(dh) == 0)
                {
                    // Success
                }
            }
            else if (strcmp(command_request_str, ARGUMENTS_TARTGET_SET_ENERGY_THRESHOLD) == 0)
            {
                //	--s_eng_th : Set gamma-ray energy lower limit
                if (command_param_cnt >= 1)
                {
                    char *endptr;
                    unsigned short threshold = strtol(&command_param_str_list[0][0], &endptr, 10);
                    if (strlen(endptr) == 0)
                    {
                        if (ProcSetEnergyThreshold(dh, threshold) == 0)
                        {
                            // Success
                        }
                    }
                }
            }
            else if (strcmp(command_request_str, ARGUMENTS_TARTGET_GET_RADIATION_UPPER_LIMIT) == 0)
            {
                //	--g_rd_up : Get upper limits of effective range of air dose
                if (ProcGetRadiationUpperLimit(dh) == 0)
                {
                    // Success
                }
            }
            else if (strcmp(command_request_str, ARGUMENTS_TARTGET_SET_RADIATION_UPPER_LIMIT) == 0)
            {
                //	--s_rd_up : Get upper limits of effective range of air dose
                if (command_param_cnt >= 1)
                {
                    char *endptr;
                    unsigned short upper_limit = strtol(&command_param_str_list[0][0], &endptr, 10);
                    if (strlen(endptr) == 0)
                    {
                        if (ProcSetRadiationUpperLimit(dh, upper_limit) == 0)
                        {
                            // Success
                        }
                    }
                }
            }
            else if (strcmp(command_request_str, ARGUMENTS_TARTGET_GET_RADIATION_LOWER_LIMIT) == 0)
            {
                //	--g_rd_lo : Get lower limits of effective range of air dose
                if (ProcGetRadiationLowerLimit(dh) == 0)
                {
                    // Success
                }
            }
            else if (strcmp(command_request_str, ARGUMENTS_TARTGET_SET_RADIATION_LOWER_LIMIT) == 0)
            {
                //	--s_rd_lo : Get lower limits of effective range of air dose
                if (command_param_cnt >= 1)
                {
                    char *endptr;
                    unsigned short lower_limit = strtol(&command_param_str_list[0][0], &endptr, 10);
                    if (strlen(endptr) == 0)
                    {
                        if (ProcSetRadiationLowerLimit(dh, lower_limit) == 0)
                        {
                            // Success
                        }
                    }
                }
            }
            else if (strcmp(command_request_str, ARGUMENTS_TARTGET_GET_DATA_AND_TEMPERATURE) == 0)
            {
                //	--g_data_and_temp : Get data and temperature
                if (ProcGetDacDataAndTemperature(dh) == 0)
                {
                    // Success
                }
            }
            else if (strcmp(command_request_str, ARGUMENTS_TARTGET_GET_SIEVERT) == 0)
            {
                //	--g_sievert : Get sievert
                if (command_param_cnt >= 2)
                {
                    char *endptr[2];
                    double second = strtod(&command_param_str_list[0][0], &endptr[0]);
                    unsigned short times = strtol(&command_param_str_list[1][0], &endptr[1], 10);
                    if (strlen(endptr[0]) == 0 && strlen(endptr[1]) == 0)
                    {
                        if (ProcGetSievert(dh, second, times, sievert_pub) == 0)
                        {
                            // Success
                        }
                    }
                }
            }
            else if (strcmp(command_request_str, ARGUMENTS_TARTGET_GET_INTERNAL_TEMPERATURE) == 0)
            {
                //	--g_int_temp : Get internal temperature
                if (ProcGetInternalTemperature(dh) == 0)
                {
                    // Success
                }
            }
            else if (strcmp(command_request_str, ARGUMENTS_TARTGET_READ_EEPROM) == 0)
            {
                //	--r_eeprom : Read EEPROM
                if (command_param_cnt >= 1)
                {
                    char *endptr;
                    unsigned short address = strtol(&command_param_str_list[0][0], &endptr, 16);
                    printf("address - %02Xh\n", address);
                    if (strlen(endptr) == 0)
                    {
                        if (ProcReadEeprom(dh, address) == 0)
                        {
                            // Success
                        }
                    }
                }
            }
            else
            {
                printf("=> REQUEST error\n");
            }

            /* USB close */
            printf("USB Close\n");
            USB_close(dh);
            printf("USB End\n");
        }
    }
        ros::shutdown();
    return 0;
}
