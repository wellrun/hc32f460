/******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co.,Ltd. All rights reserved.
 *
 * This software is owned and published by:
 * Huada Semiconductor Co.,Ltd ("HDSC").
 *
 * BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
 * BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
 *
 * This software contains source code for use with HDSC
 * components. This software is licensed by HDSC to be adapted only
 * for use in systems utilizing HDSC components. HDSC shall not be
 * responsible for misuse or illegal use of this software for devices not
 * supported herein. HDSC is providing this software "AS IS" and will
 * not be responsible for issues arising from incorrect user implementation
 * of the software.
 *
 * Disclaimer:
 * HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
 * REGARDING THE SOFTWARE (INCLUDING ANY ACCOMPANYING WRITTEN MATERIALS),
 * ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
 * WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
 * WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
 * WARRANTY OF NONINFRINGEMENT.
 * HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
 * NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
 * LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
 * LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
 * INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
 * SAVINGS OR PROFITS,
 * EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
 * INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
 * FROM, THE SOFTWARE.
 *
 * This software may be replicated in part or whole for the licensed use,
 * with the restriction that this Disclaimer and Copyright notice must be
 * included with each copy of this software, whether used in part or whole,
 * at all times.
 */
/******************************************************************************/
/** \file usbh_usr.c
 **
 ** A detailed description is available at
 ** @link
        This file includes the user application layer.
    @endlink
 **
 **   - 2018-05-21  1.0  gouwei First version for USB demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdio.h>
#include "usbh_usr.h"
#include "ff.h"       /* FATFS */
#include "usbh_msc_core.h"
#include "usbh_msc_scsi.h"
#include "usbh_msc_bot.h"
#include "hc32_ddl.h"


/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define IMAGE_BUFFER_SIZE    720

#define BUTTON_PORT         PortD
#define BUTTON_PIN          Pin03

#define GET_BUTTON_KEY()    PORT_GetBit(PortD, Pin03)

/* LED0 Port/Pin definition */
#define  LED0_PORT          PortE
#define  LED0_PIN           Pin06

/* LED1 Port/Pin definition */
#define  LED1_PORT          PortA
#define  LED1_PIN           Pin07

/* LED0~1 toggle definition */
#define  LED0_TOGGLE()      PORT_Toggle(LED0_PORT, LED0_PIN)
#define  LED1_TOGGLE()      PORT_Toggle(LED1_PORT, LED1_PIN)

/* USBH_USR_Private_Macros */
extern USB_OTG_CORE_HANDLE          USB_OTG_Core;


/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
uint8_t USBH_USR_ApplicationState = USH_USR_FS_INIT;
uint8_t filenameString[15]  = {0};

FATFS fatfs;
FIL file;
uint8_t Image_Buf[IMAGE_BUFFER_SIZE];
uint8_t line_idx = 0;

/*  Points to the DEVICE_PROP structure of current device */
/*  The purpose of this register is to speed up the execution */

USBH_Usr_cb_TypeDef USR_cb =
{
    USBH_USR_Init,
    USBH_USR_DeInit,
    USBH_USR_DeviceAttached,
    USBH_USR_ResetDevice,
    USBH_USR_DeviceDisconnected,
    USBH_USR_OverCurrentDetected,
    USBH_USR_DeviceSpeedDetected,
    USBH_USR_Device_DescAvailable,
    USBH_USR_DeviceAddressAssigned,
    USBH_USR_Configuration_DescAvailable,
    USBH_USR_Manufacturer_String,
    USBH_USR_Product_String,
    USBH_USR_SerialNum_String,
    USBH_USR_EnumerationDone,
    USBH_USR_UserInput,
    USBH_USR_MSC_Application,
    USBH_USR_DeviceNotSupported,
    USBH_USR_UnrecoveredError

};

/* USBH_USR_Private_Constants */
const uint8_t MSG_HOST_INIT[]        = "> Host Library Initialized\n";
const uint8_t MSG_DEV_ATTACHED[]     = "> Device Attached \n";
const uint8_t MSG_DEV_DISCONNECTED[] = "> Device Disconnected\n";
const uint8_t MSG_DEV_ENUMERATED[]   = "> Enumeration completed \n";
const uint8_t MSG_DEV_HIGHSPEED[]    = "> High speed device detected\n";
const uint8_t MSG_DEV_FULLSPEED[]    = "> Full speed device detected\n";
const uint8_t MSG_DEV_LOWSPEED[]     = "> Low speed device detected\n";
const uint8_t MSG_DEV_ERROR[]        = "> Device fault \n";

const uint8_t MSG_MSC_CLASS[]        = "> Mass storage device connected\n";
const uint8_t MSG_HID_CLASS[]        = "> HID device connected\n";
const uint8_t MSG_DISK_SIZE[]        = "> Size of the disk in MBytes: \n";
const uint8_t MSG_LUN[]              = "> LUN Available in the device:\n";
const uint8_t MSG_ROOT_CONT[]        = "> Exploring disk flash ...\n";
const uint8_t MSG_WR_PROTECT[]       = "> The disk is write protected\n";
const uint8_t MSG_UNREC_ERROR[]      = "> UNRECOVERED ERROR STATE\n";


static uint8_t Explore_Disk (char* path , uint8_t recu_level);
static uint8_t Image_Browser (char* path);
static void     Show_Image(void);
static void     Toggle_Leds(void);

/* USBH_USR_Private_Functions */
typedef struct
{
    uint8_t  pic_head[2];
    uint16_t pic_size_l;
    uint16_t pic_size_h;
    uint16_t pic_nc1;
    uint16_t pic_nc2;
    uint16_t pic_data_address_l;
    uint16_t pic_data_address_h;
    uint16_t pic_message_head_len_l;
    uint16_t pic_message_head_len_h;
    uint16_t pic_w_l;
    uint16_t pic_w_h;
    uint16_t pic_h_l;
    uint16_t pic_h_h;
    uint16_t pic_bit;
    uint16_t pic_dip;
    uint16_t pic_zip_l;
    uint16_t pic_zip_h;
    uint16_t pic_data_size_l;
    uint16_t pic_data_size_h;
    uint16_t pic_dipx_l;
    uint16_t pic_dipx_h;
    uint16_t pic_dipy_l;
    uint16_t pic_dipy_h;
    uint16_t pic_color_index_l;
    uint16_t pic_color_index_h;
    uint16_t pic_other_l;
    uint16_t pic_other_h;
}BMP_HEAD;

BMP_HEAD bmp;

typedef struct
{
    uint8_t  r;                         /* RED */
    uint8_t  g;                         /* GREEN */
    uint8_t  b;                         /* BLUE */
}BMP_POINT;

BMP_POINT point;

#define RGB565CONVERT(red, green, blue) (uint16_t) (((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3))

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  Displays the message on LCD for host lib initialization
 ** \param  None
 ** \retval None
 ******************************************************************************/
void USBH_USR_Init(void)
{
    static uint8_t startup = 0;
    stc_port_init_t stcPortInit;

    if(startup == 0 )
    {
        startup = 1;

        MEM_ZERO_STRUCT(stcPortInit);
        /*initiallize LED port*/
        stcPortInit.enPinMode = Pin_Mode_Out;
        stcPortInit.enExInt = Enable;
        stcPortInit.enPullUp = Enable;
        /* LED0 and LED1 Port/Pin initialization */
        PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);
        PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);

#ifdef USE_USB_OTG_HS
        printf(" USB OTG HS MSC Host\n");
#else
        printf(" USB OTG FS MSC Host\n");
#endif
        printf("> USB Host library started.\n");
        printf("     USB Host Library v2.1.0\n" );
    }
}

/**
 *******************************************************************************
 ** \brief  Displays the message on LCD on device attached
 ** \param  None
 ** \retval None
 ******************************************************************************/
void USBH_USR_DeviceAttached(void)
{
    printf((void *)MSG_DEV_ATTACHED);
}

/**
 *******************************************************************************
 ** \brief  USBH_USR_UnrecoveredError
 ** \param  None
 ** \retval None
 ******************************************************************************/
void USBH_USR_UnrecoveredError (void)
{

  /* Set default screen color*/
    printf((void *)MSG_UNREC_ERROR);
}

/**
 *******************************************************************************
 ** \brief  Device disconnect event
 ** \param  None
 ** \retval None
 ******************************************************************************/
void USBH_USR_DeviceDisconnected (void)
{
    /* Set default screen color*/
    printf((void *)MSG_DEV_DISCONNECTED);
}

/**
 *******************************************************************************
 ** \brief  USBH_USR_ResetUSBDevice
 ** \param  None
 ** \retval None
 ******************************************************************************/
void USBH_USR_ResetDevice(void)
{
    /* callback for USB-Reset */
}

/**
 *******************************************************************************
 ** \brief  USBH_USR_DeviceSpeedDetected
 ** \param  DeviceSpeed : USB speed
 ** \retval None
 ******************************************************************************/
void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed)
{
    if(DeviceSpeed == HPRT0_PRTSPD_HIGH_SPEED)
    {
        printf((void *)MSG_DEV_HIGHSPEED);
    }
    else if(DeviceSpeed == HPRT0_PRTSPD_FULL_SPEED)
    {
        printf((void *)MSG_DEV_FULLSPEED);
    }
    else if(DeviceSpeed == HPRT0_PRTSPD_LOW_SPEED)
    {
        printf((void *)MSG_DEV_LOWSPEED);
    }
    else
    {
        printf((void *)MSG_DEV_ERROR);
    }
}

/**
 *******************************************************************************
 ** \brief  USBH_USR_Device_DescAvailable
 ** \param  DeviceDesc : device descriptor
 ** \retval None
 ******************************************************************************/
void USBH_USR_Device_DescAvailable(void *DeviceDesc)
{
    USBH_DevDesc_TypeDef *hs;
    hs = DeviceDesc;

    printf("VID : %04Xh\n" , (uint32_t)(*hs).idVendor);
    printf("PID : %04Xh\n" , (uint32_t)(*hs).idProduct);
}

/**
 *******************************************************************************
 ** \brief  USBH_USR_DeviceAddressAssigned
 ** \param  None
 ** \retval None
 ******************************************************************************/
void USBH_USR_DeviceAddressAssigned(void)
{

}

/**
 *******************************************************************************
 ** \brief  USBH_USR_Configuration_DescAvailable
 ** \param  cfgDesc : Configuration desctriptor
 ** \param  itfDesc : Interface desctriptor
 ** \param  epDesc : Endpoint desctriptor
 ** \retval None
 ******************************************************************************/
void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc)
{
    USBH_InterfaceDesc_TypeDef *id;

    id = itfDesc;

    if((*id).bInterfaceClass  == 0x08)
    {
        printf((void *)MSG_MSC_CLASS);
    }
    else if((*id).bInterfaceClass  == 0x03)
    {
        printf((void *)MSG_HID_CLASS);
    }
}

/**
 *******************************************************************************
 ** \brief  Displays the message on LCD for Manufacturer String
 ** \param  ManufacturerString
 ** \retval None
 ******************************************************************************/
void USBH_USR_Manufacturer_String(void *ManufacturerString)
{
    printf("Manufacturer : %s\n", (char *)ManufacturerString);
}

/**
 *******************************************************************************
 ** \brief  Displays the message on LCD for product String
 ** \param  ProductString
 ** \retval None
 ******************************************************************************/
void USBH_USR_Product_String(void *ProductString)
{
    printf("Product : %s\n", (char *)ProductString);
}

/**
 *******************************************************************************
 ** \brief  Displays the message on LCD for SerialNum_String
 ** \param  SerialNumString
 ** \retval None
 ******************************************************************************/
void USBH_USR_SerialNum_String(void *SerialNumString)
{
    printf( "Serial Number : %s\n", (char *)SerialNumString);
}

/**
 *******************************************************************************
 ** \brief  User response request is displayed to ask application jump to class
 ** \param  None
 ** \retval None
 ******************************************************************************/
void USBH_USR_EnumerationDone(void)
{

    /* Enumeration complete */
    printf((void *)MSG_DEV_ENUMERATED);
    printf("To see the root content of the disk : \n" );
    printf("Press USER KEY...\n");

}

/**
 *******************************************************************************
 ** \brief  Device is not supported
 ** \param  None
 ** \retval None
 ******************************************************************************/
void USBH_USR_DeviceNotSupported(void)
{
    printf ("> Device not supported.\n");
}

/**
 *******************************************************************************
 ** \brief  User Action for application state entry
 ** \param  None
 ** \retval USBH_USR_Status : User response for key button
 ******************************************************************************/
USBH_USR_Status USBH_USR_UserInput(void)
{
    USBH_USR_Status usbh_usr_status;

    usbh_usr_status = USBH_USR_NO_RESP;

    /*Key B3 is in polling mode to detect user action */
    if(GET_BUTTON_KEY() == Reset)
    {

        usbh_usr_status = USBH_USR_RESP_OK;

    }

    return usbh_usr_status;
}

/**
 *******************************************************************************
 ** \brief  Over Current Detected on VBUS
 ** \param  None
 ** \retval None
 ******************************************************************************/
void USBH_USR_OverCurrentDetected (void)
{
    printf ("Overcurrent detected.\n");
}

/**
 *******************************************************************************
 ** \brief  Demo application for mass storage
 ** \param  None
 ** \retval None
 ******************************************************************************/
int USBH_USR_MSC_Application(void)
{
    FRESULT res;
    uint8_t writeTextBuff[] = "HDSC Connectivity line Host Demo application using FAT_FS   ";
    uint16_t bytesWritten, bytesToWrite;

    switch(USBH_USR_ApplicationState)
    {
        case USH_USR_FS_INIT:
            /* Initialises the File System*/
            if ( f_mount( 0, &fatfs ) != FR_OK )
            {
                /* efs initialisation fails*/
                printf("> Cannot initialize File System.\n");
                return(-1);
            }
            printf("> File System initialized.\n");
            printf("> Disk capacity : %d Bytes\n", USBH_MSC_Param.MSCapacity * \
              USBH_MSC_Param.MSPageLength);
            printf("> Disk capacity : %d * %d = %d Bytes\n",
                USBH_MSC_Param.MSCapacity,
                USBH_MSC_Param.MSPageLength,
                USBH_MSC_Param.MSCapacity * \
            USBH_MSC_Param.MSPageLength);
            if(USBH_MSC_Param.MSWriteProtect == DISK_WRITE_PROTECTED)
            {
                printf((void *)MSG_WR_PROTECT);
            }

            USBH_USR_ApplicationState = USH_USR_FS_READLIST;
            break;

        case USH_USR_FS_READLIST:
            printf((void *)MSG_ROOT_CONT);
            Explore_Disk("0:/", 1);
            line_idx = 0;
            USBH_USR_ApplicationState = USH_USR_FS_WRITEFILE;
            break;

        case USH_USR_FS_WRITEFILE:
            printf("Press USER KEY to write file\n");
            USB_OTG_BSP_mDelay(100);

            /*Key B3 in polling*/
            while((HCD_IsDeviceConnected(&USB_OTG_Core)) && \
                (GET_BUTTON_KEY() != Reset))
            {
                Toggle_Leds();
            }
            /* Writes a text file, HC32.TXT in the disk*/
            printf("> Writing File to disk flash ...\n");
            if(USBH_MSC_Param.MSWriteProtect == DISK_WRITE_PROTECTED)
            {
                printf ( "> Disk flash is write protected \n");
                USBH_USR_ApplicationState = USH_USR_FS_DRAW;
                break;
            }

            /* Register work area for logical drives */
            f_mount(0, &fatfs);

            if(f_open(&file, "0:HC32.TXT",FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
            {
                /* Write buffer to file */
                bytesToWrite = sizeof(writeTextBuff);
                res= f_write (&file, writeTextBuff, bytesToWrite, (void *)&bytesWritten);

                if((bytesWritten == 0) || (res != FR_OK)) /*EOF or Error*/
                {
                    printf("> HC32.TXT CANNOT be writen.\n");
                }
                else
                {
                    printf("> 'HC32.TXT' file created\n");
                }

                /*close file and filesystem*/
                f_close(&file);
                f_mount(0, NULL);
            }
            else
            {
                printf ("> HC32.TXT created in the disk\n");
            }

            USBH_USR_ApplicationState = USH_USR_FS_DRAW;
            printf("To start Image slide show Press USER KEY.\n");

            break;

        case USH_USR_FS_DRAW:
            /*Key B3 in polling*/
            while((HCD_IsDeviceConnected(&USB_OTG_Core)) && \
            (GET_BUTTON_KEY() != Reset))
            {
                Toggle_Leds();
            }

            while(HCD_IsDeviceConnected(&USB_OTG_Core))
            {
                if ( f_mount( 0, &fatfs ) != FR_OK )
                {
                    /* fat_fs initialisation fails*/
                    return(-1);
                }
                return Image_Browser("0:/");
            }
            break;
        default: break;
    }
    return(0);
}

/**
 *******************************************************************************
 ** \brief  Displays disk content
 ** \param  path: pointer to root path
 ** \param  recu_level
 ** \retval uint8_t
 ******************************************************************************/
static uint8_t Explore_Disk (char* path , uint8_t recu_level)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    char *fn;
    char tmp[14];

    res = f_opendir(&dir, path);
    if (res == FR_OK)
    {
        while(HCD_IsDeviceConnected(&USB_OTG_Core))
        {
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0)
            {
                break;
            }
            if (fno.fname[0] == '.')
            {
                continue;
            }

            fn = fno.fname;
            strcpy(tmp, fn);

            line_idx++;
            if(line_idx > 9)
            {
                line_idx = 0;
                printf("Press USER KEY to continue...\n");

                /*Key B3 in polling*/
                while((HCD_IsDeviceConnected(&USB_OTG_Core)) && \
                (GET_BUTTON_KEY() != Reset))
                {
                    Toggle_Leds();
                }
            }

            if(recu_level == 1)
            {
                printf("   |__");
            }
            else if(recu_level == 2)
            {
                printf("   |   |__");
            }
            if((fno.fattrib & AM_MASK) == AM_DIR)
            {
                strcat(tmp, "\n");
                printf((void *)tmp);
            }
            else
            {
                strcat(tmp, "\n");
                printf((void *)tmp);
            }

            if(((fno.fattrib & AM_MASK) == AM_DIR)&&(recu_level == 1))
            {
                Explore_Disk(fn, 2);
            }
        }
    }
    return res;
}

static uint8_t Image_Browser (char* path)
{
    FRESULT res;
    uint8_t ret = 1;
    FILINFO fno;
    DIR dir;
    char *fn;

    res = f_opendir(&dir, path);
    if (res == FR_OK)
    {
        for (;;)
        {
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0) break;
            if (fno.fname[0] == '.') continue;

            fn = fno.fname;

            if (fno.fattrib & AM_DIR)
            {
                continue;
            }
            else
            {
                if((strstr(fn, "bmp")) || (strstr(fn, "BMP")))
                {
                    res = f_open(&file, fn, FA_OPEN_EXISTING | FA_READ);
                    Show_Image();
                    USB_OTG_BSP_mDelay(100);
                    ret = 0;
                    while((HCD_IsDeviceConnected(&USB_OTG_Core)) && \
                    (GET_BUTTON_KEY() != Reset))
                    {
                        Toggle_Leds();
                    }
                    f_close(&file);
                }
            }
        }
    }

#ifdef USE_USB_OTG_HS
    printf(" USB OTG HS MSC Host\n");
#else
    printf(" USB OTG FS MSC Host\n");
#endif
    printf ("     USB Host Library v2.1.0\n" );
    printf("> Disk capacity : %d * %d = %d Bytes\n",
        USBH_MSC_Param.MSCapacity,
        USBH_MSC_Param.MSPageLength,
        USBH_MSC_Param.MSCapacity * \
        USBH_MSC_Param.MSPageLength);
    USBH_USR_ApplicationState = USH_USR_FS_READLIST;
    return ret;
}

/**
 *******************************************************************************
 ** \brief  Displays BMP image
 ** \param  None
 ** \retval None
 ******************************************************************************/
static void Show_Image(void)
{
    uint16_t i = 0;
    uint16_t numOfReadBytes = 0;
    FRESULT res;

    f_lseek (&file, 54-6);

    while (HCD_IsDeviceConnected(&USB_OTG_Core))
    {
        res = f_read(&file, Image_Buf, IMAGE_BUFFER_SIZE, (void *)&numOfReadBytes);
        if((numOfReadBytes == 0) || (res != FR_OK)) /*EOF or Error*/
        {
            break;
        }
        for(i = 0 ; i < IMAGE_BUFFER_SIZE; i+= 3)
        {
        }
    }
}

/**
 *******************************************************************************
 ** \brief  Toggle leds to shows user input state
 ** \param  None
 ** \retval None
 ******************************************************************************/
static void Toggle_Leds(void)
{
    static uint32_t i;
    if (i++ == 0x10000)
    {
        LED0_TOGGLE();
        LED1_TOGGLE();
        i = 0;
    }
}

/**
 *******************************************************************************
 ** \brief  Deint User state and associated variables
 ** \param  None
 ** \retval None
 ******************************************************************************/
void USBH_USR_DeInit(void)
{
    USBH_USR_ApplicationState = USH_USR_FS_INIT;
}


/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
