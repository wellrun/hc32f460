/*******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software is owned and published by:
 * Huada Semiconductor Co., Ltd. ("HDSC").
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
/** \file main.c
 **
 ** \brief CRC sample
 **
 **   - 2019-03-11  1.0  Wuze First version for Device Driver Library of
 **     CRC
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void CrcConfig(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Main function.
 **
 ** \param  None.
 **
 ** \retval int32_t return value, if needed.
 **
 ******************************************************************************/
int32_t main(void)
{
    bool bFlag;
    uint16_t u16InitVal;
    uint16_t u16Checksum;
    uint16_t au16Data[2u] = {0x1234, 0x5678};

    uint32_t u32InitVal;
    uint32_t u32Checksum;
    uint32_t au32Data[2u] = {0x12345678, 0x87654321};

    /* Config CRC. */
    CrcConfig();

    /* Config UART for printing. Baud rate 115200. */
    Ddl_UartInit();

    /***************** Configuration end, application start **************/
    while (1u)
    {
        /* CRC16 usage. */
        u16InitVal = 0x0u;
        CRC_Init(CRC_SEL_16B | CRC_REFIN_DISABLE | CRC_REFOUT_DISABLE | CRC_XOROUT_DISABLE);
        u16Checksum = CRC_Calculate16B(u16InitVal, au16Data, 2u);
        printf("\nCRC16 result = %.4x", u16Checksum);
        bFlag = CRC_Check16B(u16InitVal, u16Checksum, au16Data, 2u);
        printf("\nCRC16 flag = %d", bFlag);

        /* Change the CRC configuration. */
        /* The bits of the checksum will be transposed if CRC_REFOUT is enabled. */
        CRC_Init(CRC_SEL_16B | CRC_REFIN_DISABLE | CRC_REFOUT_ENABLE | CRC_XOROUT_DISABLE);
        u16Checksum = CRC_Calculate16B(u16InitVal, au16Data, 2u);
        printf("\nCRC16 result = %.4x", u16Checksum);
        bFlag = CRC_Check16B(u16InitVal, u16Checksum, au16Data, 2u);
        printf("\nCRC16 flag = %d", bFlag);

        /* CRC32 usage. */
        u32InitVal = 0xFFFFFFFFu;
        CRC_Init(CRC_SEL_32B | CRC_REFIN_ENABLE | CRC_REFOUT_ENABLE | CRC_XOROUT_DISABLE);
        u32Checksum = CRC_Calculate32B(u32InitVal, au32Data, 2u);
        printf("\nCRC32 result = %.8x", u32Checksum);
        bFlag = CRC_Check32B(u32InitVal, u32Checksum, au32Data, 2u);
        printf("\nCRC32 flag = %d", bFlag);

        /* Changes the CRC configuration. */
        CRC_Init(CRC_SEL_32B | CRC_REFIN_DISABLE | CRC_REFOUT_DISABLE | CRC_XOROUT_ENABLE);
        u32Checksum = CRC_Calculate32B(u32InitVal, au32Data, 2u);
        printf("\nCRC32 result = %.8x", u32Checksum);
        bFlag = CRC_Check32B(u32InitVal, u32Checksum, au32Data, 2u);
        printf("\nCRC32 flag = %d", bFlag);
    }
}

/**
 *******************************************************************************
 ** \brief  CRC initial configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void CrcConfig(void)
{
    /* 1. Enable CRC. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_CRC, Enable);

    /* 2. Initializes CRC here or before every CRC calculation. */
    CRC_Init(CRC_SEL_16B | CRC_REFIN_DISABLE | CRC_REFOUT_DISABLE | CRC_XOROUT_DISABLE);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
