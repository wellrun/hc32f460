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
 ** \brief This example demonstrates how to use DCU compare function.
 **
 **   - 2018-10-15  1.0  Hongjh First version for Device Driver Library of DCU
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
#define DCU_UNIT                        M4_DCU1

/* LED0(D23: red color) Port/Pin definition */
#define LED0_PORT                       PortE
#define LED0_PIN                        Pin06

/* LED1(D26: green color) Port/Pin definition */
#define LED1_PORT                       PortA
#define LED1_PIN                        Pin07

/* LED0 & LED1 */
#define LED0_ON()                       PORT_SetBits(LED0_PORT, LED0_PIN)
#define LED0_OFF()                      PORT_ResetBits(LED0_PORT, LED0_PIN)
#define LED1_ON()                       PORT_SetBits(LED1_PORT, LED1_PIN)
#define LED1_OFF()                      PORT_ResetBits(LED1_PORT, LED1_PIN)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void LedInit(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint16_t m_au8Data0Val[4] = {0x00, 0x22, 0x44, 0x88};
static uint16_t m_au8Data1Val[4] = {0x00, 0x11, 0x55, 0x88};
static uint16_t m_au8Data2Val[4] = {0x00, 0x11, 0x55, 0x88};

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Initialize LED.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void LedInit(void)
{
    stc_port_init_t stcPortInit;

    LED0_OFF();
    LED1_OFF();

    /* LED0&LED1 Port/Pin initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief  Main function of project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_dcu_init_t stcDcuInit;
    en_result_t enTestResult = Ok;

    /* Initialize LED */
    LedInit();

    /* Enable peripheral clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DCU1, Enable);

    /* Initialize DCU */
    MEM_ZERO_STRUCT(stcDcuInit);
    stcDcuInit.u32IntSel = 0;
    stcDcuInit.enIntWinMode = DcuIntInvalid;
    stcDcuInit.enDataSize = DcuDataBit8;
    stcDcuInit.enOperation = DcuOpCompare;
    stcDcuInit.enCmpTriggerMode = DcuCmpTrigbyData0;
    DCU_Init(DCU_UNIT, &stcDcuInit);

    /* DATA0 = DATA1  &&  DATA0 = DATA2 */
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData1, m_au8Data1Val[0]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData2, m_au8Data2Val[0]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData0, m_au8Data0Val[0]);
    if ((Set != DCU_GetIrqFlag(DCU_UNIT, DcuIntEq1)) ||
        (Set != DCU_GetIrqFlag(DCU_UNIT, DcuIntEq2)))
    {
        enTestResult = Error;
    }
    else
    {
    }

    DCU_ClearIrqFlag(DCU_UNIT, DcuIntEq1);
    DCU_ClearIrqFlag(DCU_UNIT, DcuIntEq2);

    /* DATA0 > DATA1  &&  DATA0 > DATA2 */
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData1, m_au8Data1Val[1]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData2, m_au8Data2Val[1]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData0, m_au8Data0Val[1]);
    if ((Set != DCU_GetIrqFlag(DCU_UNIT, DcuIntGt1)) ||
        (Set != DCU_GetIrqFlag(DCU_UNIT, DcuIntGt2)))
    {
        enTestResult = Error;
    }
    else
    {
    }

    DCU_ClearIrqFlag(DCU_UNIT, DcuIntGt1);
    DCU_ClearIrqFlag(DCU_UNIT, DcuIntGt2);

    /* DATA0 < DATA1  &&  DATA0 < DATA2 */
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData1, m_au8Data1Val[2]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData2, m_au8Data2Val[2]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData0, m_au8Data0Val[2]);
    if ((Set != DCU_GetIrqFlag(DCU_UNIT, DcuIntLs1)) ||
        (Set != DCU_GetIrqFlag(DCU_UNIT, DcuIntLs2)))
    {
        enTestResult = Error;
    }
    else
    {
    }

    DCU_ClearIrqFlag(DCU_UNIT, DcuIntLs1);
    DCU_ClearIrqFlag(DCU_UNIT, DcuIntLs2);

    /* DATA0 = DATA1  &&  DATA0 = DATA2 */
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData1, m_au8Data1Val[3]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData2, m_au8Data2Val[3]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData0, m_au8Data0Val[3]);
    if ((Set != DCU_GetIrqFlag(DCU_UNIT, DcuIntEq1)) ||
        (Set != DCU_GetIrqFlag(DCU_UNIT, DcuIntEq2)))
    {
        enTestResult = Error;
    }
    else
    {
    }

    DCU_ClearIrqFlag(DCU_UNIT, DcuIntEq1);
    DCU_ClearIrqFlag(DCU_UNIT, DcuIntEq2);

    if (Ok == enTestResult)
    {
        LED1_ON();  /* Test pass && meet the expected */
    }
    else
    {
        LED0_ON();  /* Test fail && don't meet the expected */
    }

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
