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
 ** \brief OTS sample
 **
 **   - 2018-10-26  1.0  Wuze First version for Device Driver Library of
 **     OTS
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
/* Experimental trigger pin definition. */
#define TRIGGER_PORT                PortC
#define TRIGGER_PIN                 Pin01

/* Trigger pin state definition. */
#define TRIGGER_PIN_PRESSED         Reset

#define OTS_EXP_AVG_CNT             ((uint8_t)50)

#define SYS_CLOCK_FREQ_MHZ          (SystemCoreClock / 1000000ul)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void OtsConfig(void);
static void OtsClockConfig(void);
static void OtsInitConfig(void);

static void TriggerPinConfig(void);
static en_flag_status_t TriggerPinGetState(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Main function
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    uint8_t u8ExpCount = 0u;
    uint8_t i = 0;
    uint8_t j = 0;
    float32_t f32Sum;
    stc_ots_init_t stcOtsInit;

    uint16_t  u16Dr1;
    uint16_t  u16Dr2;
    uint16_t  u16Ecr;
    float32_t f32OtsA = 0.0f;

    /* Initialize UART for printing. Baud rate 115200. */
    Ddl_UartInit();

    /* Config OTS. */
    OtsConfig();

    /* Config trigger pin. */
    TriggerPinConfig();

    /***************** Configuration end, application start **************/

    while (1u)
    {
        printf("\nOTS Scaling Experiment.");

        printf("\nPress the key (PC1) to trigger the test.");
        while (TRIGGER_PIN_PRESSED != TriggerPinGetState()) Ddl_Delay1ms(1u);
        while (TRIGGER_PIN_PRESSED == TriggerPinGetState()) Ddl_Delay1ms(1u);
        u8ExpCount++;
        printf("\nOTS experiment T%d start.", u8ExpCount);
        printf("\nOTS Clock: HRC");
        if (0u == M4_ICG->ICG1_f.HRCFREQSEL)
        {
            printf("(20MHz).");
        }
        else
        {
            printf("(16MHz).");
        }
        stcOtsInit.enAutoOff = OtsAutoOff_Disable;
        stcOtsInit.enClkSel  = OtsClkSel_Hrc;

        for (j = 0u; j < 2u; j++)
        {
            OTS_Init(&stcOtsInit);
            f32Sum = 0.0f;
            for (i = 0u; i < OTS_EXP_AVG_CNT; i++)
            {
                OTS_ScalingExperiment(&u16Dr1, &u16Dr2, &u16Ecr, &f32OtsA);
                printf("\nOTS experiment T%d::DR1 = %.4x, DR2 = %.4x, ECR = %.4x, A%d = %f.",\
                          u8ExpCount, u16Dr1, u16Dr2, u16Ecr, u8ExpCount, f32OtsA);
                f32Sum += f32OtsA;
                Ddl_Delay1ms(100u);
            }
            f32OtsA = (f32Sum / (float32_t)OTS_EXP_AVG_CNT);
            printf("\nOTS experiment T%d done. Average: A%d = %f.",\
                      u8ExpCount, u8ExpCount, f32OtsA);

            if (j == 0u)
            {
                printf("\nOTS Clock: XTAL");
                stcOtsInit.enClkSel = OtsClkSel_Xtal;
            }
        }

        if (u8ExpCount >= 2u)
          u8ExpCount = 0u;
    }
}

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  OTS configuration, including initial configuration and
 **         clock configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void OtsConfig(void)
{
    OtsInitConfig();
    OtsClockConfig();
}

/**
 *******************************************************************************
 ** \brief  OTS initial configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void OtsInitConfig(void)
{
    stc_ots_init_t stcOtsInit;

    stcOtsInit.enAutoOff = OtsAutoOff_Disable;
    stcOtsInit.enClkSel  = OtsClkSel_Hrc;
    stcOtsInit.u8ClkFreq = SYS_CLOCK_FREQ_MHZ;

    /* 1. Enable OTS. */
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_OTS, Enable);
    /* 2. Initialize OTS. */
    OTS_Init(&stcOtsInit);
}

/**
 *******************************************************************************
 ** \brief  OTS clock configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void OtsClockConfig(void)
{
#if (OTS_CLK_SEL == OTS_CLK_SEL_HRC)
    /* Enable HRC for OTS. */
    CLK_HrcCmd(Enable);
    /* Enable XTAL32 while clock selecting HRC. */
    CLK_Xtal32Cmd(Enable);
#else
    /* Enable XTAL for OTS. */
    CLK_XtalCmd(Enable);
#endif

    /* Enable LRC for OTS. */
    CLK_LrcCmd(Enable);
}

/**
 *******************************************************************************
 ** \brief  Trigger pin configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void TriggerPinConfig(void)
{
    stc_port_init_t stcPortInit;

    /* configuration structure initialization. */
    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_In;
    stcPortInit.enExInt   = Disable;
    stcPortInit.enPullUp  = Enable;

     /* Experimental trigger pin initialization. */
    PORT_Init(TRIGGER_PORT, TRIGGER_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief  Get trigger pin's state.
 **
 ** \param  None.
 **
 ** \retval Reset                       Trigger pin is pressed.
 ** \retval Set                         Trigger pin is released.
 **
 ******************************************************************************/
static en_flag_status_t TriggerPinGetState(void)
{
    en_flag_status_t enPinSt;

    enPinSt = PORT_GetBit(TRIGGER_PORT, TRIGGER_PIN);
    if (TRIGGER_PIN_PRESSED == enPinSt)
    {
        Ddl_Delay1ms(10);
        enPinSt = PORT_GetBit(TRIGGER_PORT, TRIGGER_PIN);
    }

    return enPinSt;
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
