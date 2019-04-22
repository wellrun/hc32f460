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
 ** \brief The example of ICG WDT Interrupt function
 **
 **   - 2018-10-24  1.0  Yangjp First version for Device Driver Library of ICG.
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
/* LED0 Port/Pin definition */
#define LED0_PORT                       PortE
#define LED0_PIN                        Pin06

#define LED0_ON()                       PORT_SetBits(LED0_PORT, LED0_PIN)
#define LED0_OFF()                      PORT_ResetBits(LED0_PORT, LED0_PIN)
#define LED0_TOGGLE()                   PORT_Toggle(LED0_PORT, LED0_PIN)

/* LED1 Port/Pin definition */
#define LED1_PORT                       PortA
#define LED1_PIN                        Pin07

#define LED1_ON()                       PORT_SetBits(LED1_PORT, LED1_PIN)
#define LED1_OFF()                      PORT_ResetBits(LED1_PORT, LED1_PIN)
#define LED1_TOGGLE()                   PORT_Toggle(LED1_PORT, LED1_PIN)

/* KEY0 Port/Pin definition */
#define KEY0_PORT                       PortD
#define KEY0_PIN                        Pin03

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t u8ExIntCnt = 0;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief WDT interrupt callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Wdt_IrqCallback(void)
{
    en_flag_status_t enFlagSta;

    enFlagSta = WDT_GetFlag(WdtFlagCountUnderflow);
    /* WDT underflow interrupt */
    if (Set == enFlagSta)
    {
        WDT_ClearFlag(WdtFlagCountUnderflow);
        /* Normal mode */
        if (0u == u8ExIntCnt)
        {
            LED0_TOGGLE();
        }
        /* Sleep mode */
        else
        {
            LED1_TOGGLE();
        }
    }
}

/**
 *******************************************************************************
 ** \brief ExtInt3 callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt03_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh03))
    {
        u8ExIntCnt++;
        if (u8ExIntCnt >= 2)
        {
            u8ExIntCnt = 0u;
        }
        LED0_OFF();
        LED1_OFF();
        EXINT_IrqFlgClr(ExtiCh03);
    }
}

/**
 *******************************************************************************
 ** \brief KEY0(SW2) init function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Sw2_Init(void)
{
    stc_port_init_t stcPortInit;
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Set PD03 as External Int Ch.3 input */
    stcPortInit.enExInt = Enable;
    PORT_Init(KEY0_PORT, KEY0_PIN, &stcPortInit);

    stcExtiConfig.enExitCh = ExtiCh03;
    /* Filter setting */
    stcExtiConfig.enFilterEn = Disable;
    stcExtiConfig.enFltClk = Pclk3Div1;
    /* Both edge */
    stcExtiConfig.enExtiLvl = ExIntFallingEdge;
    EXINT_Init(&stcExtiConfig);

    /* Select External Int Ch.3 */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ3;
    /* Register External Int to Vect.No.007 */
    stcIrqRegiConf.enIRQn = Int007_IRQn;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = ExtInt03_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

/**
 *******************************************************************************
 ** \brief WDT interrupt config
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Wdt_InterruptConfig(void)
{
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Select Int source WDT */
    stcIrqRegiConf.enIntSrc = INT_WDT_REFUDF;
    /* Register WDT Int to Vect.No.006 */
    stcIrqRegiConf.enIRQn = Int006_IRQn;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = Wdt_IrqCallback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

/**
 *******************************************************************************
 ** \brief  main function for ICG WDT Interrupt function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    /**
     ***************************************************************************
     ** Modify hc32f46x_icg.h file of defines
     ** #define ICG0_WDT_HARDWARE_START         ICG_FUNCTION_ON
     **
     ** #define ICG0_WDT_AUTS                   WDT_AUTO_START_AFTER_RESET
     ** #define ICG0_WDT_ITS                    WDT_INTERRUPT_REQUEST
     ** #define ICG0_WDT_PERI                   WDT_COUNT_UNDERFLOW_CYCLE_16384
     ** #define ICG0_WDT_CKS                    WDT_COUNT_PCLK3_DIV512
     ** #define ICG0_WDT_WDPT                   WDT_0To100PCT
     ** #define ICG0_WDT_SLPOFF                 WDT_SPECIAL_MODE_COUNT_CONTINUE
     **************************************************************************/
    stc_port_init_t stcPortInit;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);

    /* LED0 Port/Pin initialization */
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);
    LED0_OFF();

    /* LED1 Port/Pin initialization */
    LED1_OFF();
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);

    /* Configure WDT interrupt */
    Wdt_InterruptConfig();
    /* Key0 Port/Pin initialization */
    Sw2_Init();

    while (1)
    {
        /* Sleep mode */
        if (1u == u8ExIntCnt)
        {
            PWC_EnterSleepMd();
            __WFI();
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
