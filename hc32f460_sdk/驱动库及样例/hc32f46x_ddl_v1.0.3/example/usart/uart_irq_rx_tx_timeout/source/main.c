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
 ** \brief This sample demonstrates how to check UART data receive by timeout.
 **
 **   - 2018-11-27  1.0  Hongjh First version for Device Driver Library of USART
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
/* USART channel definition */
#define USART_CH                        M4_USART3

/* USART baudrate definition */
#define USART_BAUDRATE                  (115200)

/* USART RX Port/Pin definition */
#define USART_RX_PORT                   PortE
#define USART_RX_PIN                    Pin04
#define USART_RX_FUNC                   Func_Usart3_Rx

/* USART TX Port/Pin definition */
#define USART_TX_PORT                   PortE
#define USART_TX_PIN                    Pin05
#define USART_TX_FUNC                   Func_Usart3_Tx

/* USART interrupt number  */
#define USART_RI_NUM                    INT_USART3_RI
#define USART_RI_IRQn                   Int000_IRQn
#define USART_EI_NUM                    INT_USART3_EI
#define USART_EI_IRQn                   Int001_IRQn
#define USART_RTO_NUM                   INT_USART3_RTO
#define USART_RTO_IRQn                  Int002_IRQn
#define USART_TI_NUM                    INT_USART3_TI
#define USART_TI_IRQn                   Int003_IRQn
#define USART_TCI_NUM                   INT_USART3_TCI
#define USART_TCI_IRQn                  Int004_IRQn

/* LED(D23: red color) Port/Pin definition */
#define LED_PORT                        PortE
#define LED_PIN                         Pin06

/* LED operation */
#define LED_ON()                        PORT_SetBits(LED_PORT, LED_PIN)
#define LED_OFF()                       PORT_ResetBits(LED_PORT, LED_PIN)

/* Timer0 unit definition */
#define TMR_UNIT                        M4_TMR02

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void ClkInit(void);
static void LedInit(void);
static void Timer0Init(void);
static void UsartRxIrqCallback(void);
static void UsartTimeoutIrqCallback(void);
static void UsartTxIrqCallback(void);
static void UsartTxCmpltIrqCallback(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint16_t m_u16RxData;

static const stc_usart_uart_init_t m_stcInitCfg = {
    UsartIntClkCkOutput,
    UsartClkDiv_1,
    UsartDataBits8,
    UsartDataLsbFirst,
    UsartOneStopBit,
    UsartParityNone,
    UsartSamleBit8,
    UsartStartBitFallEdge,
};

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Initliaze Clock.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void ClkInit(void)
{
    stc_clk_xtal_cfg_t   stcXtalCfg;
    stc_clk_mpll_cfg_t   stcMpllCfg;
    en_clk_sys_source_t  enSysClkSrc;
    stc_clk_sysclk_cfg_t stcSysClkCfg;

    MEM_ZERO_STRUCT(enSysClkSrc);
    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv  = ClkSysclkDiv1;  /* Max 168MHz */
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;  /* Max 84MHz */
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;  /* Max 168MHz */
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;  /* Max 84MHz */
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;  /* Max 60MHz */
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;  /* Max 42MHz */
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;  /* Max 84MHz */
    CLK_SysClkConfig(&stcSysClkCfg);

    /* Switch system clock source to MPLL. */
    /* Use Xtal as MPLL source. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* MPLL config. */
    stcMpllCfg.pllmDiv = 1;
    stcMpllCfg.plln = 50;
    stcMpllCfg.PllpDiv = 4;
    stcMpllCfg.PllqDiv = 4;
    stcMpllCfg.PllrDiv = 4;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_5);
    EFM_Lock();

    /* Enable MPLL. */
    CLK_MpllCmd(Enable);

    /* Wait MPLL ready. */
    while (Set != CLK_GetFlagStatus(ClkFlagMPLLRdy))
    {
    }

    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);
}

/**
 *******************************************************************************
 ** \brief Initliaze LED.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void LedInit(void)
{
    stc_port_init_t stcPortInit;

    LED_OFF();

    /* LED0&LED1 Port/Pin initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    PORT_Init(LED_PORT, LED_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief Initliaze Timer0.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Timer0Init(void)
{
    stc_clk_freq_t stcClkTmp;
    stc_tim0_base_init_t stcTimerCfg;
    stc_tim0_trigger_init_t StcTimer0TrigInit;
    uint32_t u32PeriphClk = (TMR_UNIT == M4_TMR01) ? PWC_FCG2_PERIPH_TIM01 : PWC_FCG2_PERIPH_TIM02;

    MEM_ZERO_STRUCT(stcClkTmp);
    MEM_ZERO_STRUCT(stcTimerCfg);
    MEM_ZERO_STRUCT(StcTimer0TrigInit);

    /* Timer0 peripheral enable */
    PWC_Fcg2PeriphClockCmd(u32PeriphClk, Enable);

    /* Clear CNTAR register for channel A */
    TIMER0_WriteCntReg(TMR_UNIT, Tim0_ChannelA, 0u);
    TIMER0_WriteCntReg(TMR_UNIT, Tim0_ChannelB, 0u);

    /* Config register for channel A */
    stcTimerCfg.Tim0_CounterMode = Tim0_Async;
    stcTimerCfg.Tim0_AsyncClockSource = Tim0_XTAL32;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv8;
    stcTimerCfg.Tim0_CmpValue = 32000;
    TIMER0_BaseInit(TMR_UNIT, Tim0_ChannelA, &stcTimerCfg);

    /* Clear compare flag */
    TIMER0_ClearFlag(TMR_UNIT, Tim0_ChannelA);

    /* Config timer0 hardware trigger */
    StcTimer0TrigInit.Tim0_InTrigEnable = false;
    StcTimer0TrigInit.Tim0_InTrigClear = true;
    StcTimer0TrigInit.Tim0_InTrigStart = true;
    StcTimer0TrigInit.Tim0_InTrigStop = false;
    TIMER0_HardTriggerInit(TMR_UNIT, Tim0_ChannelA, &StcTimer0TrigInit);
}

/**
 *******************************************************************************
 ** \brief USART RX irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void UsartRxIrqCallback(void)
{
    m_u16RxData = USART_RecData(USART_CH);
    USART_FuncCmd(USART_CH, UsartTxAndTxEmptyInt, Enable);
}

/**
 *******************************************************************************
 ** \brief USART RX error irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void UsartErrIrqCallback(void)
{
    if (Set == USART_GetStatus(USART_CH, UsartFrameErr))
    {
        USART_ClearStatus(USART_CH, UsartFrameErr);
    }
    else
    {
    }

    if (Set == USART_GetStatus(USART_CH, UsartParityErr))
    {
        USART_ClearStatus(USART_CH, UsartParityErr);
    }
    else
    {
    }

    if (Set == USART_GetStatus(USART_CH, UsartOverrunErr))
    {
        USART_ClearStatus(USART_CH, UsartOverrunErr);
    }
    else
    {
    }
}

/**
 *******************************************************************************
 ** \brief USART timeout irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void UsartTimeoutIrqCallback(void)
{
    LED_ON();

    USART_ClearStatus(USART_CH, UsartRxTimeOut);
}

/**
 *******************************************************************************
 ** \brief USART TX irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void UsartTxIrqCallback(void)
{
    USART_SendData(USART_CH, m_u16RxData);
    USART_FuncCmd(USART_CH, UsartTxEmptyInt, Disable);
    USART_FuncCmd(USART_CH, UsartTxCmpltInt, Enable);
}

/**
 *******************************************************************************
 ** \brief USART TX complete irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void UsartTxCmpltIrqCallback(void)
{
    LED_OFF();
    USART_FuncCmd(USART_CH, UsartTxCmpltInt, Disable);
    USART_FuncCmd(USART_CH, UsartTx, Disable);
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
    en_result_t enRet = Ok;
    stc_irq_regi_conf_t stcIrqRegiCfg;
    uint32_t u32Fcg1Periph = PWC_FCG1_PERIPH_USART1 | PWC_FCG1_PERIPH_USART2 | PWC_FCG1_PERIPH_USART3 | PWC_FCG1_PERIPH_USART4;

    /* Initialize Clock */
    ClkInit();

    /* Initialize LED */
    LedInit();

    /* Initialize Timer0 */
    Timer0Init();

    /* Enable peripheral clock */
    PWC_Fcg1PeriphClockCmd(u32Fcg1Periph, Enable);

    /* Initialize USART IO */
    PORT_SetFunc(USART_RX_PORT, USART_RX_PIN, USART_RX_FUNC, Disable);
    PORT_SetFunc(USART_TX_PORT, USART_TX_PIN, USART_TX_FUNC, Disable);

    /* Initialize UART */
    enRet = USART_UART_Init(USART_CH, &m_stcInitCfg);
    if (enRet != Ok)
    {
        while (1)
        {
        }
    }
    else
    {
    }

    /* Set baudrate */
    enRet = USART_SetBaudrate(USART_CH, USART_BAUDRATE);
    if (enRet != Ok)
    {
        while (1)
        {
        }
    }
    else
    {
    }

    /* Set USART RX IRQ */
    stcIrqRegiCfg.enIRQn = USART_RI_IRQn;
    stcIrqRegiCfg.pfnCallback = UsartRxIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART_RI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Set USART RX error IRQ */
    stcIrqRegiCfg.enIRQn = USART_EI_IRQn;
    stcIrqRegiCfg.pfnCallback = UsartErrIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART_EI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Set USART RX timeout error IRQ */
    stcIrqRegiCfg.enIRQn = USART_RTO_IRQn;
    stcIrqRegiCfg.pfnCallback = UsartTimeoutIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART_RTO_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Set USART TX IRQ */
    stcIrqRegiCfg.enIRQn = USART_TI_IRQn;
    stcIrqRegiCfg.pfnCallback = UsartTxIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART_TI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Set USART TX complete IRQ */
    stcIrqRegiCfg.enIRQn = USART_TCI_IRQn;
    stcIrqRegiCfg.pfnCallback = UsartTxCmpltIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART_TCI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /*Enable RX && RX interupt && timeout interrupt function*/
    USART_FuncCmd(USART_CH, UsartRx, Enable);
    USART_FuncCmd(USART_CH, UsartRxInt, Enable);
    USART_FuncCmd(USART_CH, UsartTimeOut, Enable);
    USART_FuncCmd(USART_CH, UsartTimeOutInt, Enable);

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
