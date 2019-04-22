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
 ** \brief This sample demonstrates UART data receive and transfer by interrupt.
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
#define USART_EI_NUM                    INT_USART3_EI
#define USART_TI_NUM                    INT_USART3_TI
#define USART_TCI_NUM                   INT_USART3_TCI

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void ClkInit(void);
static void UsartRxIrqCallback(void);
static void UsartTxIrqCallback(void);
static void UsartTxCmpltIrqCallback(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint16_t m_u16RxData;

static const stc_usart_uart_init_t m_stcInitCfg = {
    UsartIntClkCkNoOutput,
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
 ** \brief Initialize Clock.
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
    stcIrqRegiCfg.enIRQn = Int000_IRQn;
    stcIrqRegiCfg.pfnCallback = UsartRxIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART_RI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Set USART RX error IRQ */
    stcIrqRegiCfg.enIRQn = Int001_IRQn;
    stcIrqRegiCfg.pfnCallback = UsartErrIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART_EI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Set USART TX IRQ */
    stcIrqRegiCfg.enIRQn = Int002_IRQn;
    stcIrqRegiCfg.pfnCallback = UsartTxIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART_TI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Set USART TX complete IRQ */
    stcIrqRegiCfg.enIRQn = Int003_IRQn;
    stcIrqRegiCfg.pfnCallback = UsartTxCmpltIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART_TCI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /*Enable RX && RX interupt function*/
    USART_FuncCmd(USART_CH, UsartRx, Enable);
    USART_FuncCmd(USART_CH, UsartRxInt, Enable);

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
