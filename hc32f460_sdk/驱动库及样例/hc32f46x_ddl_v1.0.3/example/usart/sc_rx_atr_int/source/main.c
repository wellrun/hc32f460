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
 ** \brief This sample demonstrates UART receive Smart-card ATR by interrupt.
 **
 **   - 2018-12-06  1.0  Hongjh First version for Device Driver Library of USART
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief buffer handle
 **
 ******************************************************************************/
typedef struct stc_buf_handle
{
    uint8_t u8Cnt;
    uint8_t u8Size;
    uint8_t *pu8Buf;
} stc_buf_handle_t;

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* Serial USART channel definition */
#define SERIEL_USART_CH                 M4_USART3

/* Serial USART baudrate definition */
#define SERIEL_USART_BAUDRATE           (115200)

/* Serial USART TX Port/Pin definition */
#define SERIEL_TX_PORT                  PortE
#define SERIEL_TX_PIN                   Pin05
#define SERIEL_TX_FUNC                  Func_Usart3_Tx

/* Smart-card USART channel definition */
#define SC_USART_CH                     M4_USART2

/* Smart-card USART baudrate definition */
#define SC_USART_BAUDRATE               (9600)

/* Smart-card USART RX Port/Pin definition */
#define SC_RX_PORT                      PortA
#define SC_RX_PIN                       Pin03
#define SC_RX_FUNC                      Func_Usart2_Rx

/* Smart-card USART TX Port/Pin definition */
#define SC_TX_PORT                      PortA
#define SC_TX_PIN                       Pin02
#define SC_TX_FUNC                      Func_Usart2_Tx

/* Smart-card USART CK Port/Pin definition */
#define SC_CK_PORT                      PortD
#define SC_CK_PIN                       Pin07
#define SC_CK_FUNC                      Func_Usart_Ck

/* Smart-card CD Port/Pin definition */
#define SC_CD_PORT                      PortB
#define SC_CD_PIN                       Pin01

/* Smart-card RST Port/Pin definition */
#define SC_RST_PORT                     PortA
#define SC_RST_PIN                      Pin00

/* Smart-card power on Port/Pin definition */
#define SC_PWREN_PORT                   PortA
#define SC_PWREN_PIN                    Pin01

/* Smart-card CD pin operation */
#define IS_CARD_INSERTED()              (Reset == PORT_GetBit(SC_CD_PORT, SC_CD_PIN))

/* Smart-card reset pin operation */
#define SC_RESET_LOW()                  PORT_ResetBits(SC_RST_PORT, SC_RST_PIN)
#define SC_RESET_HIGH()                 PORT_SetBits(SC_RST_PORT, SC_RST_PIN)

/* Smart-card power pin operation */
#define SC_POWER_ON()                   PORT_ResetBits(SC_PWREN_PORT, SC_PWREN_PIN)
#define SC_POWER_OFF()                  PORT_SetBits(SC_PWREN_PORT, SC_PWREN_PIN)

/* Smart-card USART RX buffer size */
#define SC_USART_RX_BUF_SIZE            (100)

/* Smart-card USART interrupt number  */
#define SC_USART_RI_NUM                 INT_USART2_RI
#define SC_USART_RI_IRQn                Int001_IRQn
#define SC_USART_EI_NUM                 INT_USART2_EI
#define SC_USART_EI_IRQn                Int002_IRQn

/* Smart-card initial character TS: 0x3B */
#define SC_INITIAL_CHARACTER_TS         (0x3B)

/* Smart-card dummy command for test */
#define SC_DUMMY_CMD                    (0xA5)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void ClkInit(void);
static void ScPinInit(void);
static void ScUsartRxIrqCallback(void);
static void ScUsartErrIrqCallback(void);
static void SerialUsartInit(void);
static void SerialUsartSend(uint8_t *pu8Data, uint8_t u8Size);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t m_au8RxBuf[SC_USART_RX_BUF_SIZE];

static stc_buf_handle_t m_stcRxBufHanlde = {0, sizeof(m_au8RxBuf), m_au8RxBuf};

static const stc_usart_sc_init_t m_stcScInitCfg = {
    UsartIntClkCkOutput,
    UsartClkDiv_1,
    UsartDataLsbFirst,
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
 ** \brief Initialize serial USART.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void SerialUsartInit(void)
{
    en_result_t enRet = Ok;
    uint32_t u32Fcg1Periph = PWC_FCG1_PERIPH_USART1 | PWC_FCG1_PERIPH_USART2 |
                             PWC_FCG1_PERIPH_USART3 | PWC_FCG1_PERIPH_USART4;
    stc_usart_uart_init_t m_stcUsartInitCfg = {
        UsartIntClkCkNoOutput,
        UsartClkDiv_1,
        UsartDataBits8,
        UsartDataLsbFirst,
        UsartOneStopBit,
        UsartParityNone,
        UsartSamleBit8,
        UsartStartBitFallEdge,
    };

    /* Enable peripheral clock */
    PWC_Fcg1PeriphClockCmd(u32Fcg1Periph, Enable);

    /* Initialize USART IO */
    PORT_SetFunc(SERIEL_TX_PORT, SERIEL_TX_PIN, SERIEL_TX_FUNC, Disable);

    /* Initialize USART */
    enRet = USART_UART_Init(SERIEL_USART_CH, &m_stcUsartInitCfg);
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
    enRet = USART_SetBaudrate(SERIEL_USART_CH, SERIEL_USART_BAUDRATE);
    if (enRet != Ok)
    {
        while (1)
        {
        }
    }
    else
    {
    }

    /*Enable TX function*/
    USART_FuncCmd(SERIEL_USART_CH, UsartTx, Enable);
}

/**
 *******************************************************************************
 ** \brief Serial USART send data.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void SerialUsartSend(uint8_t *pu8Data, uint8_t u8Size)
{
    uint8_t i;

    for (i = 0; i < u8Size; i++)
    {
        while ( Reset == USART_GetStatus(SERIEL_USART_CH, UsartTxEmpty))
        {
        }

        USART_SendData(SERIEL_USART_CH, *pu8Data++);
    }
}

/**
 *******************************************************************************
 ** \brief Initialize card-detect pin.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void ScCdPinInit(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    /* CD Pin initialization */
    stcPortInit.enPinMode = Pin_Mode_In;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    PORT_Init(SC_CD_PORT, SC_CD_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief Initialize smart card interface pin.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void ScPinInit(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    /* RX/TX/CK Pin initialization */
    PORT_SetFunc(SC_RX_PORT, SC_RX_PIN, SC_RX_FUNC, Disable);
    PORT_SetFunc(SC_TX_PORT, SC_TX_PIN, SC_TX_FUNC, Disable);
    PORT_SetFunc(SC_CK_PORT, SC_CK_PIN, SC_CK_FUNC, Disable);

    /* Set RST pin output High-level */
    PORT_SetBits(SC_RST_PORT, SC_RST_PIN);
    /* RST Pin initialization */
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    PORT_Init(SC_RST_PORT, SC_RST_PIN, &stcPortInit);

    /* Set Power on pin output High-level: Power off */
    PORT_SetBits(SC_PWREN_PORT, SC_PWREN_PIN);
    /* Power on Pin initialization */
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    PORT_Init(SC_PWREN_PORT, SC_PWREN_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief Smart-card USART RX irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void ScUsartRxIrqCallback(void)
{
    if (m_stcRxBufHanlde.u8Cnt < m_stcRxBufHanlde.u8Size)
    {
        m_stcRxBufHanlde.pu8Buf[m_stcRxBufHanlde.u8Cnt++] = USART_RecData(SC_USART_CH);;
    }
    else
    {
    }
}

/**
 *******************************************************************************
 ** \brief Smart-card USART RX error irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void ScUsartErrIrqCallback(void)
{
    if (Set == USART_GetStatus(SC_USART_CH, UsartFrameErr))
    {
        USART_ClearStatus(SC_USART_CH, UsartFrameErr);
    }
    else
    {
    }

    if (Set == USART_GetStatus(SC_USART_CH, UsartParityErr))
    {
        USART_ClearStatus(SC_USART_CH, UsartParityErr);
    }
    else
    {
    }

    if (Set == USART_GetStatus(SC_USART_CH, UsartOverrunErr))
    {
        USART_ClearStatus(SC_USART_CH, UsartOverrunErr);
    }
    else
    {
    }
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

    /* Initialize card-detect IO */
    ScCdPinInit();

    while (!IS_CARD_INSERTED())
    {
    }

    /* Initialize smart card IO */
    ScPinInit();
    SC_POWER_OFF();
    SC_RESET_LOW();
    USART_DeInit(SC_USART_CH);

    /* Initialize UART */
    enRet = USART_SC_Init(SC_USART_CH, &m_stcScInitCfg);
    if(enRet != Ok)
    {
        while (1)
        {
        }
    }
    else
    {
    }

    /* Set baudrate */
    enRet = USART_SetBaudrate(SC_USART_CH, SC_USART_BAUDRATE);
    if(enRet != Ok)
    {
        while (1)
        {
        }
    }
    else
    {
    }

    /* Set USART RX IRQ */
    stcIrqRegiCfg.enIRQn = SC_USART_RI_IRQn;
    stcIrqRegiCfg.pfnCallback = ScUsartRxIrqCallback;
    stcIrqRegiCfg.enIntSrc = SC_USART_RI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Set USART RX error IRQ */
    stcIrqRegiCfg.enIRQn = SC_USART_EI_IRQn;
    stcIrqRegiCfg.pfnCallback = ScUsartErrIrqCallback;
    stcIrqRegiCfg.enIntSrc = SC_USART_EI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /*Enable RX && RX interupt function*/
    USART_FuncCmd(SC_USART_CH, UsartRxInt, Enable);
    USART_FuncCmd(SC_USART_CH, UsartRx, Enable);

    /* Cold reset :smart card */
    SC_POWER_ON();
    Ddl_Delay1ms(1);
    SC_RESET_HIGH();
    Ddl_Delay1ms(150);  /* Delay for receving Smart-card ATR */

    /* Initialize serial USART channel and send ATR to computer */
    SerialUsartInit();
    SerialUsartSend(m_stcRxBufHanlde.pu8Buf, m_stcRxBufHanlde.u8Cnt);

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
