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
 ** \brief The example of SPI four wire interrupt tx and rx function
 **
 **   - 2018-11-08  1.0  Yangjp First version for Device Driver Library of SPI.
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

/* SPI_SCK Port/Pin definition */
#define SPI_SCK_PORT                    PortE
#define SPI_SCK_PIN                     Pin00
#define SPI_SCK_FUNC                    Func_Spi3_Sck

/* SPI_NSS Port/Pin definition */
#define SPI_NSS_PORT                    PortE
#define SPI_NSS_PIN                     Pin01
#define SPI_NSS_FUNC                    Func_Spi3_Nss0

/* SPI_MOSI Port/Pin definition */
#define SPI_MOSI_PORT                   PortE
#define SPI_MOSI_PIN                    Pin02
#define SPI_MOSI_FUNC                   Func_Spi3_Mosi

/* SPI_MISO Port/Pin definition */
#define SPI_MISO_PORT                   PortE
#define SPI_MISO_PIN                    Pin03
#define SPI_MISO_FUNC                   Func_Spi3_Miso

/* SPI unit and clock definition */
#define SPI_UNIT                        M4_SPI3
#define SPI_UNIT_CLOCK                  PWC_FCG1_PERIPH_SPI3
#define SPI_TX_INT_SOURCE               INT_SPI3_SRTI
#define SPI_RX_INT_SOURCE               INT_SPI3_SRRI

/* Choose SPI master or slave mode */
#define SPI_MASTER_MODE
//#define SPI_SLAVE_MODE

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t u8ExIntFlag = 0;
static uint8_t u8TxBuffer[] = "SPI Master/Slave example: Communication between two boards using SPI interface!";
static uint8_t u8RxBuffer[128] = {0};
static uint8_t u8TxIndex = 0, u8RxIndex = 0;
static uint8_t u8BufferLen = 0;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
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
        u8ExIntFlag = 1u;
        EXINT_IrqFlgClr(ExtiCh03);
    }
}

/**
 *******************************************************************************
 ** \brief SPI3 send callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SpiTx_IrqCallback(void)
{
    if (u8TxIndex < u8BufferLen)
    {
        SPI_SendData8(SPI_UNIT, u8TxBuffer[u8TxIndex++]);
    }
    else
    {
        SPI_IrqCmd(SPI_UNIT, SpiIrqSend, Disable);
    }

}

/**
 *******************************************************************************
 ** \brief SPI3 receive callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SpiRx_IrqCallback(void)
{
    if (u8RxIndex < u8BufferLen)
    {
        u8RxBuffer[u8RxIndex++] = SPI_ReceiveData8(SPI_UNIT);
    }
    else
    {
        SPI_IrqCmd(SPI_UNIT, SpiIrqReceive, Disable);
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
 ** \brief System clock init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void SystemClk_Init(void)
{
    en_clk_sys_source_t     enSysClkSrc;
    stc_clk_sysclk_cfg_t    stcSysClkCfg;
    stc_clk_xtal_cfg_t      stcXtalCfg;
    stc_clk_mpll_cfg_t      stcMpllCfg;
    stc_clk_freq_t          stcClkFreq;

    MEM_ZERO_STRUCT(enSysClkSrc);
    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv = ClkSysclkDiv1;   // Max 168MHz
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;  // Max 168MHz
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;  // Max 60MHz
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;  // Max 42MHz
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;  // Max 84MHz
    CLK_SysClkConfig(&stcSysClkCfg);

    /* Switch system clock source to MPLL. */
    /* Use Xtal32 as MPLL source. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* MPLL config. */
    stcMpllCfg.pllmDiv = 1;
    stcMpllCfg.plln = 42;
    stcMpllCfg.PllpDiv = 2;
    stcMpllCfg.PllqDiv = 2;
    stcMpllCfg.PllrDiv = 2;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_5);
    EFM_Lock();

    /* Enable MPLL. */
    CLK_MpllCmd(Enable);

    /* Wait MPLL ready. */
    while (Set != CLK_GetFlagStatus(ClkFlagMPLLRdy));

    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);

    /* Check source and frequence. */
    enSysClkSrc = CLK_GetSysClkSource();
    CLK_GetClockFreq(&stcClkFreq);
}

/**
 *******************************************************************************
 ** \brief Configure SPI peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Spi_Config(void)
{
    stc_spi_init_t stcSpiInit;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcSpiInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Configuration peripheral clock */
    PWC_Fcg1PeriphClockCmd(SPI_UNIT_CLOCK, Enable);

    /* Configuration SPI pin */
    PORT_SetFunc(SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_FUNC, Disable);
    PORT_SetFunc(SPI_NSS_PORT, SPI_NSS_PIN, SPI_NSS_FUNC, Disable);
    PORT_SetFunc(SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_FUNC, Disable);
    PORT_SetFunc(SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_FUNC, Disable);

    /* Configuration SPI structure */
    stcSpiInit.enClkDiv = SpiClkDiv64;
    stcSpiInit.enFrameNumber = SpiFrameNumber1;
    stcSpiInit.enDataLength = SpiDataLengthBit8;
    stcSpiInit.enFirstBitPosition = SpiFirstBitPositionMSB;
    stcSpiInit.enSckPolarity = SpiSckIdleLevelLow;
    stcSpiInit.enSckPhase = SpiSckOddSampleEvenChange;
    stcSpiInit.enReadBufferObject = SpiReadReceiverBuffer;
    stcSpiInit.enWorkMode = SpiWorkMode4Line;
    stcSpiInit.enTransMode = SpiTransFullDuplex;
    stcSpiInit.enCommAutoSuspendEn = Disable;
    stcSpiInit.enModeFaultErrorDetectEn = Disable;
    stcSpiInit.enParitySelfDetectEn = Disable;
    stcSpiInit.enParityEn = Disable;
    stcSpiInit.enParity = SpiParityEven;

#ifdef SPI_MASTER_MODE
    stcSpiInit.enMasterSlaveMode = SpiModeMaster;
    stcSpiInit.stcDelayConfig.enSsSetupDelayOption = SpiSsSetupDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsSetupDelayTime = SpiSsSetupDelaySck1;
    stcSpiInit.stcDelayConfig.enSsHoldDelayOption = SpiSsHoldDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsHoldDelayTime = SpiSsHoldDelaySck1;
    stcSpiInit.stcDelayConfig.enSsIntervalTimeOption = SpiSsIntervalCustomValue;
    stcSpiInit.stcDelayConfig.enSsIntervalTime = SpiSsIntervalSck6PlusPck2;
    stcSpiInit.stcSsConfig.enSsValidBit = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity = SpiSsLowValid;
#endif

#ifdef SPI_SLAVE_MODE
    stcSpiInit.enMasterSlaveMode = SpiModeSlave;
    stcSpiInit.stcSsConfig.enSsValidBit = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity = SpiSsLowValid;
#endif
    SPI_Init(SPI_UNIT, &stcSpiInit);

    /* SPI3 tx interrupt */
    stcIrqRegiConf.enIntSrc = SPI_TX_INT_SOURCE;
    stcIrqRegiConf.enIRQn = Int005_IRQn;
    stcIrqRegiConf.pfnCallback = SpiTx_IrqCallback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Enable software trigger interrupt */
    enIntEnable(Int5);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* SPI3 rx interrupt */
    stcIrqRegiConf.enIntSrc = SPI_RX_INT_SOURCE;
    stcIrqRegiConf.enIRQn = Int006_IRQn;
    stcIrqRegiConf.pfnCallback = SpiRx_IrqCallback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Enable SPI */
    SPI_Cmd(SPI_UNIT, Enable);
}

/**
 *******************************************************************************
 ** \brief  main function for four wire SPI interrupt tx and rx function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_port_init_t stcPortInit;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);

    /* Configure system clock frequency */
    SystemClk_Init();

    /* LED0 Port/Pin initialization */
    LED0_OFF();
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* LED1 Port/Pin initialization */
    LED1_OFF();
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);

    /* Key0 Port/Pin initialization */
    Sw2_Init();
    /* Configure SPI */
    Spi_Config();
    /* Get tx buffer length */
    u8BufferLen = sizeof(u8TxBuffer);

    while (1)
    {
        /* Wait key trigger in master mode */
#ifdef SPI_MASTER_MODE
        while (0u == u8ExIntFlag);
        u8ExIntFlag = 0;
#endif
        u8TxIndex = 0;
        u8RxIndex = 0;
        memset(u8RxBuffer, 0, sizeof(u8RxBuffer));
        /* Enable tx and rx interrupt */
        SPI_IrqCmd(SPI_UNIT, SpiIrqReceive, Enable);
        SPI_IrqCmd(SPI_UNIT, SpiIrqSend, Enable);
        /* software trigger send interrupt */
        SWI_Enable(SwiCh05);
        /* Wait end of send and receive */
        while ((u8TxIndex < u8BufferLen) || (u8RxIndex < u8BufferLen));
        /* Disable tx and rx interrupt */
        SPI_IrqCmd(SPI_UNIT, SpiIrqReceive, Disable);
        SPI_IrqCmd(SPI_UNIT, SpiIrqSend, Disable);
        SWI_Disable(SwiCh05);

        /* Compare u8TxBuffer and u8RxBuffer */
        if (memcmp(u8TxBuffer, u8RxBuffer, u8BufferLen) != 0)
        {
            LED0_ON();
            LED1_OFF();
        }
        else
        {
            LED0_OFF();
            LED1_ON();
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
