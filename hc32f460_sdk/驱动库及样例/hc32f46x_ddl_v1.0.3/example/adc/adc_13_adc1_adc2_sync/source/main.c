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
 ** \brief ADC sample
 **
 **   - 2018-11-30  1.0  Wuze First version for Device Driver Library of
 **     ADC
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
/*
 * If you remap the mapping between the channel and the pin with the function
 * ADC_ChannleRemap, define ADC_CH_REMAP as non-zero, otherwise define as 0.
 */
#define ADC_CH_REMAP                (0u)

/* The AOS function is used in this example. */
#define ENABLE_AOS()                PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS, Enable)

/* Enable ADC1. */
#define ENABLE_ADC1()               PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1, Enable)

/* Enable ADC2. */
#define ENABLE_ADC2()               PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC2, Enable)

/* Disable ADC1. */
#define DISABLE_ADC1()              PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1, Disable)

/* Disable ADC2. */
#define DISABLE_ADC2()              PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC2, Disable)

/* The number of channels for ADC1 and ADC2 must be the same in sync mode. */
#define ADC_SYNC_CH_COUNT           (1u)

/* Select EVT_AOS_STRG as ADC1 sequence A trigger source. */
#define ADC_SYNC_TRG_EVENT          EVT_AOS_STRG

/*
 * All the channels must set the same sampling time in sync mode.
 * The sampling time depends on your hardware design.
 */
/* ADC1 channel definition for this example. */
#define ADC1_SA_CHANNEL             ADC1_CH4
#define ADC1_SA_CHANNEL_COUNT       ADC_SYNC_CH_COUNT

#define ADC1_CHANNEL                ADC1_SA_CHANNEL

#define ADC1_DMA_SRC_ADDR           ((uint32_t)(&M4_ADC1->DR4))

/* ADC1 channel sampling time.             ADC1_CH4   */
#define ADC1_SA_CHANNEL_SAMPLE_TIME { ADC_SYNC_SAMPLE_TIME }

/* ADC2 channel definition for this example. */
#define ADC2_SA_CHANNEL             ADC2_CH0
#define ADC2_SA_CHANNEL_COUNT       ADC_SYNC_CH_COUNT

#define ADC2_CHANNEL                ADC2_SA_CHANNEL

#define ADC2_DMA_SRC_ADDR           ((uint32_t)(&M4_ADC2->DR0))

/* ADC2 channel sampling time.             ADC2_CH0   */
#define ADC2_SA_CHANNEL_SAMPLE_TIME { ADC_SYNC_SAMPLE_TIME }

/* Synchronous mode selection definition. */
/*
 * SYNC_SINGLE_SERIAL:
 * ADC1 and ADC2 only work once after being triggered.
 * Mode AdcMode_SAOnce and AdcMode_SAContinuous are valid.
 * ADC2 start after ADC1 N PCLK4 cycles.
 */
#define SYNC_SINGLE_SERIAL          (0u)

/*
 * SYNC_SINGLE_PARALLEL:
 * ADC1 and ADC2 only work once after being triggered.
 * Mode AdcMode_SAOnce and AdcMode_SAContinuous are valid.
 * ADC1 and ADC2 start at the same time.
 * ADC1 and ADC2 CAN NOT select the same ADC pin.
 */
#define SYNC_SINGLE_PARALLEL        (2u)

/*
 * SYNC_CONTINUOUS_SERIAL:
 * ADC1 and ADC2 are always working after being triggered.
 * Mode AdcMode_SAOnce is valid.
 * ADC2 start after ADC1 N PCLK4 cycles.
 * ADC1 and ADC2 CAN NOT select the same ADC pin.
 */
#define SYNC_CONTINUOUS_SERIAL      (4u)

/*
 * SYNC_CONTINUOUS_PARALLEL:
 * ADC1 and ADC2 are always working after being triggered.
 * Mode AdcMode_SAOnce is valid.
 * ADC1 and ADC2 start at the same time.
 */
#define SYNC_CONTINUOUS_PARALLEL    (6u)

/* Select sync mode depending on your application. */
#define SYNC_MODE                   SYNC_SINGLE_SERIAL

/* Sequence A scan mode definition. It depends on SYNC_MODE. */
#if ((SYNC_MODE == SYNC_SINGLE_PARALLEL) || (SYNC_MODE == SYNC_SINGLE_SERIAL))
/* AdcMode_SAOnce and AdcMode_SAContinuous can be used. */
//#define SA_SCAN_MODE                AdcMode_SAOnce
#define SA_SCAN_MODE                AdcMode_SAContinuous
#elif ((SYNC_MODE == SYNC_CONTINUOUS_PARALLEL) || (SYNC_MODE == SYNC_CONTINUOUS_SERIAL))
/* Only AdcMode_SAOnce can be used. */
#define SA_SCAN_MODE                AdcMode_SAOnce
#else
#endif

#if (SYNC_MODE == SYNC_SINGLE_SERIAL)
/* ADC sampling time is 11 ADCLK cycles. */
#define ADC_SYNC_SAMPLE_TIME        (11u)

/*
 * ADC_SYNC_DELAY_TIME definition depends on ADCLK, trigger source, etc.
 * See section 17.3.8 and 17.4.16 of user manual for details.
 */
#define ADC_SYNC_DELAY_TIME         (12u)

#elif (SYNC_MODE == SYNC_SINGLE_PARALLEL)
#define ADC_SYNC_SAMPLE_TIME        (11u)
#define ADC_SYNC_DELAY_TIME         (11u)

#elif (SYNC_MODE == SYNC_CONTINUOUS_SERIAL)
#define ADC_SYNC_SAMPLE_TIME        (11u)
#define ADC_SYNC_DELAY_TIME         (17u)

#elif (SYNC_MODE == SYNC_CONTINUOUS_PARALLEL)
#define ADC_SYNC_SAMPLE_TIME        (11u)
#define ADC_SYNC_DELAY_TIME         (40u)

#else
#endif // (SYNC_MODE == SYNC_SINGLE_SERIAL)

/* DMA definition for ADC1. */
#define ADC1_SA_DMA_UNIT            M4_DMA2
#define ADC1_SA_DMA_PWC             PWC_FCG0_PERIPH_DMA2
#define ADC1_SA_DMA_CH              DmaCh3
#define ADC1_SA_DMA_TRGSRC          EVT_ADC1_EOCA

/* DMA definition for ADC2. */
#define ADC2_SA_DMA_UNIT            M4_DMA1
#define ADC2_SA_DMA_PWC             PWC_FCG0_PERIPH_DMA1
#define ADC2_SA_DMA_CH              DmaCh2
#define ADC2_SA_DMA_TRGSRC          EVT_ADC2_EOCA

#define ADC_BUFFER_LENGTH           (512u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void SystemClockConfig(void);

static void AdcConfig(void);
static void AdcClockConfig(void);
static void AdcInitConfig(void);
static void AdcChannelConfig(void);
static void AdcTriggerConfig(void);
static void AdcSyncConfig(void);

static void AdcDmaConfig(void);

static void IndicatePinConfig(void);

static void AdcSetChannelPinMode(M4_ADC_TypeDef *ADCx,
                                 uint32_t u32Channel,
                                 en_pin_mode_t enMode);
static void AdcSetPinMode(uint8_t u8AdcPin, en_pin_mode_t enMode);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint16_t m_au16Adc1SaValue[ADC_BUFFER_LENGTH];
static uint16_t m_au16Adc2SaValue[ADC_BUFFER_LENGTH];

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
    /* Config a new system clock. */
    SystemClockConfig();

    /* Config ADCs. */
    AdcConfig();

    /* 
     * Config indicate pins.
     * Its purpose is simply to indicate the sampling rate.
     */
    IndicatePinConfig();

    /***************** Configuration end, application start **************/
    AOS_SW_Trigger();

    while (1u)
    {
        // Your application code.
    }
}

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Configuring a new system clock.
 **         System clock frequency: 168MHz.
 **         System clock source:    MPLL.
 **         MPLL clock source:      XTAL(8MHz).
 **
 ******************************************************************************/
static void SystemClockConfig(void)
{
    stc_clk_xtal_cfg_t stcXtalCfg;
    stc_clk_mpll_cfg_t stcMpllCfg;
    stc_clk_sysclk_cfg_t stcSysclkCfg =
    {
        /* Default system clock division. */
        .enHclkDiv  = ClkSysclkDiv1,  // Max 168MHz
        .enExclkDiv = ClkSysclkDiv2,  // Max 84MHz
        .enPclk0Div = ClkSysclkDiv1,  // Max 168MHz
        .enPclk1Div = ClkSysclkDiv2,  // Max 84MHz
        .enPclk2Div = ClkSysclkDiv4,  // Max 60MHz
        .enPclk3Div = ClkSysclkDiv4,  // Max 42MHz
        .enPclk4Div = ClkSysclkDiv2,  // Max 84MHz
    };

    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);

    /* Set bus clock division first. */
    CLK_SysClkConfig(&stcSysclkCfg);

    /* Switch system clock source to MPLL. */
    /* Use XTAL as MPLL source. */
    stcXtalCfg.enFastStartup = Enable;
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv  = ClkXtalLowDrv;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* Set MPLL out 168MHz. */
    stcMpllCfg.pllmDiv = 1u;
    /* sysclk = 8M / pllmDiv * plln / PllpDiv */
    stcMpllCfg.plln    = 42u;
    stcMpllCfg.PllpDiv = 2u;
    stcMpllCfg.PllqDiv = 16u;
    stcMpllCfg.PllrDiv = 16u;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);

    /* Flash read wait cycle setting. */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_5);
    EFM_Lock();

    CLK_MpllCmd(Enable);

    /* Wait MPLL ready. */
    while(Set != CLK_GetFlagStatus(ClkFlagMPLLRdy));

    /* Set system clock source. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);
}

/**
 *******************************************************************************
 ** \brief  ADC configuration, including clock configuration, initial configuration
 **         and channel configuration, trigger source configuration, synchronous
 **         mode configuration and DMA configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void AdcConfig(void)
{
    AdcClockConfig();
    AdcInitConfig();
    AdcChannelConfig();
    AdcTriggerConfig();
    AdcSyncConfig();
    AdcDmaConfig();
}

/**
 *******************************************************************************
 ** \brief  ADC clock configuration. Configure UPLLR as ADC clock.
 **
 ** \note   1) ADCLK max frequency is 60MHz.
 **         2) If PCLK2 and PCLK4 are selected as the ADC clock,
 **            the following conditions must be met:
 **            a. ADCLK(PCLK2) max 60MHz;
 **            b. PCLK4 : ADCLK = 1:1, 2:1, 4:1, 8:1, 1:2, 1:4
 **
 ******************************************************************************/
static void AdcClockConfig(void)
{
    stc_clk_xtal_cfg_t stcXtalCfg;
    stc_clk_upll_cfg_t stcUpllCfg;

    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcUpllCfg);

    /* Use XTAL as UPLL source. */
    stcXtalCfg.enFastStartup = Enable;
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv  = ClkXtalLowDrv;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* Set UPLL out 240MHz. */
    stcUpllCfg.pllmDiv = 2u;
    /* upll = 8M(XTAL) / pllmDiv * plln */
    stcUpllCfg.plln    = 60u;
    stcUpllCfg.PllpDiv = 16u;
    stcUpllCfg.PllqDiv = 16u;
    stcUpllCfg.PllrDiv = 4u;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_UpllConfig(&stcUpllCfg);
    CLK_UpllCmd(Enable);
    CLK_SetPeriClkSource(ClkPeriSrcUpllr);
}

/**
 *******************************************************************************
 ** \brief  ADC initial configuration.
 **
 ******************************************************************************/
static void AdcInitConfig(void)
{
    stc_adc_init_t stcAdcInit;

    MEM_ZERO_STRUCT(stcAdcInit);

    /* ADC1 and ADC2 use the same configuration in sync mode. */
    stcAdcInit.enResolution = AdcResolution_12Bit;
    stcAdcInit.enDataAlign  = AdcDataAlign_Right;
    stcAdcInit.enAutoClear  = AdcClren_Disable;
    stcAdcInit.enScanMode   = SA_SCAN_MODE;
    /* 1. Enable ADC1. */
    ENABLE_ADC1();
    /* 2. Initialize ADC1. */
    ADC_Init(M4_ADC1, &stcAdcInit);

    /* 1. Enable ADC2. */
    ENABLE_ADC2();
    /* 2. Initialize ADC2. */
    ADC_Init(M4_ADC2, &stcAdcInit);
}

/**
 *******************************************************************************
 ** \brief  ADC channel configuration.
 **
 ** \note   Synchronous mode DO NOT support sequence B.
 **
 ******************************************************************************/
static void AdcChannelConfig(void)
{
    stc_adc_ch_cfg_t stcChCfg;
    uint8_t au8Adc1SaSampTime[ADC1_SA_CHANNEL_COUNT] = ADC1_SA_CHANNEL_SAMPLE_TIME;
    uint8_t au8Adc2SaSampTime[ADC2_SA_CHANNEL_COUNT] = ADC2_SA_CHANNEL_SAMPLE_TIME;

    MEM_ZERO_STRUCT(stcChCfg);

    stcChCfg.u32Channel  = ADC1_SA_CHANNEL;
    stcChCfg.u8Sequence  = AdcSequence_A;
    stcChCfg.pu8SampTime = au8Adc1SaSampTime;
    /* 1. Set the ADC pin to analog mode. */
    AdcSetChannelPinMode(M4_ADC1, ADC1_CHANNEL, Pin_Mode_Ana);
    /* 2. Add ADC channel. */
    ADC_AddAdcChannel(M4_ADC1, &stcChCfg);

    stcChCfg.u32Channel  = ADC2_SA_CHANNEL;
    stcChCfg.pu8SampTime = au8Adc2SaSampTime;
    /* 1. Set the ADC pin to analog mode. */
    /* Not need any more. ADC2 selects the same analog input with ADC1. */
    //AdcSetChannelPinMode(M4_ADC2, ADC2_CHANNEL, Pin_Mode_Ana);
    /* 2. Add ADC channel. */
    ADC_AddAdcChannel(M4_ADC2, &stcChCfg);
}

/**
 *******************************************************************************
 ** \brief  ADC trigger source configuration.
 **
 ** \note  1) Software startup is invalid in synchronous mode.
 **        2) Only ADC1's trigger source is valid.
 **
 ******************************************************************************/
static void AdcTriggerConfig(void)
{
    stc_adc_trg_cfg_t stcTrgCfg;

    MEM_ZERO_STRUCT(stcTrgCfg);
    /*
     * If select an event(@ref en_event_src_t) to trigger ADC,
     * AOS must be enabled first.
     */
    ENABLE_AOS();

    /* Select EVT_AOS_STRG as ADC1 sequence A trigger source. */
    stcTrgCfg.u8Sequence = AdcSequence_A;
    stcTrgCfg.enTrgSel   = AdcTrgsel_TRGX0;
    stcTrgCfg.enInTrg0   = ADC_SYNC_TRG_EVENT;

    ADC_ConfigTriggerSrc(M4_ADC1, &stcTrgCfg);
    ADC_TriggerSrcCmd(M4_ADC1, AdcSequence_A, Enable);
}

/**
 *******************************************************************************
 ** \brief  ADC synchronous mode configuration.
 **
 ******************************************************************************/
static void AdcSyncConfig(void)
{
    stc_adc_sync_cfg_t stcSync;

    MEM_ZERO_STRUCT(stcSync);

    stcSync.enMode     = (en_adc_sync_mode_t)SYNC_MODE;
    stcSync.u8TrgDelay = ADC_SYNC_DELAY_TIME;

    ADC_ConfigSync(&stcSync);
    ADC_SyncCmd(Enable);
}

/**
 *******************************************************************************
 ** \brief  DMA configuration for ADC1 and ADC2.
 **
 ******************************************************************************/
static void AdcDmaConfig(void)
{
    stc_dma_config_t stcDmaCfg;

    MEM_ZERO_STRUCT(stcDmaCfg);

    stcDmaCfg.u16BlockSize   = 1u;
    stcDmaCfg.u16TransferCnt = 0u;
    stcDmaCfg.u32SrcAddr     = ADC1_DMA_SRC_ADDR;
    stcDmaCfg.u32DesAddr     = (uint32_t)(&m_au16Adc1SaValue[0u]);
    stcDmaCfg.u16DesRptSize  = ADC_BUFFER_LENGTH;
    //stcDmaCfg.u16SrcRptSize  = 1u;
    stcDmaCfg.u32DmaLlp      = 0u;
    stcDmaCfg.stcSrcNseqCfg.u16Cnt    = 0;
    stcDmaCfg.stcSrcNseqCfg.u32Offset = 0;
    stcDmaCfg.stcDesNseqCfg.u16Cnt    = 0;
    stcDmaCfg.stcDesNseqCfg.u32Offset = 0;
    stcDmaCfg.stcDmaChCfg.enSrcInc    = AddressFix;
    stcDmaCfg.stcDmaChCfg.enDesInc    = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enSrcRptEn  = Disable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn  = Enable;
    stcDmaCfg.stcDmaChCfg.enSrcNseqEn = Disable;
    stcDmaCfg.stcDmaChCfg.enDesNseqEn = Disable;
    stcDmaCfg.stcDmaChCfg.enTrnWidth  = Dma16Bit;
    stcDmaCfg.stcDmaChCfg.enLlpEn     = Disable;
    stcDmaCfg.stcDmaChCfg.enIntEn     = Disable;

    PWC_Fcg0PeriphClockCmd(ADC1_SA_DMA_PWC, Enable);
    DMA_InitChannel(ADC1_SA_DMA_UNIT, ADC1_SA_DMA_CH, &stcDmaCfg);
    DMA_Cmd(ADC1_SA_DMA_UNIT, Enable);
    DMA_ChannelCmd(ADC1_SA_DMA_UNIT, ADC1_SA_DMA_CH, Enable);
    DMA_ClearIrqFlag(ADC1_SA_DMA_UNIT, ADC1_SA_DMA_CH, TrnCpltIrq);
    /* AOS must be enabled to use DMA */
    /* AOS enabled at first. */
    /* If you have enabled AOS before, then the following statement is not needed. */
    ENABLE_AOS();
    DMA_SetTriggerSrc(ADC1_SA_DMA_UNIT, ADC1_SA_DMA_CH, ADC1_SA_DMA_TRGSRC);

    stcDmaCfg.u16BlockSize  = 1u;
    stcDmaCfg.u32SrcAddr    = ADC2_DMA_SRC_ADDR;
    stcDmaCfg.u32DesAddr    = (uint32_t)(&m_au16Adc2SaValue[0u]);
    stcDmaCfg.u16DesRptSize = ADC_BUFFER_LENGTH;
    //stcDmaCfg.u16SrcRptSize = 1u;
    PWC_Fcg0PeriphClockCmd(ADC2_SA_DMA_PWC, Enable);
    DMA_InitChannel(ADC2_SA_DMA_UNIT, ADC2_SA_DMA_CH, &stcDmaCfg);
    DMA_Cmd(ADC2_SA_DMA_UNIT, Enable);
    DMA_ChannelCmd(ADC2_SA_DMA_UNIT, ADC2_SA_DMA_CH, Enable);
    DMA_ClearIrqFlag(ADC2_SA_DMA_UNIT, ADC2_SA_DMA_CH, TrnCpltIrq);
    DMA_SetTriggerSrc(ADC2_SA_DMA_UNIT, ADC2_SA_DMA_CH, ADC2_SA_DMA_TRGSRC);
}

/**
 *******************************************************************************
 ** \brief  Indicate pins configuration.
 **
 ******************************************************************************/
static void IndicatePinConfig(void)
{
    stc_event_port_init_t stcEPConfig;

    /* Set PB5 as event port 2.5. */
    PORT_SetFunc(PortB, Pin05, Func_Evnpt, Disable);

    /* Set PD8 as event port 4.8. */
    PORT_SetFunc(PortD, Pin08, Func_Evnpt, Disable);

    /* Set event port 2.5 and 4.8 as output function. */
    MEM_ZERO_STRUCT(stcEPConfig);
    stcEPConfig.enDirection = EventPortOut;
    stcEPConfig.enSet       = Enable;
    stcEPConfig.enReset     = Enable;

    EVENTPORT_Init(EventPort2, EventPin05, &stcEPConfig);
    EVENTPORT_Init(EventPort4, EventPin08, &stcEPConfig);

    /* Set EVT_ADC1_EOCA as the trigger source for event port 2. */
    EVENTPORT_SetTriggerSrc(EventPort2, EVT_ADC1_EOCA);

    /* Set EVT_ADC2_EOCA as the trigger source for event port 4. */
    EVENTPORT_SetTriggerSrc(EventPort4, EVT_ADC2_EOCA);
}

/**
 *******************************************************************************
 ** \brief IRQ callbacks.
 **
 ******************************************************************************/
#if 0
void ADC1A_IrqHandler(void)
{
#if 0
    if (Set == ADC_GetConvFlag(M4_ADC1, AdcSequence_A))
    {
        ADC_GetData(M4_ADC1, m_au16Adc1Value);
        ADC_ClrConvFlag(M4_ADC1, AdcSequence_A);
        m_u32AdcIrqFlag |= ADC1_SA_IRQ_BIT;
    }
#else
    INDICATE_HI();
    INDICATE_LO();
#endif
}

void ADC2A_IrqHandler(void)
{
#if 0
    if (Set == ADC_GetConvFlag(M4_ADC2, AdcSequence_A))
    {
        ADC_GetData(M4_ADC2, m_au16Adc2Value);
        ADC_ClrConvFlag(M4_ADC2, AdcSequence_A);
        m_u32AdcIrqFlag |= ADC2_SA_IRQ_BIT;
    }
#else
    INDICATE_HI();
    INDICATE_LO();
#endif
}
#endif

/**
 *******************************************************************************
 ** \brief  Config the pin which is mapping the channel to analog or digit mode.
 **
 ******************************************************************************/
static void AdcSetChannelPinMode(M4_ADC_TypeDef *ADCx,
                                 uint32_t u32Channel,
                                 en_pin_mode_t enMode)
{
    uint8_t u8ChIndex;
#if (ADC_CH_REMAP)
    uint8_t u8AdcPin;
#else
    uint8_t u8ChOffset = 0u;
#endif

    if ((NULL == ADCx) || (0u == u32Channel))
    {
        return;
    }

    if (M4_ADC1 == ADCx)
    {
        u32Channel &= ADC1_PIN_MASK_ALL;
    }
    else
    {
        u32Channel &= ADC2_PIN_MASK_ALL;
#if (!ADC_CH_REMAP)
        u8ChOffset = 4u;
#endif
    }

    u8ChIndex = 0u;
    while (0u != u32Channel)
    {
        if (u32Channel & 0x1ul)
        {
#if (ADC_CH_REMAP)
            u8AdcPin = ADC_GetChannelPinNum(ADCx, u8ChIndex);
            AdcSetPinMode(u8AdcPin, enMode);
#else
            AdcSetPinMode((u8ChIndex + u8ChOffset), enMode);
#endif
        }

        u32Channel >>= 1u;
        u8ChIndex++;
    }
}

/**
 *******************************************************************************
 ** \brief  Set an ADC pin as analog input mode or digit mode.
 **
 ******************************************************************************/
static void AdcSetPinMode(uint8_t u8AdcPin, en_pin_mode_t enMode)
{
    en_port_t enPort;
    en_pin_t enPin;
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = enMode;
    stcPortInit.enPullUp = Disable;

    switch (u8AdcPin)
    {
    case ADC1_IN0:
        enPort = PortA;
        enPin  = Pin00;
        break;

    case ADC1_IN1:
        enPort = PortA;
        enPin  = Pin01;
        break;

    case ADC1_IN2:
        enPort = PortA;
        enPin  = Pin02;
        break;

    case ADC1_IN3:
        enPort = PortA;
        enPin  = Pin03;
        break;

    case ADC12_IN4:
        enPort = PortA;
        enPin  = Pin04;
        break;

    case ADC12_IN5:
        enPort = PortA;
        enPin  = Pin05;
        break;

    case ADC12_IN6:
        enPort = PortA;
        enPin  = Pin06;
        break;

    case ADC12_IN7:
        enPort = PortA;
        enPin  = Pin07;
        break;

    case ADC12_IN8:
        enPort = PortB;
        enPin  = Pin00;
        break;

    case ADC12_IN9:
        enPort = PortB;
        enPin  = Pin01;
        break;

    case ADC12_IN10:
        enPort = PortC;
        enPin  = Pin00;
        break;

    case ADC12_IN11:
        enPort = PortC;
        enPin  = Pin01;
        break;

    case ADC1_IN12:
        enPort = PortC;
        enPin  = Pin02;
        break;

    case ADC1_IN13:
        enPort = PortC;
        enPin  = Pin03;
        break;

    case ADC1_IN14:
        enPort = PortC;
        enPin  = Pin04;
        break;

    case ADC1_IN15:
        enPort = PortC;
        enPin  = Pin05;
        break;

    default:
        return;
    }

    PORT_Init(enPort, enPin, &stcPortInit);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
