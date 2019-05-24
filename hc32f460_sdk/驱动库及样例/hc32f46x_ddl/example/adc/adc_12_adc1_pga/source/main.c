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

/* ADC clock selection definition. */
#define ADC_CLK_PCLK                (1u)
#define ADC_CLK_MPLLQ               (2u)
#define ADC_CLK_UPLLR               (3u)

/* Select MPLLQ as ADC clock. */
#define ADC_CLK                     ADC_CLK_MPLLQ

/* ADC1 channel definition for this example. */
#define ADC1_SA_NORMAL_CHANNEL      ADC1_CH0
#define ADC1_SA_CHANNEL             ADC1_SA_NORMAL_CHANNEL
#define ADC1_SA_CHANNEL_COUNT       (1u)

#define ADC1_SB_NORMAL_CHANNEL      (ADC1_CH2 | ADC1_CH5 | ADC1_CH6)
#define ADC1_SB_CHANNEL             ADC1_SB_NORMAL_CHANNEL
#define ADC1_SB_CHANNEL_COUNT       (3u)

#define ADC1_CHANNEL                (ADC1_SA_CHANNEL | ADC1_SB_CHANNEL)
#define ADC1_PGA_CHANNLE            (PGA_CH0 | PGA_CH2 | PGA_CH5)

/* ADC1 channel sampling time.      ADC1_CH0 */
#define ADC1_SA_CHANNEL_SAMPLE_TIME { 0x30 }

/* ADC1 channel sampling time.      ADC1_CH2   ADC1_CH5   ADC1_CH6 */
#define ADC1_SB_CHANNEL_SAMPLE_TIME { 0x50,     0x60,      0x45 }

/* ADC resolution definitions. */
#define ADC_RESOLUTION_8BIT         (8u)
#define ADC_RESOLUTION_10BIT        (10u)
#define ADC_RESOLUTION_12BIT        (12u)

#define ADC1_RESOLUTION             ADC_RESOLUTION_12BIT
#define ADC2_RESOLUTION             ADC_RESOLUTION_10BIT

/* ADC reference voltage. The voltage of pin VREFH. */
#define ADC_VREF                    (3.3382f)

/* ADC accuracy. */
#define ADC1_ACCURACY               (1ul << ADC1_RESOLUTION)

/* PGA factor definitions. */
/* Choose a part from @ref en_adc_pga_factor_t. */
#define PGA_FACTOR_3P2              (5u)
#define PGA_FACTOR_4P571            (8u)
#define PGA_FACTOR                  PGA_FACTOR_4P571

#if (PGA_FACTOR == PGA_FACTOR_3P2)
#define ADC_PGA_FACTOR              (3.2f)
#elif (PGA_FACTOR == PGA_FACTOR_4P571)
#define ADC_PGA_FACTOR              (4.571f)
#else
#endif

#define TIMEOUT_MS                  (10u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void AdcConfig(void);
static void AdcClockConfig(void);
static void AdcInitConfig(void);
static void AdcChannelConfig(void);
static void AdcTriggerConfig(void);

static void AdcSetChannelPinMode(M4_ADC_TypeDef *ADCx,
                                 uint32_t u32Channel,
                                 en_pin_mode_t enMode);
static void AdcSetPinMode(uint8_t u8AdcPin, en_pin_mode_t enMode);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint16_t m_au16Adc1Value[ADC1_CH_COUNT];

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
    /* Default clock is MRC(8MHz) */

    /* Config ADC. */
    AdcConfig();

    /* Config UART for printing. Baud rate 115200. */
    Ddl_UartInit();

    /***************** Configuration end, application start **************/

    while (1u)
    {
        ADC_PgaCmd(Disable);
        /* Start ADC1. */
        ADC_StartConvert(M4_ADC1);
        /* Check ADC1 sequence A and sequence B. */
        if (Ok == ADC_CheckConvert(M4_ADC1, AdcSequence_A, m_au16Adc1Value, TIMEOUT_MS))
        {
            printf("\nBefore PGA enable.");
            printf("\nADC1_IN0 value %d.", m_au16Adc1Value[0u]);
            printf("\nADC1_IN0 voltage is %.4fV.",
                    ((float)m_au16Adc1Value[0u] * ADC_VREF) / (float)ADC1_ACCURACY);
        }
        if (Ok == ADC_CheckConvert(M4_ADC1, AdcSequence_B, m_au16Adc1Value, TIMEOUT_MS))
        {
            // Do something.
        }

        ADC_PgaCmd(Enable);
        /* Start ADC1. */
        ADC_StartConvert(M4_ADC1);
        /* Check ADC1 sequence A and sequence B. */
        if (Ok == ADC_CheckConvert(M4_ADC1, AdcSequence_A, m_au16Adc1Value, TIMEOUT_MS))
        {
            printf("\nAfter PGA enable and factor is %f.", ADC_PGA_FACTOR);
            printf("\nADC1_IN0 value %d.", m_au16Adc1Value[0u]);
            printf("\nADC1_IN0 voltage is %.4fV.",
                ((float)m_au16Adc1Value[0u] * ADC_VREF) / (float)ADC1_ACCURACY);
        }
        if (Ok == ADC_CheckConvert(M4_ADC1, AdcSequence_B, m_au16Adc1Value, TIMEOUT_MS))
        {
            // Do something.
        }

        /* Main loop cycles is 1000ms. */
        Ddl_Delay1ms(1000u);
    }
}

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  ADC configuration, including clock configuration, initial configuration,
 **         channel configuration and trigger source configuration.
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
}

/**
 *******************************************************************************
 ** \brief  ADC clock configuration.
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
#if (ADC_CLK == ADC_CLK_PCLK)
    stc_clk_sysclk_cfg_t stcSysclkCfg;

    /* Set bus clock division, depends on the system clock frequency. */
    stcSysclkCfg.enHclkDiv  = ClkSysclkDiv1;  // Max 168MHz
    stcSysclkCfg.enExclkDiv = ClkSysclkDiv2;  // Max 84MHz
    stcSysclkCfg.enPclk0Div = ClkSysclkDiv1;  // Max 168MHz
    stcSysclkCfg.enPclk1Div = ClkSysclkDiv2;  // Max 84MHz
    stcSysclkCfg.enPclk2Div = ClkSysclkDiv4;  // Max 60MHz
    stcSysclkCfg.enPclk3Div = ClkSysclkDiv4;  // Max 42MHz
    stcSysclkCfg.enPclk4Div = ClkSysclkDiv1;  // Max 84MHz.
    CLK_SysClkConfig(&stcSysclkCfg);
    CLK_SetPeriClkSource(ClkPeriSrcPclk);

#elif (ADC_CLK == ADC_CLK_MPLLQ)
    stc_clk_xtal_cfg_t stcXtalCfg;
    stc_clk_mpll_cfg_t stcMpllCfg;

    if (CLKSysSrcMPLL == CLK_GetSysClkSource())
    {
        /*
         * Configure MPLLQ(same as MPLLP and MPLLR) when you
         * configure MPLL as the system clock.
         */
    }
    else
    {
        /* Use XTAL as MPLL source. */
        stcXtalCfg.enFastStartup = Enable;
        stcXtalCfg.enMode = ClkXtalModeOsc;
        stcXtalCfg.enDrv  = ClkXtalLowDrv;
        CLK_XtalConfig(&stcXtalCfg);
        CLK_XtalCmd(Enable);

        /* Set MPLL out 240MHz. */
        stcMpllCfg.pllmDiv = 1u;
        /* mpll = 8M / pllmDiv * plln */
        stcMpllCfg.plln    = 30u;
        stcMpllCfg.PllpDiv = 16u;
        stcMpllCfg.PllqDiv = 16u;
        stcMpllCfg.PllrDiv = 16u;
        CLK_SetPllSource(ClkPllSrcXTAL);
        CLK_MpllConfig(&stcMpllCfg);
        CLK_MpllCmd(Enable);
    }
    CLK_SetPeriClkSource(ClkPeriSrcMpllp);

#elif (ADC_CLK == ADC_CLK_UPLLR)
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
    stcUpllCfg.PllrDiv = 16u;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_UpllConfig(&stcUpllCfg);
    CLK_UpllCmd(Enable);
    CLK_SetPeriClkSource(ClkPeriSrcUpllr);
#endif
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

#if (ADC1_RESOLUTION == ADC_RESOLUTION_8BIT)
    stcAdcInit.enResolution = AdcResolution_8Bit;
#elif (ADC1_RESOLUTION == ADC_RESOLUTION_10BIT)
    stcAdcInit.enResolution = AdcResolution_10Bit;
#else
    stcAdcInit.enResolution = AdcResolution_12Bit;
#endif
    stcAdcInit.enDataAlign  = AdcDataAlign_Right;
    stcAdcInit.enAutoClear  = AdcClren_Disable;
    stcAdcInit.enScanMode   = AdcMode_SAOnceSBOnce;

    /* 1. Enable ADC1. */
    ENABLE_ADC1();
    /* 2. Initialize ADC1. */
    ADC_Init(M4_ADC1, &stcAdcInit);
}

/**
 *******************************************************************************
 ** \brief  ADC channel configuration.
 **
 ** \note   1. Only ADC1 supports PGA.
 **         2. The PGA channel maps external pins directly and does not
 **            correspond to the ADC channels. Please ensure that the ADC channel
 **            corresponding to the PGA mapped pin has been added to Sequence A
 **            or Sequence B.
 **
 ******************************************************************************/
static void AdcChannelConfig(void)
{
    stc_adc_ch_cfg_t  stcChCfg;
    stc_adc_pga_cfg_t stcPgaCfg;
    uint8_t au8Adc1SaSampTime[ADC1_SA_CHANNEL_COUNT] = ADC1_SA_CHANNEL_SAMPLE_TIME;
    uint8_t au8Adc1SbSampTime[ADC1_SB_CHANNEL_COUNT] = ADC1_SB_CHANNEL_SAMPLE_TIME;

    MEM_ZERO_STRUCT(stcChCfg);
    MEM_ZERO_STRUCT(stcPgaCfg);

    /**************************** Add ADC1 channels ****************************/
    /* 1. Set the ADC pin to analog mode. */
    AdcSetChannelPinMode(M4_ADC1, ADC1_CHANNEL, Pin_Mode_Ana);

    stcChCfg.u32Channel  = ADC1_SA_CHANNEL;
    stcChCfg.u8Sequence  = AdcSequence_A;
    stcChCfg.pu8SampTime = au8Adc1SaSampTime;
    /* 2. Add ADC channel. */
    ADC_AddAdcChannel(M4_ADC1, &stcChCfg);

    stcChCfg.u32Channel  = ADC1_SB_CHANNEL;
    stcChCfg.u8Sequence  = AdcSequence_B;
    stcChCfg.pu8SampTime = au8Adc1SbSampTime;
    /* 2. Add ADC channel. */
    ADC_AddAdcChannel(M4_ADC1, &stcChCfg);

    /* 3. Config PGA and add PGA channels. */
    stcPgaCfg.enFactor     = (en_adc_pga_factor_t)PGA_FACTOR;
    stcPgaCfg.enNegativeIn = AdcPgaNegative_VSSA;
    ADC_ConfigPga(&stcPgaCfg);

    /* Add PGA pin ADC1_IN0, ADC1_IN2, ADC12_IN5 */
    ADC_AddPgaChannel(ADC1_PGA_CHANNLE);

    /* Enable PGA. */
    ADC_PgaCmd(Enable);
}

/**
 *******************************************************************************
 ** \brief  ADC trigger source configuration.
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

    /* Sequence A will be started by software. */
    ADC_TriggerSrcCmd(M4_ADC1, AdcSequence_A, Disable);

    /* Sequence A scan ends to trigger sequence B. */
    stcTrgCfg.u8Sequence = AdcSequence_B;
    stcTrgCfg.enTrgSel   = AdcTrgsel_TRGX0;
    stcTrgCfg.enInTrg0   = EVT_ADC1_EOCA;
    ADC_ConfigTriggerSrc(M4_ADC1, &stcTrgCfg);
    ADC_TriggerSrcCmd(M4_ADC1, AdcSequence_B, Enable);
}

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

