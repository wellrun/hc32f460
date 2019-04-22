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
/* OTS clock selection. */
#define OTS_CLK_SEL_XTAL            (0u)
#define OTS_CLK_SEL_HRC             (1u)

/* Select XTAL as OTS clock. */
#define OTS_CLK_SEL                 OTS_CLK_SEL_HRC

/* Timer definition for this example. */
#define TMR_UNIT                    M4_TMR02
#define ENABLE_TMR0()               PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02, Enable)

/* Select EVT_TMR02_GCMA as OTS trigger source. */
#define OTS_TRG_SRC                 EVT_TMR02_GCMA

/* Enable AOS macro definition */
#define ENABLE_AOS()                PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS, Enable)

#define SYS_CLOCK_FREQ_MHZ          (SystemCoreClock / 1000000ul)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void SystemClockConfig(void);

static void OtsConfig(void);
static void OtsInitConfig(void);
static void OtsClockConfig(void);
static void OtsTriggerSrcConfig(void);
static void OtsIrqConfig(void);

static void TimerConfig(void);

static void OtsIrqCallback(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static float32_t m_f32Temperature = .0f;
static bool m_bOtsIrqFlag = false;

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
    /* Config a new system clock if you need. */
    SystemClockConfig();

    /* Config OTS. */
    OtsConfig();

    /* Config timer. */
    TimerConfig();

    /***************** Configuration end, application start **************/

    while (1u)
    {
        /* Event EVT_TMR02_GCMA generates every second. */
        /* Check OTS. */
        if (true == m_bOtsIrqFlag)
        {
            m_bOtsIrqFlag = false;
            // Do something.
        }
        (void)m_f32Temperature;
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
    stc_clk_sysclk_cfg_t stcSysclkCfg;
    stc_clk_freq_t stcClkFreq;

    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);
    MEM_ZERO_STRUCT(stcClkFreq);
    MEM_ZERO_STRUCT(stcSysclkCfg);

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

    /* Set bus clock division. */
    stcSysclkCfg.enHclkDiv  = ClkSysclkDiv1;  // Max 168MHz
    stcSysclkCfg.enExclkDiv = ClkSysclkDiv2;  // Max 84MHz
    stcSysclkCfg.enPclk0Div = ClkSysclkDiv1;  // Max 168MHz
    /* PCLK1 for timer0 in this application. */
    stcSysclkCfg.enPclk1Div = ClkSysclkDiv8;  // Max 84MHz
    stcSysclkCfg.enPclk2Div = ClkSysclkDiv4;  // Max 60MHz
    stcSysclkCfg.enPclk3Div = ClkSysclkDiv4;  // Max 42MHz
    stcSysclkCfg.enPclk4Div = ClkSysclkDiv2;  // Max 84MHz
    CLK_SysClkConfig(&stcSysclkCfg);
}

/**
 *******************************************************************************
 ** \brief  OTS configuration, including initial configuration,
 **         clock configuration, trigger source configuration and
 **         interrupt configuration.
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
    OtsTriggerSrcConfig();
    OtsIrqConfig();
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

    stcOtsInit.enAutoOff = OtsAutoOff_Enable;
#if (OTS_CLK_SEL == OTS_CLK_SEL_HRC)
    stcOtsInit.enClkSel = OtsClkSel_Hrc;
#else
    stcOtsInit.enClkSel = OtsClkSel_Xtal;
#endif
    stcOtsInit.u8ClkFreq = SYS_CLOCK_FREQ_MHZ;

    /* 1. Enable OTS. */
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_OTS, Enable);
    /* 2. Initialize OTS. */
    OTS_Init(&stcOtsInit);

    /* Enable OTS interrupt. */
    OTS_ITCmd(Enable);
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
 ** \brief  OTS trigger source configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void OtsTriggerSrcConfig(void)
{
    /* 1. Enable AOS. */
    ENABLE_AOS();
    /* 2. Set OTS trigger source. */
    OTS_SetTriggerSrc(OTS_TRG_SRC);
}

/**
 *******************************************************************************
 ** \brief  OTS interrupt configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void OtsIrqConfig(void)
{
    stc_irq_regi_conf_t stcOtsIrqCfg;
    en_result_t         enIrqRegResult;

    stcOtsIrqCfg.enIntSrc    = INT_OTS;
    /* stcOtsIrqCfg.enIRQn: [Int000_IRQn, Int031_IRQn] [Int110_IRQn, Int115_IRQn] */
    stcOtsIrqCfg.enIRQn      = Int113_IRQn;
    stcOtsIrqCfg.pfnCallback = OtsIrqCallback;
    enIrqRegResult           = enIrqRegistration(&stcOtsIrqCfg);

    if (Ok == enIrqRegResult)
    {
        NVIC_ClearPendingIRQ(stcOtsIrqCfg.enIRQn);
        NVIC_SetPriority(stcOtsIrqCfg.enIRQn, DDL_IRQ_PRIORITY_03);
        NVIC_EnableIRQ(stcOtsIrqCfg.enIRQn);
    }
}

/**
 *******************************************************************************
 ** \brief  Timer configuration, for generating event EVT_TMR02_GCMA.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void TimerConfig(void)
{
    stc_tim0_base_init_t stcTimerCfg;
    stc_clk_freq_t stcClkTmp;
    uint32_t u32Pclk1;

    MEM_ZERO_STRUCT(stcTimerCfg);
    /* Get PCLK1. */
    CLK_GetClockFreq(&stcClkTmp);
    u32Pclk1 = stcClkTmp.pclk1Freq;

    /* Timer0 peripheral enable. */
    ENABLE_TMR0();
    /* Config register for channel A. */
    stcTimerCfg.Tim0_CounterMode = Tim0_Sync;
    stcTimerCfg.Tim0_SyncClockSource = Tim0_Pclk1;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv1024;
    /* Tim0_CmpValue's type is uint16_t!!! Be careful!!! */
    stcTimerCfg.Tim0_CmpValue = u32Pclk1 / 1024 - 1;
    TIMER0_BaseInit(TMR_UNIT, Tim0_ChannelA, &stcTimerCfg);

    /* Start timer0. */
    TIMER0_Cmd(TMR_UNIT, Tim0_ChannelA, Enable);
}

/**
 *******************************************************************************
 ** \brief  OTS interrupt callback function.
 **         Its main function is to get temperature value.
 **
 ******************************************************************************/
static void OtsIrqCallback(void)
{
    m_f32Temperature = OTS_GetTempIT();
    m_bOtsIrqFlag    = true;
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
