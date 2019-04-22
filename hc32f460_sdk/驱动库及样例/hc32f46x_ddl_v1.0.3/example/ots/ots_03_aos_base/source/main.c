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

/* Select EVT_AOS_STRG as OTS trigger source. */
#define OTS_TRG_SRC                 EVT_AOS_STRG

/* Enable AOS macro definition. */
#define ENABLE_AOS()                PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS, Enable)

#define SYS_CLOCK_FREQ_MHZ          (SystemCoreClock / 1000000ul)

#define TIMEOUT_10MS                (10u)

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

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static float32_t m_f32Temperature = .0f;

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

    /***************** Configuration end, application start **************/

    /* 1. Make an event to trigger OTS. */
    AOS_SW_Trigger();

    /* 2. Check OTS. */
    OTS_CheckSample(&m_f32Temperature, TIMEOUT_10MS);

    while (1u)
    {
        // TODO: YOUR CODE.
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
    stcSysclkCfg.enPclk1Div = ClkSysclkDiv2;  // Max 84MHz
    stcSysclkCfg.enPclk2Div = ClkSysclkDiv4;  // Max 60MHz
    stcSysclkCfg.enPclk3Div = ClkSysclkDiv4;  // Max 42MHz
    stcSysclkCfg.enPclk4Div = ClkSysclkDiv2;  // Max 84MHz
    CLK_SysClkConfig(&stcSysclkCfg);
}

/**
 *******************************************************************************
 ** \brief  OTS configuration, including initial configuration,
 **         clock configuration and trigger source configuration.
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

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
