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
 ** \brief TRNG sample
 **
 **   - 2018-10-20  1.0  Wuze First version for Device Driver Library of
 **     Trng
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
/* TRNG clock selection definition. */
#define TRNG_CLK_PCLK4              (1u)
#define TRNG_CLK_MPLLQ              (2u)
#define TRNG_CLK_UPLLR              (3u)

/* Select MPLLQ as TRNG clock. */
#define TRNG_CLK                    TRNG_CLK_MPLLQ

#define TIMEOUT_10MS                (10u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void SystemClockConfig(void);

static void TrngConfig(void);
static void TrngClockConfig(void);
static void TrngInitConfig(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint32_t m_au32Random[2u];

static stc_clk_sysclk_cfg_t m_stcSysclkCfg =
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
    /* Configuring a new system clock if you need. */
    SystemClockConfig();

    /* Config TRNG. */
    TrngConfig();

    /* Config UART for printing. Baud rate 115200. */
    Ddl_UartInit();

    /***************** Configuration end, application start **************/

    while (1u)
    {
        /* Start TRNG, check TRNG and get random number. */
        TRNG_Generate(m_au32Random, TIMEOUT_10MS);
        printf("\nRand number: 0x%.8x 0x%.8x.", m_au32Random[0u], m_au32Random[1u]);
        Ddl_Delay1ms(100u);
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

    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);

    /* Set bus clock division first. */
    CLK_SysClkConfig(&m_stcSysclkCfg);

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
 ** \brief  TRNG configuration, including clock configuration and
 **         initial configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void TrngConfig(void)
{
    TrngClockConfig();
    TrngInitConfig();
}

/**
 *******************************************************************************
 ** \brief  TRNG clock configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void TrngClockConfig(void)
{
#if (TRNG_CLK == TRNG_CLK_PCLK4)
    /* PCLK4 is TRNG's clock. */
    /* TRNG's clock frequency below 1MHz(inclusive) if possible will be better. */
    m_stcSysclkCfg.enPclk4Div = ClkSysclkDiv64;
    CLK_SysClkConfig(&m_stcSysclkCfg);
    CLK_SetPeriClkSource(ClkPeriSrcPclk);

#elif (TRNG_CLK == TRNG_CLK_MPLLQ)
    stc_clk_xtal_cfg_t stcXtalCfg;
    stc_clk_mpll_cfg_t stcMpllCfg;
    en_clk_sys_source_t enSysClkSrc;

    enSysClkSrc = CLK_GetSysClkSource();
    if (enSysClkSrc == CLKSysSrcMPLL)
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
        stcMpllCfg.PllpDiv = 2u;
        stcMpllCfg.PllqDiv = 16u;
        stcMpllCfg.PllrDiv = 4u;
        CLK_SetPllSource(ClkPllSrcXTAL);
        CLK_MpllConfig(&stcMpllCfg);
        CLK_MpllCmd(Enable);
    }

    CLK_SetPeriClkSource(ClkPeriSrcMpllp);

#elif (TRNG_CLK == TRNG_CLK_UPLLR)
    stc_clk_xtal_cfg_t stcXtalCfg;
    stc_clk_upll_cfg_t stcUpllCfg;

    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcSysclkCfg);

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
    stcUpllCfg.PllpDiv = 2u;
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
 ** \brief  TRNG initial configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void TrngInitConfig(void)
{
    stc_trng_init_t stcTrngInit;

    stcTrngInit.enLoadCtrl   = TrngLoadNewInitValue_Enable;
    stcTrngInit.enShiftCount = TrngShiftCount_64;

    /* 1. Enable TRNG. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_TRNG, Enable);
    /* 2. Initialize TRNG. */
    TRNG_Init(&stcTrngInit);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
