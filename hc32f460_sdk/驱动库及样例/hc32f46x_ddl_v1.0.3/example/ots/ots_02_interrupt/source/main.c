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

#define SYS_CLOCK_FREQ_MHZ          (SystemCoreClock / 1000000ul)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void OtsConfig(void);
static void OtsClockConfig(void);
static void OtsInitConfig(void);
static void OtsIrqConfig(void);

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
    /* Config OTS. */
    OtsConfig();

    /***************** Configuration end, application start **************/

    /* Start OTS. */
    OTS_StartIT();

    /* Check OTS. */
    while (false == m_bOtsIrqFlag);
    m_bOtsIrqFlag = false;
    (void)m_f32Temperature;

    while (1u)
    {
        // TODO: YOUR CODE
    }
}

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
 /**
 *******************************************************************************
 ** \brief  OTS configuration, including initial configuration and
 **         clock configuration.
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
