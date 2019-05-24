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
 ** \brief This example demonstrates how to use the OCCR buffer function &&
 **         the through mode function of Timer4Pwm.
 **
 **   - 2018-10-30  1.0  Hongjh First version for Device Driver Library of
 **                      Timer4Pwm
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
/* Timer4 CNT */
#define TIMER4_UNIT                     M4_TMR41
#define TIMER4_CNT_CYCLE_VAL            (40000u)       /* Timer4 counter cycle value */

/* Timer4 OCO */
#define TIMER4_OCO_HIGH_CH              Timer4OcoOuh   /* only Timer4OcoOuh  Timer4OcoOvh  Timer4OcoOwh */

/* Define port and pin for Timer4Pwm */
#define TIMER4_PWM_H_PORT               PortE          /* TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13 */
#define TIMER4_PWM_H_PIN                Pin09

/* Parameter validity check for oco channel */
#define IS_VALID_OCO_CH(x)                                                     \
(   (Timer4OcoOuh == (x))               ||                                     \
    (Timer4OcoOul == (x))               ||                                     \
    (Timer4OcoOvh == (x))               ||                                     \
    (Timer4OcoOvl == (x))               ||                                     \
    (Timer4OcoOwh == (x))               ||                                     \
    (Timer4OcoOwl == (x)))

/* Parameter validity check for timer4 unit */
#define IS_VALID_TIMER4(__TMRx__)                                              \
(   (M4_TMR41 == (__TMRx__))            ||                                     \
    (M4_TMR42 == (__TMRx__))            ||                                     \
    (M4_TMR43 == (__TMRx__)))

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void Timer4CntIrqCallback(void);
static en_int_src_t GetTimer4CntZeroIntNum(M4_TMR4_TypeDef *TMR4x);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint16_t m_u16OccrValIdx = 0;
static uint16_t m_au16OccrVal[] = {1 * (TIMER4_CNT_CYCLE_VAL / 4),
                                   2 * (TIMER4_CNT_CYCLE_VAL / 4),
                                   3 * (TIMER4_CNT_CYCLE_VAL / 4)};

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief Timer4-CNT Zero match interrupt handler
 **
 ******************************************************************************/
static void Timer4CntIrqCallback(void)
{
    TIMER4_CNT_ClearIrqFlag(TIMER4_UNIT, Timer4CntZeroMatchInt);
    TIMER4_OCO_WriteOccr(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, m_au16OccrVal[m_u16OccrValIdx++]);

    if (m_u16OccrValIdx >= ARRAY_SZ(m_au16OccrVal))
    {
        m_u16OccrValIdx = 0;
    }
    else
    {
    }
}

/**
 *******************************************************************************
 ** \brief Get Timer4 CNT zero interrupt number.
 **
 ** \param [in] TMR4x                   Pointer to Timer4 instance register base
 ** \arg M4_TMR41                       Timer4 unit 1 instance register base
 ** \arg M4_TMR42                       Timer4 unit 2 instance register base
 ** \arg M4_TMR43                       Timer4 unit 3 instance register base
 **
 ** \retval                             Timer4 CNT zero interrupt number
 **
 ******************************************************************************/
static en_int_src_t GetTimer4CntZeroIntNum(M4_TMR4_TypeDef *TMR4x)
{
    en_int_src_t enCntZeroIntNum;

    DDL_ASSERT(IS_VALID_TIMER4(TMR4x));

    if (M4_TMR41 == TMR4x)
    {
        enCntZeroIntNum = INT_TMR41_GUDF;
    }
    else if (M4_TMR42 == TMR4x)
    {
        enCntZeroIntNum = INT_TMR42_GUDF;
    }
    else if (M4_TMR43 == TMR4x)
    {
        enCntZeroIntNum = INT_TMR43_GUDF;
    }
    else
    {
    }

    return enCntZeroIntNum;
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
    en_timer4_pwm_ch_t enPwmCh;
    stc_timer4_cnt_init_t stcCntInit;
    stc_timer4_oco_init_t stcOcoInit;
    stc_timer4_pwm_init_t stcPwmInit;
    stc_irq_regi_conf_t stcIrqRegiCfg;
    stc_oco_high_ch_compare_mode_t stcHighChCmpMode;

    /* Clear structures */
    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(stcOcoInit);
    MEM_ZERO_STRUCT(stcPwmInit);
    MEM_ZERO_STRUCT(stcIrqRegiCfg);
    MEM_ZERO_STRUCT(stcHighChCmpMode);

    /* Enable peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM41 | PWC_FCG2_PERIPH_TIM42 | PWC_FCG2_PERIPH_TIM43, Enable);

    /* Timer4 CNT : Initialize CNT configuration structure */
    stcCntInit.enBufferCmd = Disable;
    stcCntInit.enClk = Timer4CntPclk;
    stcCntInit.enClkDiv = Timer4CntPclkDiv16;  /* CNT clock divide */
    stcCntInit.u16Cycle = TIMER4_CNT_CYCLE_VAL;
    stcCntInit.enCntMode = Timer4CntSawtoothWave;
    stcCntInit.enZeroIntCmd = Enable;
    stcCntInit.enPeakIntCmd = Disable;
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;
    TIMER4_CNT_Init(TIMER4_UNIT, &stcCntInit); /* Initialize CNT */

    /* Set Timer4 OCO IRQ */
    MEM_ZERO_STRUCT(stcIrqRegiCfg);
    stcIrqRegiCfg.enIRQn = Int000_IRQn;
    stcIrqRegiCfg.pfnCallback = Timer4CntIrqCallback;
    stcIrqRegiCfg.enIntSrc = GetTimer4CntZeroIntNum(TIMER4_UNIT);
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Timer4 OCO : Initialize OCO configuration structure */
    stcOcoInit.enOccrBufMode = OccrBufTrsfByCntPeak;
    stcOcoInit.enOcmrBufMode = OcmrBufDisable;
    stcOcoInit.enPortLevel = OcPortLevelLow;
    stcOcoInit.enOcoIntCmd = Disable;
    TIMER4_OCO_Init(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, &stcOcoInit); /* Initialize OCO high channel */

    if ((Timer4OcoOuh == TIMER4_OCO_HIGH_CH) || (Timer4OcoOvh == TIMER4_OCO_HIGH_CH) || (Timer4OcoOwh == TIMER4_OCO_HIGH_CH))
    {
        /* ocmr[15:0] = 0x030F */
        stcHighChCmpMode.enCntZeroMatchOpState = OcoOpOutputHold;
        stcHighChCmpMode.enCntZeroNotMatchOpState = OcoOpOutputHold;
        stcHighChCmpMode.enCntUpCntMatchOpState = OcoOpOutputReverse;
        stcHighChCmpMode.enCntPeakMatchOpState = OcoOpOutputHold;
        stcHighChCmpMode.enCntPeakNotMatchOpState = OcoOpOutputHold;
        stcHighChCmpMode.enCntDownCntMatchOpState = OcoOpOutputHold;

        stcHighChCmpMode.enCntZeroMatchOcfState = OcoOcfSet;
        stcHighChCmpMode.enCntUpCntMatchOcfState = OcoOcfSet;
        stcHighChCmpMode.enCntPeakMatchOcfState = OcoOcfSet;
        stcHighChCmpMode.enCntDownCntMatchOcfState = OcoOcfSet;

        stcHighChCmpMode.enMatchConditionExtendCmd = Disable;

        TIMER4_OCO_SetHighChCompareMode(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, &stcHighChCmpMode);  /* Set OCO high channel compare mode */
    }
    else
    {
    }

    /* Set OCO compare value */
    TIMER4_OCO_WriteOccr(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, m_au16OccrVal[m_u16OccrValIdx++]);

    /* Enable OCO */
    TIMER4_OCO_OutputCompareCmd(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, Enable);

    /* Timer4 PWM: Get pwm couple channel */
    if (Timer4OcoOuh == TIMER4_OCO_HIGH_CH)
    {
        enPwmCh = Timer4PwmU;
    }
    else if (Timer4OcoOvh == TIMER4_OCO_HIGH_CH)
    {
        enPwmCh = Timer4PwmV;
    }
    else if (Timer4OcoOwh == TIMER4_OCO_HIGH_CH)
    {
        enPwmCh = Timer4PwmW;
    }
    else
    {
    }

    /* Initialize PWM I/O */
    PORT_SetFunc(TIMER4_PWM_H_PORT, TIMER4_PWM_H_PIN, Func_Tim4, Disable);

    /* Timer4 PWM: Initialize PWM configuration structure */
    stcPwmInit.enRtIntMaskCmd = Enable;
    stcPwmInit.enClkDiv = PwmPlckDiv1;
    stcPwmInit.enOutputState = PwmHPwmLHold;
    stcPwmInit.enMode = PwmThroughMode;
    TIMER4_PWM_Init(TIMER4_UNIT, enPwmCh, &stcPwmInit); /* Initialize timer4 pwm */

    /* Clear && Start CNT */
    TIMER4_CNT_ClearCountVal(TIMER4_UNIT);
    TIMER4_CNT_Start(TIMER4_UNIT);

    /* Set OCO compare value */
    TIMER4_OCO_WriteOccr(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, m_au16OccrVal[m_u16OccrValIdx++]);

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
