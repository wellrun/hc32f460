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
 ** \brief This example demonstrates how to use the dead timer filter mode
 **        function of Timer4Pwm.
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
#define TIMER4_CNT_CYCLE_VAL            (50000)        /* Timer4 counter cycle value */

/* Timer4 OCO */
#define TIMER4_OCO_LOW_CH               Timer4OcoOul   /* only Timer4OcoOul  Timer4OcoOvl  Timer4OcoOwl */

/* Define port and pin for Timer4Pwm */
#define TIMER4_PWM_H_PORT               PortE          /* TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13 */
#define TIMER4_PWM_H_PIN                Pin09
#define TIMER4_PWM_L_PORT               PortE          /* TIM4_1_OUL_B:PE8   TIM4_1_OVL_B:PE10   TIM4_1_OWL_B:PE12 */
#define TIMER4_PWM_L_PIN                Pin08

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
static void OcoIrqCallback(void);
static en_int_src_t GetTimer4OcoIntNum(M4_TMR4_TypeDef *TMR4x,
                                  en_timer4_oco_ch_t enCh);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t m_CntOcoMatchIrq = 0;
static uint16_t m_au16OccrVal[ ] = {6250, 12500, 25000, 25000};

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief oco match interrupt handler
 **
 ******************************************************************************/
static void OcoIrqCallback(void)
{
    m_CntOcoMatchIrq++;

    if (m_CntOcoMatchIrq >= ARRAY_SZ(m_au16OccrVal))
    {
        m_CntOcoMatchIrq = 0;
    }
    else
    {
    }

    TIMER4_OCO_ClearIrqFlag(TIMER4_UNIT, TIMER4_OCO_LOW_CH);
    TIMER4_OCO_WriteOccr(TIMER4_UNIT, TIMER4_OCO_LOW_CH, m_au16OccrVal[m_CntOcoMatchIrq]);
}

/**
 *******************************************************************************
 ** \brief Get Timer4 OCO interrupt number.
 **
 ** \param [in] TMR4x                   Pointer to Timer4 instance register base
 ** \arg M4_TMR41                       Timer4 unit 1 instance register base
 ** \arg M4_TMR42                       Timer4 unit 2 instance register base
 ** \arg M4_TMR43                       Timer4 unit 3 instance register base
 ** \param [in] enCh                    Channel of Timer4 OCO
 ** \arg Timer4OcoOuh                   Timer4 OCO channel:OUH
 ** \arg Timer4OcoOul                   Timer4 OCO channel:OUL
 ** \arg Timer4OcoOvh                   Timer4 OCO channel:OVH
 ** \arg Timer4OcoOvl                   Timer4 OCO channel:OVL
 ** \arg Timer4OcoOwh                   Timer4 OCO channel:OWH
 ** \arg Timer4OcoOwl                   Timer4 OCO channel:OWL
 **
 ** \retval                             Timer4 OCO interrupt number
 **
 ******************************************************************************/
static en_int_src_t GetTimer4OcoIntNum(M4_TMR4_TypeDef *TMR4x,
                                  en_timer4_oco_ch_t enCh)
{
    uint8_t u8Timer4Unit = 0u;
    uint8_t u8Timer4OcoCh = (uint8_t)enCh;
    static const en_int_src_t aenOcoIntNum[3][6] = {
    {INT_TMR41_GCMUH, INT_TMR41_GCMUL, INT_TMR41_GCMVH, INT_TMR41_GCMVL, INT_TMR41_GCMWH, INT_TMR41_GCMWL},
    {INT_TMR42_GCMUH, INT_TMR42_GCMUL, INT_TMR42_GCMVH, INT_TMR42_GCMVL, INT_TMR42_GCMWH, INT_TMR42_GCMWL},
    {INT_TMR43_GCMUH, INT_TMR43_GCMUL, INT_TMR43_GCMVH, INT_TMR43_GCMVL, INT_TMR43_GCMWH, INT_TMR43_GCMWL}};

    DDL_ASSERT(IS_VALID_OCO_CH(enCh));
    DDL_ASSERT(IS_VALID_TIMER4(TMR4x));

    if (M4_TMR41 == TMR4x)
    {
        u8Timer4Unit = 0u;
    }
    else if (M4_TMR42 == TMR4x)
    {
        u8Timer4Unit = 1u;
    }
    else if (M4_TMR43 == TMR4x)
    {
        u8Timer4Unit = 2u;
    }
    else
    {
    }

    return aenOcoIntNum[u8Timer4Unit][u8Timer4OcoCh];
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
    stc_irq_regi_conf_t stcIrqRegiCfg;
    stc_timer4_cnt_init_t stcCntInit;
    stc_timer4_oco_init_t stcOcoInit;
    stc_timer4_pwm_init_t stcPwmInit;
    stc_oco_low_ch_compare_mode_t stcLowChCmpMode;

    /* Clear structures */
    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(stcOcoInit);
    MEM_ZERO_STRUCT(stcPwmInit);
    MEM_ZERO_STRUCT(stcIrqRegiCfg);
    MEM_ZERO_STRUCT(stcLowChCmpMode);

    /* Enable peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM41 | PWC_FCG2_PERIPH_TIM42 | PWC_FCG2_PERIPH_TIM43, Enable);

    /* Timer4 CNT : Initialize CNT configuration structure */
    stcCntInit.enBufferCmd = Disable;
    stcCntInit.enClk = Timer4CntPclk;
    stcCntInit.enClkDiv = Timer4CntPclkDiv16;  /* CNT clock divide */
    stcCntInit.u16Cycle = TIMER4_CNT_CYCLE_VAL;
    stcCntInit.enCntMode = Timer4CntSawtoothWave;
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;
    TIMER4_CNT_Init(TIMER4_UNIT, &stcCntInit); /* Initialize CNT */

    /* Timer4 OCO : Initialize OCO configuration structure */
    stcOcoInit.enOcoIntCmd = Enable;
    stcOcoInit.enPortLevel = OcPortLevelLow;
    stcOcoInit.enOccrBufMode = OccrBufDisable;
    stcOcoInit.enOcmrBufMode = OcmrBufDisable;
    TIMER4_OCO_Init(TIMER4_UNIT, TIMER4_OCO_LOW_CH, &stcOcoInit);  /* Initialize OCO low channel */

    /*************Timer4 OCO ocmr1[31:0] = 0x0FF0 0FFF*****************************/
    if ((Timer4OcoOul == TIMER4_OCO_LOW_CH) || (Timer4OcoOvl == TIMER4_OCO_LOW_CH) || (Timer4OcoOwl == TIMER4_OCO_LOW_CH))
    {
        /* OCMR[31:0] Ox 0FF0 0FFF    0000 1111 1111 0000   0000 1111 1111 1111 */
        stcLowChCmpMode.enCntZeroLowMatchHighMatchLowChOpState = OcoOpOutputReverse;         /* bit[27:26]  11 */
        stcLowChCmpMode.enCntZeroLowMatchHighNotMatchLowChOpState = OcoOpOutputReverse;      /* bit[11:10]  11 */
        stcLowChCmpMode.enCntZeroLowNotMatchHighMatchLowChOpState = OcoOpOutputHold;         /* bit[31:30]  00 */
        stcLowChCmpMode.enCntZeroLowNotMatchHighNotMatchLowChOpState = OcoOpOutputHold;      /* bit[15:14]  00 */

        stcLowChCmpMode.enCntUpCntLowMatchHighMatchLowChOpState = OcoOpOutputReverse;        /* bit[25:24]  11 */
        stcLowChCmpMode.enCntUpCntLowMatchHighNotMatchLowChOpState = OcoOpOutputReverse;     /* bit[9:8]    11 */
        stcLowChCmpMode.enCntUpCntLowNotMatchHighMatchLowChOpState = OcoOpOutputHold;        /* bit[19:18]  00 */

        stcLowChCmpMode.enCntPeakLowMatchHighMatchLowChOpState = OcoOpOutputReverse;         /* bit[23:22]  11 */
        stcLowChCmpMode.enCntPeakLowMatchHighNotMatchLowChOpState = OcoOpOutputReverse;      /* bit[7:6]    11 */
        stcLowChCmpMode.enCntPeakLowNotMatchHighMatchLowChOpState = OcoOpOutputHold;         /* bit[29:28]  00 */
        stcLowChCmpMode.enCntPeakLowNotMatchHighNotMatchLowChOpState = OcoOpOutputHold;      /* bit[13:12]  00 */

        stcLowChCmpMode.enCntDownLowMatchHighMatchLowChOpState = OcoOpOutputReverse;         /* bit[21:20]  11 */
        stcLowChCmpMode.enCntDownLowMatchHighNotMatchLowChOpState = OcoOpOutputReverse;      /* bit[5:4]    11 */
        stcLowChCmpMode.enCntDownLowNotMatchHighMatchLowChOpState = OcoOpOutputHold;         /* bit[17:16]  00 */

        stcLowChCmpMode.enCntZeroMatchOcfState = OcoOcfSet;    /* bit[3] 1 */
        stcLowChCmpMode.enCntUpCntMatchOcfState = OcoOcfSet;   /* bit[2] 1 */
        stcLowChCmpMode.enCntPeakMatchOcfState = OcoOcfSet;    /* bit[1] 1 */
        stcLowChCmpMode.enCntDownCntMatchOcfState = OcoOcfSet; /* bit[0] 1 */

        TIMER4_OCO_SetLowChCompareMode(TIMER4_UNIT, TIMER4_OCO_LOW_CH, &stcLowChCmpMode);  /* Set OCO low channel compare mode */
    }
    else
    {
    }

    /* Set OCO compare value */
    TIMER4_OCO_WriteOccr(TIMER4_UNIT, TIMER4_OCO_LOW_CH, m_au16OccrVal[0]); /* Set OCO low channel compare value */

    /* Enable OCO */
    TIMER4_OCO_OutputCompareCmd(TIMER4_UNIT, TIMER4_OCO_LOW_CH, Enable);

    /* Set Timer4 OCO IRQ */
    stcIrqRegiCfg.enIRQn = Int000_IRQn;
    stcIrqRegiCfg.pfnCallback = OcoIrqCallback;
    stcIrqRegiCfg.enIntSrc = GetTimer4OcoIntNum(TIMER4_UNIT, TIMER4_OCO_LOW_CH);
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Timer4 PWM: Get pwm couple channel */
    if (Timer4OcoOul == TIMER4_OCO_LOW_CH)
    {
        enPwmCh = Timer4PwmU;
    }
    else if (Timer4OcoOvl == TIMER4_OCO_LOW_CH)
    {
        enPwmCh = Timer4PwmV;
    }
    else if (Timer4OcoOwl == TIMER4_OCO_LOW_CH)
    {
        enPwmCh = Timer4PwmW;
    }
    else
    {
    }

    /* Initialize PWM I/O */
    PORT_SetFunc(TIMER4_PWM_H_PORT, TIMER4_PWM_H_PIN, Func_Tim4, Disable);
    PORT_SetFunc(TIMER4_PWM_L_PORT, TIMER4_PWM_L_PIN, Func_Tim4, Disable);

    /* Timer4 PWM: Initialize PWM configuration structure */
    stcPwmInit.enRtIntMaskCmd = Enable;
    stcPwmInit.enClkDiv = PwmPlckDiv16;
    stcPwmInit.enOutputState = PwmHPwmLHold;
    stcPwmInit.enMode = PwmDeadTimerFilterMode;
    TIMER4_PWM_SetFilterCountValue(TIMER4_UNIT, enPwmCh, (m_au16OccrVal[1] + m_au16OccrVal[2])/2);
    TIMER4_PWM_WriteDeadRegionValue(TIMER4_UNIT, enPwmCh, 1u, 1u);
    TIMER4_PWM_Init(TIMER4_UNIT, enPwmCh, &stcPwmInit); /* Initialize timer4 pwm */

    /* Clear && Start CNT */
    TIMER4_CNT_ClearCountVal(TIMER4_UNIT);
    TIMER4_CNT_Start(TIMER4_UNIT);

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
