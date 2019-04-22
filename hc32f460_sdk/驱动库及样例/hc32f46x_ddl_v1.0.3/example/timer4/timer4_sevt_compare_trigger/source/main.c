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
 ** \brief This example demonstrates how to use the compare trigger function of
 **        Timer4Sevt.
 **
 **   - 2018-10-30  1.0  Hongjh First version for Device Driver Library of
 **                      Timer4Sevt
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
/* DCU unit */
#define DCU_UNIT                        M4_DCU1
#define DCU_DATA0_VAL                   0x0000u
#define DCU_DATA1_VAL                   0x4444u

/* Timer4 unit */
#define TIMER4_UNIT                     M4_TMR41

/* Timer4 CNT cycle */
#define TIMER4_CNT_CYCLE_VAL            (50000u)               /*< Timer4 counter cycle value */

/* Timer4 SEVT channel */
#define TIMER4_SEVT_CH                  Timer4SevtCh0
#define TIMER4_SEVT_CMP_VAL             (TIMER4_CNT_CYCLE_VAL*3/4)    /*< Timer4 counter compare value */
#define TIMER4_SEVT_TRG_EVT             SevtTrgEvtSCMUH

/* LED0(D23: red color) Port/Pin definition */
#define LED0_PORT                       PortE
#define LED0_PIN                        Pin06

/* LED1(D26: green color) Port/Pin definition */
#define LED1_PORT                       PortA
#define LED1_PIN                        Pin07

/* LED0 & LED1 */
#define LED0_ON()                       PORT_SetBits(LED0_PORT, LED0_PIN)
#define LED0_OFF()                      PORT_ResetBits(LED0_PORT, LED0_PIN)
#define LED1_ON()                       PORT_SetBits(LED1_PORT, LED1_PIN)
#define LED1_OFF()                      PORT_ResetBits(LED1_PORT, LED1_PIN)

/* Parameter validity check for Timer4 unit */
#define IS_VALID_TIMER4(__TMRx__)                                              \
(   (M4_TMR41 == (__TMRx__))                ||                                 \
    (M4_TMR42 == (__TMRx__))                ||                                 \
    (M4_TMR43 == (__TMRx__)))

/* Parameter validity check for SEVT trigger event */
#define IS_VALID_SEVT_TRG_EVT(x)                                                \
(   (SevtTrgEvtSCMUH == (x))                ||                                 \
    (SevtTrgEvtSCMUL == (x))                ||                                 \
    (SevtTrgEvtSCMVH == (x))                ||                                 \
    (SevtTrgEvtSCMVL == (x))                ||                                 \
    (SevtTrgEvtSCMWH == (x))                ||                                 \
    (SevtTrgEvtSCMWL == (x)))

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void LedInit(void);
static en_event_src_t GetTimer4SevtEventNum(M4_TMR4_TypeDef *TMR4x,
                                  en_timer4_sevt_trigger_evt_t enEvt);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint16_t m_au16Data0Val = 0u;
static uint16_t m_au16Data2Val = 0u;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Initialize LED.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void LedInit(void)
{
    stc_port_init_t stcPortInit;

    LED0_OFF();
    LED1_OFF();

    /* LED0&LED1 Port/Pin initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief Get Timer SEVT event number.
 **
 ** \param [in] TMR4x                   Pointer to Timer4 instance register base
 ** \arg M4_TMR41                       Timer4 unit 1 instance register base
 ** \arg M4_TMR42                       Timer4 unit 2 instance register base
 ** \arg M4_TMR43                       Timer4 unit 3 instance register base
 ** \param [in] enCh                    Channel of SEVT
 ** \arg Timer4SevtCh0                  Timer SEVT channel: 0
 ** \arg Timer4SevtCh1                  Timer SEVT channel: 1
 ** \arg Timer4SevtCh2                  Timer SEVT channel: 2
 ** \arg Timer4SevtCh3                  Timer SEVT channel: 3
 ** \arg Timer4SevtCh4                  Timer SEVT channel: 4
 ** \arg Timer4SevtCh5                  Timer SEVT channel: 5
 **
 ** \retval                             Timer SEVT event number
 **
 ******************************************************************************/
static en_event_src_t GetTimer4SevtEventNum(M4_TMR4_TypeDef *TMR4x,
                                  en_timer4_sevt_trigger_evt_t enCh)
{
    uint8_t u8Timer4Unit = 0u;
    uint8_t u8Timer4SevtCh = (uint8_t)enCh;
    static const en_event_src_t enSevtEventNum[3][6] = {
    {EVT_TMR41_SCMUH, EVT_TMR41_SCMUL, EVT_TMR41_SCMVH, EVT_TMR41_SCMVL, EVT_TMR41_SCMWH, EVT_TMR41_SCMWL},
    {EVT_TMR42_SCMUH, EVT_TMR42_SCMUL, EVT_TMR42_SCMVH, EVT_TMR42_SCMVL, EVT_TMR42_SCMWH, EVT_TMR42_SCMWL},
    {EVT_TMR43_SCMUH, EVT_TMR43_SCMUL, EVT_TMR43_SCMVH, EVT_TMR43_SCMVL, EVT_TMR43_SCMWH, EVT_TMR43_SCMWL}};

    DDL_ASSERT(IS_VALID_TIMER4(TMR4x));
    DDL_ASSERT(IS_VALID_SEVT_TRG_EVT(enCh));

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

    return enSevtEventNum[u8Timer4Unit][u8Timer4SevtCh];
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
    en_result_t enTestResult = Ok;
    stc_dcu_init_t stcDcuInit;
    stc_timer4_cnt_init_t stcCntInit;
    stc_timer4_sevt_init_t stcSevtInit;
    stc_timer4_sevt_trigger_cond_t stcSevtTrgCond;

    /* Clear structures */
    MEM_ZERO_STRUCT(stcDcuInit);
    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(stcSevtInit);
    MEM_ZERO_STRUCT(stcSevtTrgCond);

    /* Initialize LED */
    LedInit();

    /* Enable peripheral clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS|PWC_FCG0_PERIPH_DCU1, Enable);
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM41|PWC_FCG2_PERIPH_TIM42|PWC_FCG2_PERIPH_TIM43, Enable);

    /* Initialize DCU */
    stcDcuInit.u32IntSel = 0;
    stcDcuInit.enIntWinMode = DcuIntInvalid;
    stcDcuInit.enDataSize = DcuDataBit16;
    stcDcuInit.enOperation = DcuHwTrigOpAdd;
    DCU_Init(DCU_UNIT, &stcDcuInit);
    DCU_WriteDataHalfWord(DCU_UNIT, DcuRegisterData0, DCU_DATA0_VAL);
    DCU_WriteDataHalfWord(DCU_UNIT, DcuRegisterData1, DCU_DATA1_VAL);
    DCU_SetTriggerSrc(DCU_UNIT, GetTimer4SevtEventNum(TIMER4_UNIT, TIMER4_SEVT_TRG_EVT));

    /* Timer4 CNT : Initialize CNT configuration structure */
    stcCntInit.enBufferCmd = Disable;
    stcCntInit.enClk = Timer4CntPclk;
    stcCntInit.enClkDiv = Timer4CntPclkDiv16;  /* CNT clock divide */
    stcCntInit.enCntMode = Timer4CntTriangularWave;
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;
    TIMER4_CNT_Init(TIMER4_UNIT, &stcCntInit); /* Initialize CNT */
    TIMER4_CNT_SetCycleVal(TIMER4_UNIT, TIMER4_CNT_CYCLE_VAL);

    /* Timer4-SEVT: Initialize SEVT configuration structure */
    stcSevtInit.enBuf = SevtBufDisable;
    stcSevtInit.enMode = SevtCompareTrigMode;
    stcSevtInit.enTrigEvt = TIMER4_SEVT_TRG_EVT;
    TIMER4_SEVT_Init(TIMER4_UNIT, TIMER4_SEVT_CH, &stcSevtInit); /* Initialize SEVT */
    TIMER4_SEVT_WriteSCCR(TIMER4_UNIT, TIMER4_SEVT_CH, TIMER4_SEVT_CMP_VAL); /* Set SEVT compare value */

    stcSevtTrgCond.enUpMatchCmd = Enable;
    stcSevtTrgCond.enDownMatchCmd = Disable;
    stcSevtTrgCond.enZeroMatchCmd = Disable;
    stcSevtTrgCond.enPeakMatchCmd = Disable;
    TIMER4_SEVT_SetTriggerCond(TIMER4_UNIT, TIMER4_SEVT_CH, &stcSevtTrgCond); /* Set SEVT operation condition */

    /* Clear && Start CNT */
    TIMER4_CNT_ClearCountVal(TIMER4_UNIT);
    TIMER4_CNT_Start(TIMER4_UNIT);

    /* Wait DCU add operation */
    while (DCU_DATA0_VAL == m_au16Data0Val)
    {
        m_au16Data0Val = DCU_ReadDataHalfWord(DCU_UNIT, DcuRegisterData0);
    }

    /* Stop CNT */
    TIMER4_CNT_Stop(TIMER4_UNIT);

    m_au16Data2Val = DCU_ReadDataHalfWord(DCU_UNIT, DcuRegisterData2);
    if (m_au16Data0Val != (2 * m_au16Data2Val))
    {
        enTestResult = Error;
    }
    else
    {
    }

    if (Ok == enTestResult)
    {
        LED1_ON();  /* Test pass && meet the expected */
    }
    else
    {
        LED0_ON();  /* Test fail && don't meet the expected */
    }

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
