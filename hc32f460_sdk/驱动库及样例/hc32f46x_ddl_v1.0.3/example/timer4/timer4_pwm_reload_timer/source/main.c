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
 ** \brief This example demonstrates how to use the reload timer mode function
 **        of Timer4Pwm.
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

/* Timer4 PWM */
#define TIMER4_PWM_CH                   Timer4PwmU  /* Timer4PwmU  Timer4PwmV  Timer4PwmW */
#define TIMER4_PWM_RT_VAL               (50000u)

/* Parameter validity check for PWM channel */
#define IS_VALID_PWM_CH(x)                                                     \
(   (Timer4PwmU == (x))                 ||                                     \
    (Timer4PwmV == (x))                 ||                                     \
    (Timer4PwmW == (x)))

/* Parameter validity check for timer4 unit */
#define IS_VALID_TIMER4(__TMRx__)                                              \
(   (M4_TMR41 == (__TMRx__))            ||                                     \
    (M4_TMR42 == (__TMRx__))            ||                                     \
    (M4_TMR43 == (__TMRx__)))

/* Wave I/O */
#define WAVE_IO_PORT                    PortE
#define WAVE_IO_PIN                     Pin09

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void ToggleIo(void);
static void PwmCoupleChIrqCallback(void);
static en_int_src_t GetTimer4RtIntNum(M4_TMR4_TypeDef *TMR4x,
                                  en_timer4_pwm_ch_t enCh);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Toggle gpio.
 **
 ** \param [in]                   None
 **
 ** \retval                       None
 **
 ******************************************************************************/
static void ToggleIo(void)
{
    PORT_Toggle(WAVE_IO_PORT, WAVE_IO_PIN);
    PORT_Toggle(WAVE_IO_PORT, WAVE_IO_PIN);
}

/**
 *******************************************************************************
 ** \brief pwm couple channel reload timer interrupt handler
 **
 ******************************************************************************/
static void PwmCoupleChIrqCallback(void)
{
    ToggleIo();

    TIMER4_PWM_ClearIrqFlag(TIMER4_UNIT, TIMER4_PWM_CH);
}

/**
 *******************************************************************************
 ** \brief Get Timer4 PWM reload-timer interrupt number.
 **
 ** \param [in] TMR4x                   Pointer to Timer4 instance register base
 ** \arg M4_TMR41                       Timer4 unit 1 instance register base
 ** \arg M4_TMR42                       Timer4 unit 2 instance register base
 ** \arg M4_TMR43                       Timer4 unit 3 instance register base
 ** \param [in] enCh                    Channel of Timer4 PWM
 ** \arg Timer4PwmU                     Timer4 PWM channel: U
 ** \arg Timer4PwmV                     Timer4 PWM channel: V
 ** \arg Timer4PwmW                     Timer4 PWM channel: W
 **
 ** \retval                             PWM reload-timer interrupt number
 **
 ******************************************************************************/
static en_int_src_t GetTimer4RtIntNum(M4_TMR4_TypeDef *TMR4x,
                                  en_timer4_pwm_ch_t enCh)
{
    uint8_t u8Timer4Unit = 0u;
    uint8_t u8Timer4PwmCh = (uint8_t)enCh;
    static const en_int_src_t aenRtIntNum[3][3] = {
    {INT_TMR41_RLOU, INT_TMR41_RLOV, INT_TMR41_RLOW},
    {INT_TMR42_RLOU, INT_TMR42_RLOV, INT_TMR42_RLOW},
    {INT_TMR43_RLOU, INT_TMR43_RLOV, INT_TMR43_RLOW}};

    DDL_ASSERT(IS_VALID_PWM_CH(enCh));
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

    return aenRtIntNum[u8Timer4Unit][u8Timer4PwmCh];
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
    stc_port_init_t stcPortInit;
    stc_irq_regi_conf_t stcIrqRegiCfg;
    stc_timer4_pwm_init_t stcPwmInit;

    /* Clear structures */
    MEM_ZERO_STRUCT(stcPwmInit);
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcIrqRegiCfg);

    /* Initialize Port/Pin */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    PORT_Init(WAVE_IO_PORT, WAVE_IO_PIN, &stcPortInit);

    /* Enable peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM41 | PWC_FCG2_PERIPH_TIM42 | PWC_FCG2_PERIPH_TIM43, Enable);

    /* Timer4 PWM: Initialize PWM couple channel configuration structure */
    stcPwmInit.enMode = PwmDeadTimerMode;  /* Change: PwmDeadTimerMode  PwmThroughMode */
    stcPwmInit.enClkDiv = PwmPlckDiv16;
    stcPwmInit.enRtIntMaskCmd = Disable;
    stcPwmInit.enOutputState = PwmHPwmLHold;  /* change: PwmHPwmLHold  PwmHPwmLReverse  PwmHReversePwmLHold  PwmHHoldPwmLReverse */
    TIMER4_PWM_Init(TIMER4_UNIT, TIMER4_PWM_CH, &stcPwmInit);  /* Initialize PWM channel */
    TIMER4_PWM_SetFilterCountValue(TIMER4_UNIT, TIMER4_PWM_CH, TIMER4_PWM_RT_VAL);

    /* Set Timer4 PWM RT IRQ */
    MEM_ZERO_STRUCT(stcIrqRegiCfg);
    stcIrqRegiCfg.enIRQn = Int000_IRQn;
    stcIrqRegiCfg.pfnCallback = PwmCoupleChIrqCallback;
    stcIrqRegiCfg.enIntSrc = GetTimer4RtIntNum(TIMER4_UNIT, TIMER4_PWM_CH);
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Start pwm count */
    TIMER4_PWM_StartTimer(TIMER4_UNIT, TIMER4_PWM_CH);

    /* Toggle I/O */
    ToggleIo();

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
