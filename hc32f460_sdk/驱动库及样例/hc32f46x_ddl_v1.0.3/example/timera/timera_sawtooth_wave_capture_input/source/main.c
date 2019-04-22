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
 ** \brief The example of Timera capture function
 **
 **   - 2018-11-13  1.0  Yangjp First version for Device Driver Library of
 **                      Timera.
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
/* LED0 Port/Pin definition */
#define LED0_PORT                       PortE
#define LED0_PIN                        Pin06

#define LED0_ON()                       PORT_SetBits(LED0_PORT, LED0_PIN)
#define LED0_OFF()                      PORT_ResetBits(LED0_PORT, LED0_PIN)
#define LED0_TOGGLE()                   PORT_Toggle(LED0_PORT, LED0_PIN)

/* LED1 Port/Pin definition */
#define LED1_PORT                       PortA
#define LED1_PIN                        Pin07

#define LED1_ON()                       PORT_SetBits(LED1_PORT, LED1_PIN)
#define LED1_OFF()                      PORT_ResetBits(LED1_PORT, LED1_PIN)
#define LED1_TOGGLE()                   PORT_Toggle(LED1_PORT, LED1_PIN)

/* KEY1 Port/Pin definition */
#define KEY1_PORT                       PortD
#define KEY1_PIN                        Pin04
#define KEY1_TRIGGER_EVENT              EVT_PORT_EIRQ4

/* TIMERA unit and clock definition */
#define TIMERA_UNIT1                    M4_TMRA1
#define TIMERA_UNIT1_CLOCK              PWC_FCG2_PERIPH_TIMA1
#define TIMERA_UNIT1_COMPARE_INT        INT_TMRA1_CMP

/* TIMERA channel 1 Port/Pin definition */
#define TIMERA_UNIT1_CH1                TimeraCh1
#define TIMERA_UNIT1_CH1_PORT           PortE
#define TIMERA_UNIT1_CH1_PIN            Pin09
#define TIMERA_UNIT1_CH1_FUNC           Func_Tima0
#define TIMERA_UNIT1_CH1_INT_FLAG       TimeraFlagCaptureOrCompareCh1
#define TIMERA_UNIT1_CH1_INT            TimeraIrqCaptureOrCompareCh1

/* TIMERA channel 2 Port/Pin definition */
#define TIMERA_UNIT1_CH2                TimeraCh2
#define TIMERA_UNIT1_CH2_PORT           PortE
#define TIMERA_UNIT1_CH2_PIN            Pin11
#define TIMERA_UNIT1_CH2_FUNC           Func_Tima0
#define TIMERA_UNIT1_CH2_INT_FLAG       TimeraFlagCaptureOrCompareCh2
#define TIMERA_UNIT1_CH2_INT            TimeraIrqCaptureOrCompareCh2

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Timera unit 1 callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void TimeraUnit1_IrqCallback(void)
{
    /* Capture channel 0 */
    if (Set == TIMERA_GetFlag(TIMERA_UNIT1, TIMERA_UNIT1_CH1_INT_FLAG))
    {
        LED0_TOGGLE();
        TIMERA_ClearFlag(TIMERA_UNIT1, TIMERA_UNIT1_CH1_INT_FLAG);
    }
    /* Capture channel 1 */
    if (Set == TIMERA_GetFlag(TIMERA_UNIT1, TIMERA_UNIT1_CH2_INT_FLAG))
    {
        LED1_TOGGLE();
        TIMERA_ClearFlag(TIMERA_UNIT1, TIMERA_UNIT1_CH2_INT_FLAG);
    }
}

/**
 *******************************************************************************
 ** \brief System clock init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void SystemClk_Init(void)
{
    en_clk_sys_source_t     enSysClkSrc;
    stc_clk_sysclk_cfg_t    stcSysClkCfg;
    stc_clk_xtal_cfg_t      stcXtalCfg;
    stc_clk_mpll_cfg_t      stcMpllCfg;
    stc_clk_freq_t          stcClkFreq;

    MEM_ZERO_STRUCT(enSysClkSrc);
    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv = ClkSysclkDiv1;   // Max 168MHz
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;  // Max 168MHz
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;  // Max 60MHz
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;  // Max 42MHz
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;  // Max 84MHz
    CLK_SysClkConfig(&stcSysClkCfg);

    /* Switch system clock source to MPLL. */
    /* Use Xtal32 as MPLL source. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* MPLL config. */
    stcMpllCfg.pllmDiv = 1;
    stcMpllCfg.plln = 42;
    stcMpllCfg.PllpDiv = 2;
    stcMpllCfg.PllqDiv = 2;
    stcMpllCfg.PllrDiv = 2;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_5);
    EFM_Lock();

    /* Enable MPLL. */
    CLK_MpllCmd(Enable);

    /* Wait MPLL ready. */
    while (Set != CLK_GetFlagStatus(ClkFlagMPLLRdy));

    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);

    /* Check source and frequence. */
    enSysClkSrc = CLK_GetSysClkSource();
    CLK_GetClockFreq(&stcClkFreq);
}

/**
 *******************************************************************************
 ** \brief Configure Timera peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Timera_Config(void)
{
    stc_timera_base_init_t stcTimeraInit;
    stc_timera_capture_init_t stcTimeraCaptureInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcTimeraCaptureInit);
    MEM_ZERO_STRUCT(stcPortInit);

    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(TIMERA_UNIT1_CLOCK, Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS, Enable);

    /* Configuration TIMERA capture pin */
    PORT_SetFunc(TIMERA_UNIT1_CH1_PORT, TIMERA_UNIT1_CH1_PIN, TIMERA_UNIT1_CH1_FUNC, Disable);
    PORT_SetFunc(TIMERA_UNIT1_CH2_PORT, TIMERA_UNIT1_CH2_PIN, TIMERA_UNIT1_CH2_FUNC, Disable);

    /* Configuration timera unit 1 base structure */
    stcTimeraInit.enClkDiv = TimeraPclkDiv256;
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = 0x802C;        //100ms
    TIMERA_BaseInit(TIMERA_UNIT1, &stcTimeraInit);

    /* Configuration timera unit 1 capture structure */
    stcTimeraCaptureInit.enCapturePwmRisingEn = Enable;
    stcTimeraCaptureInit.enCapturePwmFallingEn = Disable;
    stcTimeraCaptureInit.enCaptureSpecifyEventEn = Enable;
    stcTimeraCaptureInit.enPwmClkDiv = TimeraFilterPclkDiv4;
    stcTimeraCaptureInit.enPwmFilterEn = Enable;
    stcTimeraCaptureInit.enCaptureTrigRisingEn = Disable;
    stcTimeraCaptureInit.enCaptureTrigFallingEn = Disable;
    stcTimeraCaptureInit.enTrigClkDiv = TimeraFilterPclkDiv1;
    stcTimeraCaptureInit.enTrigFilterEn = Disable;
    /* Enable channel 1 capture and interrupt */
    TIMERA_CaptureInit(TIMERA_UNIT1, TIMERA_UNIT1_CH1, &stcTimeraCaptureInit);
    TIMERA_IrqCmd(TIMERA_UNIT1, TIMERA_UNIT1_CH1_INT, Enable);

    /* Enable channel 2 capture and interrupt */
    TIMERA_CaptureInit(TIMERA_UNIT1, TIMERA_UNIT1_CH2, &stcTimeraCaptureInit);
    TIMERA_IrqCmd(TIMERA_UNIT1, TIMERA_UNIT1_CH2_INT, Enable);

    /* Configure interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT1_COMPARE_INT;
    stcIrqRegiConf.enIRQn = Int006_IRQn;
    stcIrqRegiConf.pfnCallback = TimeraUnit1_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Set external Int Ch.4 trigger timera compare */
    stcPortInit.enExInt = Enable;
    PORT_Init(KEY1_PORT, KEY1_PIN, &stcPortInit);
    TIMERA_SetCaptureTriggerSrc(KEY1_TRIGGER_EVENT);

    /* Timera unit 1 startup */
    TIMERA_Cmd(TIMERA_UNIT1, Enable);
}

/**
 *******************************************************************************
 ** \brief  main function for Timera capture function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_port_init_t stcPortInit;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);

    /* Configure system clock frequency */
    SystemClk_Init();

    /* LED0 Port/Pin initialization */
    LED0_OFF();
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* LED1 Port/Pin initialization */
    LED1_OFF();
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);

    /* Configure Timera */
    Timera_Config();

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
