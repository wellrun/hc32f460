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
 ** \brief The example of Timera compare function
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

/* KEY0 Port/Pin definition */
#define KEY0_PORT                       PortD
#define KEY0_PIN                        Pin03

/* KEY1 Port/Pin definition */
#define KEY1_PORT                       PortD
#define KEY1_PIN                        Pin04
#define KEY1_TRIGGER_EVENT              EVT_PORT_EIRQ4

/* TIMERA unit and clock definition */
#define TIMERA_UNIT1                    M4_TMRA1
#define TIMERA_UNIT1_CLOCK              PWC_FCG2_PERIPH_TIMA1
#define TIMERA_UNIT1_OVERFLOW_INT       INT_TMRA1_OVF

/* TIMERA channel 1 Port/Pin definition */
#define TIMERA_UNIT1_CH1                TimeraCh1
#define TIMERA_UNIT1_CH1_PORT           PortE
#define TIMERA_UNIT1_CH1_PIN            Pin09
#define TIMERA_UNIT1_CH1_FUNC           Func_Tima0

/* TIMERA channel 3 Port/Pin definition */
#define TIMERA_UNIT1_CH3                TimeraCh3
#define TIMERA_UNIT1_CH3_PORT           PortE
#define TIMERA_UNIT1_CH3_PIN            Pin13
#define TIMERA_UNIT1_CH3_FUNC           Func_Tima0

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t u8ExIntFlag = 0, u8TmraUnit1Cnt = 0;

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
    u8TmraUnit1Cnt++;
    if (u8TmraUnit1Cnt >= 100)      //1s
    {
        u8TmraUnit1Cnt = 0u;
        LED0_TOGGLE();
    }
    TIMERA_ClearFlag(TIMERA_UNIT1, TimeraFlagOverflow);
}

/**
 *******************************************************************************
 ** \brief ExtInt3 callback function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt03_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh03))
    {
        u8ExIntFlag = 1u;
        EXINT_IrqFlgClr(ExtiCh03);
    }
}

/**
 *******************************************************************************
 ** \brief KEY0(SW2) init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Sw2_Init(void)
{
    stc_port_init_t stcPortInit;
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Set PD03 as External Int Ch.3 input */
    stcPortInit.enExInt = Enable;
    PORT_Init(KEY0_PORT, KEY0_PIN, &stcPortInit);

    stcExtiConfig.enExitCh = ExtiCh03;
    /* Filter setting */
    stcExtiConfig.enFilterEn = Enable;
    stcExtiConfig.enFltClk = Pclk3Div8;
    /* Both edge */
    stcExtiConfig.enExtiLvl = ExIntFallingEdge;
    EXINT_Init(&stcExtiConfig);

    /* Select External Int Ch.3 */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ3;
    /* Register External Int to Vect.No.007 */
    stcIrqRegiConf.enIRQn = Int007_IRQn;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = ExtInt03_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
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
    stc_timera_compare_init_t stcTimerCompareInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_timera_hw_startup_cofig_t stcTimeraHwConfig;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcTimerCompareInit);
    MEM_ZERO_STRUCT(stcTimeraHwConfig);
    MEM_ZERO_STRUCT(stcPortInit);

    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(TIMERA_UNIT1_CLOCK, Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS, Enable);

    /* Configuration TIMERA compare pin */
    PORT_SetFunc(TIMERA_UNIT1_CH1_PORT, TIMERA_UNIT1_CH1_PIN, TIMERA_UNIT1_CH1_FUNC, Disable);
    PORT_SetFunc(TIMERA_UNIT1_CH3_PORT, TIMERA_UNIT1_CH3_PIN, TIMERA_UNIT1_CH3_FUNC, Disable);

    /* Configuration timera unit 1 base structure */
    stcTimeraInit.enClkDiv = TimeraPclkDiv128;
    stcTimeraInit.enCntMode = TimeraCountModeTriangularWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = 0xCD0;        //freq: 100Hz
    TIMERA_BaseInit(TIMERA_UNIT1, &stcTimeraInit);

    /* Configuration timera unit 1 compare structure */
    stcTimerCompareInit.u16CompareVal = stcTimeraInit.u16PeriodVal * 4 / 5;
    stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputLow;
    stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputLow;
    stcTimerCompareInit.enCompareMatchOutput = TimeraCompareMatchOutputReverse;
    stcTimerCompareInit.enPeriodMatchOutput = TimeraPeriodMatchOutputKeep;
    stcTimerCompareInit.enSpecifyOutput = TimeraSpecifyOutputInvalid;
    stcTimerCompareInit.enCacheEn = Enable;
    stcTimerCompareInit.enTriangularTroughTransEn = Enable;
    stcTimerCompareInit.enTriangularCrestTransEn = Disable;
    stcTimerCompareInit.u16CompareCacheVal = stcTimerCompareInit.u16CompareVal;
    /* Configure Channel 1 */
    TIMERA_CompareInit(TIMERA_UNIT1, TIMERA_UNIT1_CH1, &stcTimerCompareInit);
    TIMERA_CompareCmd(TIMERA_UNIT1, TIMERA_UNIT1_CH1, Enable);

    /* Configure channel 3 */
    stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputHigh;
    stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputHigh;
    TIMERA_CompareInit(TIMERA_UNIT1, TIMERA_UNIT1_CH3, &stcTimerCompareInit);
    TIMERA_CompareCmd(TIMERA_UNIT1, TIMERA_UNIT1_CH3, Enable);

    /* Enable period count interrupt */
    TIMERA_IrqCmd(TIMERA_UNIT1, TimeraIrqOverflow, Enable);
    /* Interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT1_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = Int006_IRQn;
    stcIrqRegiConf.pfnCallback = TimeraUnit1_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Configure timera unit 1 startup */
    stcTimeraHwConfig.enSpecifyEventStartupEn = Enable;
    stcTimeraHwConfig.enTrigFallingStartupEn = Disable;
    stcTimeraHwConfig.enTrigRisingStartupEn = Disable;
    TIMERA_HwStartupConfig(TIMERA_UNIT1, &stcTimeraHwConfig);

    /* Set external Int Ch.4 trigger timera startup */
    stcPortInit.enExInt = Enable;
    PORT_Init(KEY1_PORT, KEY1_PIN, &stcPortInit);
    TIMERA_SetCountTriggerSrc(KEY1_TRIGGER_EVENT);
}

/**
 *******************************************************************************
 ** \brief  main function for Timera compare function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    uint16_t u16TimerPeriod = 0, u16DutyCycle = 0;
    stc_port_init_t stcPortInit;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);

    /* Configure system clock frequency */
    SystemClk_Init();

    /* LED0 Port/Pin initialization */
    LED0_OFF();
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* Key0 Port/Pin initialization */
    Sw2_Init();
    /* Configure Timera */
    Timera_Config();
    u16DutyCycle = TIMERA_GetCompareValue(TIMERA_UNIT1, TIMERA_UNIT1_CH1);
    u16TimerPeriod = TIMERA_GetPeriodValue(TIMERA_UNIT1);

    while (1)
    {
        if (1u == u8ExIntFlag)
        {
            u8ExIntFlag = 0;
            u16DutyCycle += u16TimerPeriod / 20;
            if (u16DutyCycle > u16TimerPeriod)
            {
                u16DutyCycle = 0;
            }
            TIMERA_SetCacheValue(TIMERA_UNIT1, TIMERA_UNIT1_CH1, u16DutyCycle);
            TIMERA_SetCacheValue(TIMERA_UNIT1, TIMERA_UNIT1_CH3, u16DutyCycle);
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
