/*******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co.,Ltd All rights reserved.
 *
 * This software is owned and published by:
 * Huada Semiconductor Co.,Ltd ("HDSC").
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
 ** \brief This sample demonstrates how to use timer0.
 **
 **   - 2018-11-09  1.0  wangmin first version for inside trigger function of Timer0.
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
/* KEY0 */
#define  SW2_PORT           PortD
#define  SW2_PIN            Pin03

/* LED0 Port/Pin definition */
#define  LED0_PORT          PortE
#define  LED0_PIN           Pin06

/* LED1 Port/Pin definition */
#define  LED1_PORT          PortA
#define  LED1_PIN           Pin07

/* LED0~1 toggle definition */
#define  LED0_TOGGLE()    PORT_Toggle(LED0_PORT, LED0_PIN)
#define  LED1_TOGGLE()    PORT_Toggle(LED1_PORT, LED1_PIN)

/* Define Timer Unit for example */
#define TMR_UNIT            M4_TMR02
#define TMR_INI_GCMA        INT_TMR02_GCMA
#define TMR_INI_GCMB        INT_TMR02_GCMB

#define ENABLE_TMR0()       PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02, Enable)
/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint16_t u16CmpLast = 0;
static __IO uint16_t u16Campture = 0;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Callback function of timer0
 **
 ******************************************************************************/
void Timer0_TriggerCallBack(void)
{
    uint16_t tmp;
    tmp = TIMER0_GetCmpReg(TMR_UNIT,Tim0_ChannelB);

    u16Campture = tmp - u16CmpLast;
    u16CmpLast = tmp;

    LED0_TOGGLE();
}

/**
 *******************************************************************************
 ** \brief Callback function of external interrupt ch.0
 **
 ******************************************************************************/
void ExtInt03_Callback(void)
{
    __asm("nop");
}

 /*******************************************************************************
 ** \brief SW2 init function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Sw2_Init(void)
{
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    /* External Int Ch.3 */
    stcExtiConfig.enExitCh = ExtiCh03;

    /* Filter setting */
    stcExtiConfig.enFilterEn = Enable;
    stcExtiConfig.enFltClk = Pclk3Div8;
    /* Both edge */
    stcExtiConfig.enExtiLvl = ExIntBothEdge;
    EXINT_Init(&stcExtiConfig);

    /* Set PD03 as External Int Ch.3 input */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enExInt = Enable;
    PORT_Init(SW2_PORT, SW2_PIN, &stcPortInit);
}

/**
 ******************************************************************************
 ** \brief  Initialize the system clock for the sample
 **
 ** \param  None
 **
 ** \retval None
 ******************************************************************************/
static void SysClkIni(void)
{
    en_clk_sys_source_t     enSysClkSrc;
    stc_clk_sysclk_cfg_t    stcSysClkCfg;
    stc_clk_xtal_cfg_t      stcXtalCfg;
    stc_clk_mpll_cfg_t      stcMpllCfg;

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
    /*system clk = 21M, pclk1 = 10.5M, pclk3 = 5.25M*/
    stcMpllCfg.pllmDiv = 1;
    stcMpllCfg.plln =42;
    stcMpllCfg.PllpDiv = 16;
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
    while(Set != CLK_GetFlagStatus(ClkFlagMPLLRdy));

    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);
}

/**
 *******************************************************************************
 ** \brief  Main function of example project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_tim0_base_init_t stcTimerCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_tim0_trigger_init_t stcInputCaptruecfg;
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcTimerCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcInputCaptruecfg);
    MEM_ZERO_STRUCT(stcPortInit);

    /* system clock intialize */
    SysClkIni();
    /*init sw2*/
    Sw2_Init();

    /*initiallize LED port*/
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    /* LED0 and LED1 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);

    /* Timer0 peripheral enable and config */
    ENABLE_TMR0();
    stcTimerCfg.Tim0_CounterMode = Tim0_Sync;
    stcTimerCfg.Tim0_SyncClockSource = Tim0_Pclk1;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv32;
    TIMER0_BaseInit(TMR_UNIT,Tim0_ChannelB,&stcTimerCfg);
    TIMER0_IntCmd(TMR_UNIT,Tim0_ChannelB,Enable);

    /* Input captrue enable */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS, Enable);
    /*config timer0 trigger function*/
    stcInputCaptruecfg.Tim0_OCMode = Tim0_InputCaptrue;
    stcInputCaptruecfg.Tim0_SelTrigSrc = EVT_PORT_EIRQ3;
    stcInputCaptruecfg.Tim0_InTrigEnable = true;
    stcInputCaptruecfg.Tim0_InTrigClear = false;
    stcInputCaptruecfg.Tim0_InTrigStart = true;
    stcInputCaptruecfg.Tim0_InTrigStop = false;
    TIMER0_HardTriggerInit(TMR_UNIT,Tim0_ChannelB,&stcInputCaptruecfg);

    /* Enable channel B interrupt */
    TIMER0_IntCmd(TMR_UNIT,Tim0_ChannelB,Enable);
    /* Register TMR_INI_GCMB Int to Vect.No.002 */
    stcIrqRegiConf.enIRQn = Int002_IRQn;
    /* Select TIMER channalB interrupt as source */
    stcIrqRegiConf.enIntSrc = TMR_INI_GCMB;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = Timer0_TriggerCallBack;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    while(1)
    {
        if(u16CmpLast != 0)
        {
            LED1_TOGGLE();
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
