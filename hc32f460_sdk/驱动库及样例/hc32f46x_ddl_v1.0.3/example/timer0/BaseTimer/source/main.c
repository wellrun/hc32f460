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
 **   - 2018-11-06  1.0  wangmin first version for base timer function of Timer0.
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

#define ENABLE_TMR0()      PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02, Enable)

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
 ** \brief Callback function of external interrupt ch.0
 **
 ******************************************************************************/
void Timer0A_CallBack(void)
{
    LED0_TOGGLE();
}

void Timer0B_CallBack(void)
{
    LED1_TOGGLE();
}

/**
 ******************************************************************************
 ** \brief  Initialize the system clock for the sample
 **
 ** \param  None
 **
 ** \return None
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
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv4;  // Max 42MHz
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
    /*system clk = 168M, pclk1 = 84M, pclk3 = 42M*/
    stcMpllCfg.pllmDiv = 1;
    stcMpllCfg.plln =42;
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
    stc_port_init_t stcPortInit;
    __IO uint16_t u16cmp = 0;
    __IO uint16_t u16cnt = 0;
    uint32_t u32Pclk1;
    stc_clk_freq_t stcClkTmp;
    uint32_t u32tmp;

    MEM_ZERO_STRUCT(stcTimerCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    SysClkIni();

    /*initiallize LED port*/
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    /* LED0 and LED1 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);

    /* Get pclk1 */
    CLK_GetClockFreq(&stcClkTmp);
    u32Pclk1 = stcClkTmp.pclk1Freq;

    /* Timer0 peripheral enable */
    ENABLE_TMR0();
    /*config register for channel A */
    stcTimerCfg.Tim0_CounterMode = Tim0_Async;
    stcTimerCfg.Tim0_AsyncClockSource = Tim0_XTAL32;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv4;
    stcTimerCfg.Tim0_CmpValue = 32000/4 - 1;
    TIMER0_BaseInit(TMR_UNIT,Tim0_ChannelA,&stcTimerCfg);

    /* Enable channel A interrupt */
    TIMER0_IntCmd(TMR_UNIT,Tim0_ChannelA,Enable);
    /* Register TMR_INI_GCMA Int to Vect.No.001 */
    stcIrqRegiConf.enIRQn = Int001_IRQn;
    /* Select I2C Error or Event interrupt function */
    stcIrqRegiConf.enIntSrc = TMR_INI_GCMA;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = Timer0A_CallBack;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /*config register for channel B */
    stcTimerCfg.Tim0_CounterMode = Tim0_Sync;
    stcTimerCfg.Tim0_SyncClockSource = Tim0_Pclk1;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv1024;
    stcTimerCfg.Tim0_CmpValue = u32Pclk1/1024 - 1;
    TIMER0_BaseInit(TMR_UNIT,Tim0_ChannelB,&stcTimerCfg);

    /* Enable channel B interrupt */
    TIMER0_IntCmd(TMR_UNIT,Tim0_ChannelB,Enable);
    /* Register TMR_INI_GCMB Int to Vect.No.002 */
    stcIrqRegiConf.enIRQn = Int002_IRQn;
    /* Select I2C Error or Event interrupt function */
    stcIrqRegiConf.enIntSrc = TMR_INI_GCMB;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = Timer0B_CallBack;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /*start timer0*/
    TIMER0_Cmd(TMR_UNIT,Tim0_ChannelA,Enable);
    TIMER0_Cmd(TMR_UNIT,Tim0_ChannelB,Enable);

    while(1)
    {
        /* Read counter register of channelB*/
        u16cnt = TIMER0_GetCntReg(TMR_UNIT,Tim0_ChannelB);
        u16cmp = TIMER0_GetCntReg(TMR_UNIT,Tim0_ChannelB);

        /* Read counter register of channel A, need stop counter function for asynchronous mode*/
        TIMER0_Cmd(TMR_UNIT,Tim0_ChannelA,Disable);
        u16cnt = TIMER0_GetCntReg(TMR_UNIT,Tim0_ChannelA);
        u16cmp = TIMER0_GetCntReg(TMR_UNIT,Tim0_ChannelA);
        TIMER0_Cmd(TMR_UNIT,Tim0_ChannelA,Enable);

        u32tmp = 0xFFFFF;
        while(u32tmp--);
    }

}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
