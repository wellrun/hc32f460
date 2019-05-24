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
 ** \brief The example of Timera position overflow count function
 **
 **   - 2018-11-14  1.0  Yangjp First version for Device Driver Library of
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

/* TIMERA unit and clock definition */
#define TIMERA_UNIT1                    M4_TMRA1
#define TIMERA_UNIT1_CLOCK              PWC_FCG2_PERIPH_TIMA1
#define TIMERA_UNIT1_OVERFLOW_INT       INT_TMRA1_OVF

#define TIMERA_UNIT2                    M4_TMRA2
#define TIMERA_UNIT2_CLOCK              PWC_FCG2_PERIPH_TIMA2
#define TIMERA_UNIT2_OVERFLOW_INT       INT_TMRA2_OVF

/* TIMERA CLKA Port/Pin definition */
#define TIMERA_UNIT1_CLKA_PORT          PortE
#define TIMERA_UNIT1_CLKA_PIN           Pin09
#define TIMERA_UNIT1_CLKA_FUNC          Func_Tima0

/* TIMERA CLKB Port/Pin definition */
#define TIMERA_UNIT1_CLKB_PORT          PortE
#define TIMERA_UNIT1_CLKB_PIN           Pin11
#define TIMERA_UNIT1_CLKB_FUNC          Func_Tima0

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
 ** \brief Timera unit 1 count overflow callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void TimeraUnit1Over_IrqCallback(void)
{
    LED0_TOGGLE();
    TIMERA_ClearFlag(TIMERA_UNIT1, TimeraFlagOverflow);
}

/**
 *******************************************************************************
 ** \brief Timera unit 2 count overflow callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void TimeraUnit2Over_IrqCallback(void)
{
    LED1_TOGGLE();
    TIMERA_ClearFlag(TIMERA_UNIT2, TimeraFlagOverflow);
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
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_timera_orthogonal_coding_init_t stcTimeraCondingInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcTimeraCondingInit);

    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(TIMERA_UNIT1_CLOCK | TIMERA_UNIT2_CLOCK, Enable);

    /* Configuration TIMERA coding pin */
    PORT_SetFunc(TIMERA_UNIT1_CLKA_PORT, TIMERA_UNIT1_CLKA_PIN, TIMERA_UNIT1_CLKA_FUNC, Disable);
    PORT_SetFunc(TIMERA_UNIT1_CLKB_PORT, TIMERA_UNIT1_CLKB_PIN, TIMERA_UNIT1_CLKB_FUNC, Disable);

    /* Configuration timera unit 1 structure */
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = 1000;
    TIMERA_BaseInit(TIMERA_UNIT1, &stcTimeraInit);
    TIMERA_IrqCmd(TIMERA_UNIT1, TimeraIrqOverflow, Enable);

    /* Configure timera uint 2 structure */
    stcTimeraInit.u16PeriodVal = 6;
    TIMERA_BaseInit(TIMERA_UNIT2, &stcTimeraInit);
    TIMERA_IrqCmd(TIMERA_UNIT2, TimeraIrqOverflow, Enable);

    /* Configure coding count structure */
    stcTimeraCondingInit.enIncClkBHighAndClkARisingEn = Enable;
    stcTimeraCondingInit.enClkAFilterEn = Enable;
    stcTimeraCondingInit.enClkAClkDiv = TimeraFilterPclkDiv4;
    stcTimeraCondingInit.enClkBFilterEn = Enable;
    stcTimeraCondingInit.enClkBClkDiv = TimeraFilterPclkDiv4;
    TIMERA_OrthogonalCodingInit(TIMERA_UNIT1, &stcTimeraCondingInit);

    /* Configure position overflow count structure */
    MEM_ZERO_STRUCT(stcTimeraCondingInit);
    stcTimeraCondingInit.enIncAnotherUnitOverflowEn = Enable;
    TIMERA_OrthogonalCodingInit(TIMERA_UNIT2, &stcTimeraCondingInit);

    /* Configure count overflow interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT1_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = Int006_IRQn;
    stcIrqRegiConf.pfnCallback = TimeraUnit1Over_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Configure count overflow interrupt of timera unit 2 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT2_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = Int007_IRQn;
    stcIrqRegiConf.pfnCallback = TimeraUnit2Over_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Timera unit 1 and unit 2 startup */
    TIMERA_Cmd(TIMERA_UNIT1, Enable);
    TIMERA_Cmd(TIMERA_UNIT2, Enable);
}

/**
 *******************************************************************************
 ** \brief  main function for Timera position overflow count function
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
