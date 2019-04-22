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
 ** \brief This sample demonstrates how to use EMB for Timer6.
 **
 **   - 2018-12-07  1.0  husj first version for Device Driver Library of EMB.
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
/* KEY0 (SW2)*/
#define  SW2_PORT   PortD
#define  SW2_PIN    Pin03
/* KEY1 (SW4)*/
#define  SW4_PORT   PortD
#define  SW4_PIN    Pin04
/* KEY2 (SW3)*/
#define  SW3_PORT   PortD
#define  SW3_PIN    Pin05
/* KEY3 (SW5)*/
#define  SW5_PORT   PortD
#define  SW5_PIN    Pin06

/* LED0 Port/Pin definition */
#define  LED0_PORT        PortE
#define  LED0_PIN         Pin06

/* LED1 Port/Pin definition */
#define  LED1_PORT        PortD
#define  LED1_PIN         Pin07

/* LED2 Port/Pin definition */
#define  LED2_PORT        PortB
#define  LED2_PIN         Pin05

/* LED3 Port/Pin definition */
#define  LED3_PORT        PortB
#define  LED3_PIN         Pin09

/* LED0~1 toggle definition */
#define  LED0_TOGGLE()    PORT_Toggle(LED0_PORT, LED0_PIN)
#define  LED1_TOGGLE()    PORT_Toggle(LED1_PORT, LED1_PIN)
#define  LED2_TOGGLE()    PORT_Toggle(LED2_PORT, LED2_PIN)
#define  LED3_TOGGLE()    PORT_Toggle(LED3_PORT, LED3_PIN)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
uint16_t u16Flag_EMB1_Braking;

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

void Timer6_UnderFlow_CallBack(void)
{
    static uint8_t i;

    if( 0 == i)
    {
        Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareC, 0x3000);
        i = 1;
    }
    else
    {
        Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareC, 0x6000);
        i = 0;
    }
}

void EMB1_CallBack(void)
{
    if(true == EMB_GetStatus(M4_EMB1, EMBFlagOSCFail))
    {
        M4_PORT->PODRE_f.POUT06 = 1;

        EMB_SwBrake(M4_EMB1, true);  //Software brake Enable, still shunt down PWM after Clear Port In Brake

        //M4_SYSREG->CMU_XTALSTDSR_f.XTALSTDF = 0;
        __set_PRIMASK(1);
        NVIC_SystemReset();
        __set_PRIMASK(0);

        if(M4_SYSREG->CMU_XTALCR_f.XTALSTP == 0)
        {
            EMB_ClrStatus(M4_EMB1, EMBOSCFailFlagCLr);  //Clear Port In Brake

            u16Flag_EMB1_Braking = 1;
        }
    }
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
    en_clk_sys_source_t       enSysClkSrc;
    stc_clk_sysclk_cfg_t      stcSysClkCfg;
    stc_clk_mpll_cfg_t        stcMpllCfg;
    stc_clk_xtal_cfg_t        stcXtalCfg;
    stc_clk_xtal_stp_cfg_t    stcXtalStpCfg;

    MEM_ZERO_STRUCT(enSysClkSrc);
    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcXtalStpCfg);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv = ClkSysclkDiv1;   // Max 168MHz
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;  // Max 168MHz
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;  // Max 60MHz
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;  // Max 42MHz
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;  // Max 84MHz
    CLK_SysClkConfig(&stcSysClkCfg);
#if 1
    /* Switch system clock source to MPLL. */
    /* Use Xtal32 as MPLL source. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    stcXtalStpCfg.enDetect    = Enable;
    stcXtalStpCfg.enMode      = ClkXtalStpModeInt;
    stcXtalStpCfg.enModeInt   = Enable;
    stcXtalStpCfg.enModeReset = Disable;
    CLK_XtalStpConfig(&stcXtalStpCfg);
#endif
    /* MPLL config. */
    stcMpllCfg.pllmDiv = 1;   //XTAL 8M
    stcMpllCfg.plln =42;      //8M*42 = 336M
    stcMpllCfg.PllpDiv = 2;   //MLLP = 168M
    stcMpllCfg.PllqDiv = 2;   //MLLQ = 168M
    stcMpllCfg.PllrDiv = 2;   //MLLR = 168M
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

void Timer6_Config(void)
{
    uint16_t                         u16Period;
    uint16_t                         u16Compare;
    stc_timer6_basecnt_cfg_t         stcTIM6BaseCntCfg;
    stc_timer6_port_output_cfg_t     stcTIM6PWMxCfg;
    stc_timer6_gcmp_buf_cfg_t        stcGCMPBufCfg;
    stc_irq_regi_conf_t              stcIrqRegiConf;
    stc_timer6_deadtime_cfg_t        stcDeadTimeCfg;

    MEM_ZERO_STRUCT(stcTIM6BaseCntCfg);
    MEM_ZERO_STRUCT(stcTIM6PWMxCfg);
    MEM_ZERO_STRUCT(stcGCMPBufCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcDeadTimeCfg);

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM61, Enable);

    PORT_SetFunc(PortE, Pin09, Func_Tim6, Disable);    //Timer61 PWMA
    PORT_SetFunc(PortE, Pin08, Func_Tim6, Disable);    //Timer61 PWMB

    stcTIM6BaseCntCfg.enCntMode   = Timer6CntTriangularModeA;           //Triangular wave mode
    stcTIM6BaseCntCfg.enCntDir    = Timer6CntDirUp;                     //Counter counting up
    stcTIM6BaseCntCfg.enCntClkDiv = Timer6PclkDiv1;                     //Count clock: pclk
    Timer6_Init(M4_TMR61, &stcTIM6BaseCntCfg);                           //timer6 PWM frequency, count mode and clk config

    u16Period = 0x8340;
    Timer6_SetPeriod(M4_TMR61, Timer6PeriodA, u16Period);                //period set

    u16Compare = 0x3000;
    Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareA, u16Compare);  //Set General Compare RegisterA Value
    Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareC, u16Compare);  //Set General Compare RegisterC Value as buffer register of GCMAR

    /*PWMA/PWMB output buf config*/
    stcGCMPBufCfg.bEnGcmpTransBuf = true;
    stcGCMPBufCfg.enGcmpBufTransType = Timer6GcmpPrdSingleBuf;          //Single buffer transfer
    Timer6_SetGeneralBuf(M4_TMR61, Timer6PWMA, &stcGCMPBufCfg);          //GCMAR buffer transfer set
    Timer6_SetGeneralBuf(M4_TMR61, Timer6PWMB, &stcGCMPBufCfg);          //GCMBR buffer transfer set


    stcTIM6PWMxCfg.enPortMode = Timer6ModeCompareOutput;    //Compare output function
    stcTIM6PWMxCfg.bOutEn     = true;                       //Output enable
    stcTIM6PWMxCfg.enPerc     = Timer6PWMxCompareKeep;      //PWMA port output keep former level when CNTER value match PERAR
    stcTIM6PWMxCfg.enCmpc     = Timer6PWMxCompareInv;       //PWMA port output inverse level when CNTER value match with GCMAR
    stcTIM6PWMxCfg.enStaStp   = Timer6PWMxStateSelSS;       //PWMA output status is decide by STACA STPCA when CNTER start and stop
    stcTIM6PWMxCfg.enStaOut   = Timer6PWMxPortOutLow;       //PWMA port output set low level when CNTER start
    stcTIM6PWMxCfg.enStpOut   = Timer6PWMxPortOutLow;       //PWMA port output set low level when CNTER stop
    stcTIM6PWMxCfg.enDisVal   = Timer6PWMxDisValLow;
    Timer6_PortOutputConfig(M4_TMR61, Timer6PWMA, &stcTIM6PWMxCfg);

    stcTIM6PWMxCfg.enPortMode = Timer6ModeCompareOutput;    //Compare output function
    stcTIM6PWMxCfg.bOutEn     = true;                       //Output enable
    stcTIM6PWMxCfg.enPerc     = Timer6PWMxCompareKeep;      //PWMB port output keep former level when CNTER value match PERAR
    stcTIM6PWMxCfg.enCmpc     = Timer6PWMxCompareInv;       //PWMB port output inverse level when CNTER value match with GCMBR
    stcTIM6PWMxCfg.enStaStp   = Timer6PWMxStateSelSS;       //PWMB output status is decide by STACB STPCB when CNTER start and stop
    stcTIM6PWMxCfg.enStaOut   = Timer6PWMxPortOutHigh;      //PWMB port output set high level when CNTER start
    stcTIM6PWMxCfg.enStpOut   = Timer6PWMxPortOutLow;       //PWMB port output set low level when CNTER stop
    stcTIM6PWMxCfg.enDisVal   = Timer6PWMxDisValLow;
    Timer6_PortOutputConfig(M4_TMR61, Timer6PWMB, &stcTIM6PWMxCfg);

    Timer6_SetDeadTimeValue(M4_TMR61, Timer6DeadTimUpAR, 3360);     // Set dead time value (up count)

    stcDeadTimeCfg.bEnDeadtime     = true;  //Enable Hardware DeadTime
    stcDeadTimeCfg.bEnDtBufUp      = false; //Disable buffer transfer
    stcDeadTimeCfg.bEnDtBufDwn     = false; //Disable buffer transfer
    stcDeadTimeCfg.bEnDtEqualUpDwn = true;  //Make the down count dead time value equal to the up count dead time setting
    Timer6_ConfigDeadTime(M4_TMR61, &stcDeadTimeCfg);        // Hardware dead time function config

    /*config interrupt*/
    /* Enable timer61 under flow interrupt */
    Timer6_ConfigIrq(M4_TMR61, Timer6INTENUDF, true);

    stcIrqRegiConf.enIRQn = Int002_IRQn;                    //Register INT_TMR61_GUDF Int to Vect.No.002
    stcIrqRegiConf.enIntSrc = INT_TMR61_GUDF;               //Select Event interrupt function
    stcIrqRegiConf.pfnCallback = Timer6_UnderFlow_CallBack; //Callback function
    enIrqRegistration(&stcIrqRegiConf);                     //Registration IRQ

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);            //Clear Pending
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_03);//Set priority
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);                   //Enable NVIC
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
    stc_emb_ctrl_timer6_t   stcEMBConfigCR;
    stc_irq_regi_conf_t     stcIrqRegiConf;
    stc_port_init_t         stcPortInit;

    MEM_ZERO_STRUCT(stcEMBConfigCR);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    SysClkIni();

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_EMB, Enable);

    /*initiallize LED port*/
    stcPortInit.enPinMode = Pin_Mode_Out;
    //stcPortInit.enPullUp = Enable;
    /* LED0 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);       //LED0

    Timer6_Config();

    stcEMBConfigCR.bEnOSCFailBrake = true;
    EMB_Config_CR_Timer6(&stcEMBConfigCR);

    EMB_ClrStatus(M4_EMB1, EMBOSCFailFlagCLr);  //Clear OSC Brake flag

    EMB_ConfigIrq(M4_EMB1, OSCFailBrkIrq, true);

    stcIrqRegiConf.enIRQn = Int005_IRQn;                    //Register INT_TMR61_GUDF Int to Vect.No.002
    stcIrqRegiConf.enIntSrc = INT_EMB_GR0;                  //Select Event interrupt function
    stcIrqRegiConf.pfnCallback = EMB1_CallBack;             //Callback function
    enIrqRegistration(&stcIrqRegiConf);                     //Registration IRQ

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);            //Clear Pending
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_01);//Set priority
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);                   //Enable NVIC


    /*start timer6*/
    Timer6_StartCount(M4_TMR61);

    while(1)
    {
        if(1 == u16Flag_EMB1_Braking)
        {
            //Add brake process code

            Ddl_Delay1ms(3000);  //only for demo using

            EMB_SwBrake(M4_EMB1, false); //Disable software brake, Enable PWM output

            M4_PORT->PODRE_f.POUT06 = 0;
            u16Flag_EMB1_Braking = 0;
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
