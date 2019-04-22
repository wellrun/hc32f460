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
 ** \brief This sample demonstrates how to use EMB compare function for Timer4.
 **
 **   - 2018-12-11  1.0  Hongjh first version for Device Driver Library of EMB.
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
#define SW2_PORT                        PortD
#define SW2_PIN                         Pin03

/* LED(D23: red color) Port/Pin definition */
#define LED_PORT                        PortE
#define LED_PIN                         Pin06

/* LED operation */
#define LED_ON()                        PORT_SetBits(LED_PORT, LED_PIN)
#define LED_OFF()                       PORT_ResetBits(LED_PORT, LED_PIN)

/* Timer4 CNT */
#define TIMER4_UNIT                     M4_TMR41
#define TIMER4_CNT_CYCLE_VAL            (50000u)       /* Timer4 counter cycle value */

/* Timer4 OCO */
#define TIMER4_OCO_HIGH_CH              Timer4OcoOuh   /* only Timer4OcoOuh  Timer4OcoOvh  Timer4OcoOwh */

/* Define port and pin for Timer4Pwm */
#define TIMER4_PWM_H_PORT               PortE          /* TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13 */
#define TIMER4_PWM_H_PIN                Pin09

/* EMB unit */
#define EMB_UNIT                        M4_EMB2

/* EMB unit interrupt number */
#define EMB_INT_NUM                     INT_EMB_GR1

#define DAC_Enable

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void LedInit(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static bool m_bEmbBraking = false;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Callback function of external interrupt ch.0
 **
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief Callback function of EMB interrupt
 **
 ******************************************************************************/
static void EmbCallBack(void)
{
    if(true == EMB_GetStatus(EMB_UNIT, EMBFlagCmp))
    {
        LED_ON();

        EMB_SwBrake(EMB_UNIT, true);  /* Software brake Enable, still shunt down PWM after Clear Port In Brake */

        EMB_ClrStatus(EMB_UNIT, EMBCmpFlagClr);  /* Clear CMP In Brake */

        m_bEmbBraking = true;
    }
}

/**
 *******************************************************************************
 ** \brief Initliaze LED.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void LedInit(void)
{
    stc_port_init_t stcPortInit;

    LED_OFF();

    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;

    /* LED Port/Pin initialization */
    PORT_Init(LED_PORT, LED_PIN, &stcPortInit);
}

/**
 ******************************************************************************
 ** \brief  Initialize the system clock
 **
 ** \param  None
 **
 ** \return None
 ******************************************************************************/
static void ClkInit(void)
{
    en_clk_sys_source_t       enSysClkSrc;
    stc_clk_sysclk_cfg_t      stcSysClkCfg;
    stc_clk_mpll_cfg_t        stcMpllCfg;

    MEM_ZERO_STRUCT(enSysClkSrc);
    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv = ClkSysclkDiv1;   /* Max 168MHz */
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;  /* Max 84MHz */
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;  /* Max 168MHz */
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;  /* Max 84MHz */
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;  /* Max 60MHz */
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;  /* Max 42MHz */
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;  /* Max 84MHz */
    CLK_SysClkConfig(&stcSysClkCfg);

    CLK_HrcCmd(Enable);      /* Enable HRC */

    /* MPLL config. */
    stcMpllCfg.pllmDiv = 2;   /* HRC 16M / 2 */
    stcMpllCfg.plln = 42;     /* 8M*42 = 336M */
    stcMpllCfg.PllpDiv = 2;   /* MLLP = 168M */
    stcMpllCfg.PllqDiv = 2;   /* MLLQ = 168M */
    stcMpllCfg.PllrDiv = 2;   /* MLLR = 168M */
    CLK_SetPllSource(ClkPllSrcHRC);
    CLK_MpllConfig(&stcMpllCfg);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_4);
    EFM_Lock();

    /* Enable MPLL. */
    CLK_MpllCmd(Enable);

    /* Wait MPLL ready. */
    while(Set != CLK_GetFlagStatus(ClkFlagMPLLRdy))
    {
    }

    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);
}

/**
 ******************************************************************************
 ** \brief Timer4 PWM function configuration
 **
 ** \param  None
 **
 ** \return None
 ******************************************************************************/
void Timer4PwmConfig(void)
{
    en_timer4_pwm_ch_t enPwmCh;
    stc_timer4_cnt_init_t stcCntInit;
    stc_timer4_oco_init_t stcOcoInit;
    stc_timer4_pwm_init_t stcPwmInit;
    stc_timer4_emb_init_t stcEmbInit;
    stc_oco_high_ch_compare_mode_t stcHighChCmpMode;

    /* Clear structures */
    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(stcOcoInit);
    MEM_ZERO_STRUCT(stcPwmInit);
    MEM_ZERO_STRUCT(stcEmbInit);
    MEM_ZERO_STRUCT(stcHighChCmpMode);

    /* Enable peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM41 | PWC_FCG2_PERIPH_TIM42 | PWC_FCG2_PERIPH_TIM43, Enable);

    /* Timer4 CNT : Initialize CNT configuration structure */
    stcCntInit.enBufferCmd = Disable;
    stcCntInit.enClk = Timer4CntPclk;
    stcCntInit.enClkDiv = Timer4CntPclkDiv16;  /* CNT clock divide */
    stcCntInit.u16Cycle = TIMER4_CNT_CYCLE_VAL;
    stcCntInit.enCntMode = Timer4CntSawtoothWave;
    stcCntInit.enZeroIntCmd = Disable;
    stcCntInit.enPeakIntCmd = Disable;
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;
    TIMER4_CNT_Init(TIMER4_UNIT, &stcCntInit); /* Initialize CNT */

    /* Timer4 OCO : Initialize OCO configuration structure */
    stcOcoInit.enOccrBufMode = OccrBufDisable;
    stcOcoInit.enOcmrBufMode = OcmrBufDisable;
    stcOcoInit.enPortLevel = OcPortLevelLow;
    stcOcoInit.enOcoIntCmd = Disable;
    TIMER4_OCO_Init(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, &stcOcoInit); /* Initialize OCO high channel */

    if ((Timer4OcoOuh == TIMER4_OCO_HIGH_CH) || (Timer4OcoOvh == TIMER4_OCO_HIGH_CH) || (Timer4OcoOwh == TIMER4_OCO_HIGH_CH))
    {
        /* ocmr[15:0] = 0x0FFF */
        stcHighChCmpMode.enCntZeroMatchOpState = OcoOpOutputReverse;
        stcHighChCmpMode.enCntZeroNotMatchOpState = OcoOpOutputHold;
        stcHighChCmpMode.enCntUpCntMatchOpState = OcoOpOutputReverse;
        stcHighChCmpMode.enCntPeakMatchOpState = OcoOpOutputReverse;
        stcHighChCmpMode.enCntPeakNotMatchOpState = OcoOpOutputHold;
        stcHighChCmpMode.enCntDownCntMatchOpState = OcoOpOutputReverse;

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
    TIMER4_OCO_WriteOccr(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, TIMER4_CNT_CYCLE_VAL/2);

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

    /* Timer4 EMB: Initialize EMB configuration structure */
    stcEmbInit.enPwmHold = EmbChangePwm;
    stcEmbInit.enEmbState = EmbTrigPwmOutputLowLevel;
    TIMER4_EMB_Init(TIMER4_UNIT, &stcEmbInit); /* Initialize timer4 pwm */
}

 /*******************************************************************************
 ** \brief CMP init function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void M4_CMP_Init(void)
{
    stc_cmp_init_t         stcCmpConfig;
    stc_irq_regi_conf_t    stcIrqRegiConf;
    stc_port_init_t        stcPortInit;
    stc_cmp_input_sel_t    stcCmpInput;
    stc_cmp_dac_init_t     stcDacInitCfg;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcCmpConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcCmpInput);
    MEM_ZERO_STRUCT(stcDacInitCfg);

    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_CMP, Enable);
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_DAC, Enable);

#ifdef DAC_Enable
    /* Set DAC */
    //DAC1 for CMP1(INM3); DAC2 for CMP2(INM3); DAC1 for CMP3(INM3) / DAC2 for CMP3(INM4)
    stcDacInitCfg.u8DacData = 0x80;
    stcDacInitCfg.enCmpDacEN = Enable;
    CMP_DAC_Init(CmpDac1, &stcDacInitCfg);
    CMP_DAC_Init(CmpDac2, &stcDacInitCfg);
#endif


//CMP1
#if 1
    stcPortInit.enPinMode = Pin_Mode_Ana;
    /* Set PA0 as CMP1_INP1 input */
    PORT_Init(PortA, Pin00, &stcPortInit);
    /* Set PC3 as CMP1_INM2 input */
    PORT_Init(PortC, Pin03, &stcPortInit);

    /* Set PB12 as CH1  output */
    PORT_SetFunc(PortB, Pin12, Func_Vcout, Disable);

    /* mode set */
    stcCmpConfig.enCmpIntEN = Disable;            //intrrupt disable
    stcCmpConfig.enCmpInvEn = Disable;            //CMP INV sel for output
    stcCmpConfig.enCmpOutputEn = Enable;          //CMP Output enable
    stcCmpConfig.enCmpVcoutOutputEn = Enable;     //CMP output result enable
    stcCmpConfig.enEdgeSel = CmpRisingEdge;       //Fall edge active brake
    stcCmpConfig.enFltClkDiv = CmpFltPclk3Div32;  //PCLK3/32
    CMP_Init(M4_CMP1, &stcCmpConfig);

    //gpio set for cmp
    stcCmpInput.enInp4Sel = CmpInp4None;
    stcCmpInput.enInpSel = CmpInp1;
    stcCmpInput.enInmSel = CmpInm2;
  #ifdef DAC_Enable
     stcCmpInput.enInmSel = CmpInm3;
  #endif
    CMP_InputSel(M4_CMP1, &stcCmpInput);

  #if 0
    stcIrqRegiConf.enIntSrc = INT_ACMP1;          //Select CMP
    stcIrqRegiConf.enIRQn = Int112_IRQn;          //Register CMP
    stcIrqRegiConf.pfnCallback = ACMP1_Callback;  //Callback function
    enIrqRegistration(&stcIrqRegiConf);           //Registration IRQ


    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);  //Clear pending
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);//Set priority
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);       //Enable NVIC
  #endif

    CMP_Cmd(M4_CMP1,Enable);    //Enable CMP1
#endif


//CMP2
#if 0
    stcPortInit.enPinMode = Pin_Mode_Ana;
    /* Set PA6 as CMP2_INP3 input */
    PORT_Init(PortA, Pin06, &stcPortInit);
    /* Set PC4 as CMP2_INN2 input */
    PORT_Init(PortC, Pin04, &stcPortInit);

    /* Set PB13 as CH2  output */
    PORT_SetFunc(PortB, Pin13, Func_Vcout, Disable);

    /* mode set */
    stcCmpConfig.enCmpIntEN = Disable;            //intrrupt disable
    stcCmpConfig.enCmpInvEn = Disable;            //CMP INV sel for output
    stcCmpConfig.enCmpOutputEn = Enable;          //CMP Output enable
    stcCmpConfig.enCmpVcoutOutputEn = Enable;     //CMP output result enable
    stcCmpConfig.enEdgeSel = CmpRisingEdge;       //Fall edge active brake
    stcCmpConfig.enFltClkDiv = CmpFltPclk3Div32;  //PCLK3/32
    CMP_Init(M4_CMP2, &stcCmpConfig);

    stcCmpInput.enInpSel = CmpInp3;
    stcCmpInput.enInmSel = CmpInm2;
  #ifdef DAC_Enable
     stcCmpInput.enInmSel = CmpInm3;
  #endif
    CMP_InputSel(M4_CMP2, &stcCmpInput);

    CMP_Cmd(M4_CMP2,Enable);    //Enable CMP2
#endif


//CMP3
#if 0
    stcPortInit.enPinMode = Pin_Mode_Ana;
    /* Set PB1 as CMP3_INP2 input */
    PORT_Init(PortB, Pin01, &stcPortInit);
    /* Set PC5 as CMP3_INM2 input */
    PORT_Init(PortC, Pin05, &stcPortInit);

    /* Set PB14 as CH3  output */
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPinSubFunc = Enable;
    PORT_Init(PortB,  Pin14, &stcPortInit);
    PORT_SetFunc(PortB, Pin14, Func_Vcout, Disable);

    /* mode set */
    stcCmpConfig.enCmpIntEN = Disable;            //intrrupt disable
    stcCmpConfig.enCmpInvEn = Disable;            //CMP INV sel for output
    stcCmpConfig.enCmpOutputEn = Enable;          //CMP Output enable
    stcCmpConfig.enCmpVcoutOutputEn = Enable;     //CMP output result enable
    stcCmpConfig.enEdgeSel = CmpRisingEdge;       //Fall edge active brake
    stcCmpConfig.enFltClkDiv = CmpFltPclk3Div32;  //PCLK3/32
    CMP_Init(M4_CMP3, &stcCmpConfig);

    stcCmpInput.enInpSel = CmpInp2;
    stcCmpInput.enInmSel = CmpInm2;
  #ifdef DAC_Enable
    stcCmpInput.enInmSel = CmpInm3;
  #endif
    CMP_InputSel(M4_CMP3, &stcCmpInput);

    CMP_Cmd(M4_CMP3, Enable);   //Enable CMP3
#endif

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
    stc_irq_regi_conf_t   stcIrqRegiConf;
    stc_emb_ctrl_timer4_t stcEMBConfigCR;

    /* Clear structures */
    MEM_ZERO_STRUCT(stcEMBConfigCR);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Initialize Clock */
    ClkInit();

    /* Enable peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_EMB, Enable);

    /* Initialize LED port */
    LedInit();

    /* Configure Timer4 PWM function */
    Timer4PwmConfig();

    /* Configure Compare function */
    M4_CMP_Init();

    /* Configure EMB function */
    stcEMBConfigCR.bEnCmp1Brake = true;
    EMB_Config_CR_Timer4(EMB_UNIT, &stcEMBConfigCR);
    EMB_ClrStatus(EMB_UNIT, EMBCmpFlagClr);  /* Clear Port In Brake */
    EMB_ConfigIrq(EMB_UNIT, CMPBrkIrq, true);

    /* Configure EMB interrupt */
    stcIrqRegiConf.enIRQn = Int000_IRQn;                    /* Register INT_TMR61_GUDF Int to Vect.No.002 */
    stcIrqRegiConf.enIntSrc = EMB_INT_NUM;                  /* Select Event interrupt function */
    stcIrqRegiConf.pfnCallback = EmbCallBack;               /* Callback function */
    enIrqRegistration(&stcIrqRegiConf);                     /* Registration IRQ */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);            /* Clear Pending */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_01); /* Set priority */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);                  /* Enable NVIC */

    /* Clear && Start CNT */
    TIMER4_CNT_ClearCountVal(TIMER4_UNIT);
    TIMER4_CNT_Start(TIMER4_UNIT);

    while(1)
    {
        if (true == m_bEmbBraking)
        {
            /* Add brake process code */

            Ddl_Delay1ms(3000);  /* only for demo using */

            EMB_SwBrake(EMB_UNIT, false); /* Disable software brake, Enable PWM output */

            LED_OFF();

            m_bEmbBraking = false;
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
