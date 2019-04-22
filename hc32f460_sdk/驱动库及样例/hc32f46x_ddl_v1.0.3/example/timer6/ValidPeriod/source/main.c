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
 ** \brief This sample demonstrates how to use Timer6.
 **
 **   - 2018-11-30  1.0  husj first version for Device Driver Library of Timer6.
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

//#define SCMA_ValidPeriod  1

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

void Timer6_CallBack(void)
{
    static uint8_t i;

    if( 0 == i)
    {
        Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareC, 0x3000);
       #ifdef SCMA_ValidPeriod
        Timer6_SetSpecialCmpValue(M4_TMR61, Timer6SpclCompC, 0x3000);
       #endif

        i = 1;
    }
    else
    {
        Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareC, 0x6000);
      #ifdef SCMA_ValidPeriod
        Timer6_SetSpecialCmpValue(M4_TMR61, Timer6SpclCompC, 0x6000);
      #endif
        i = 0;
    }

    M4_TMR61->STFLR_f.UDFF = 0;
}

void ADC1A_CallBack(void)
{

    LED0_TOGGLE();
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

    MEM_ZERO_STRUCT(enSysClkSrc);
    MEM_ZERO_STRUCT(stcSysClkCfg);
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

    CLK_HrcCmd(Enable);       //Enable HRC

    /* MPLL config. */
    stcMpllCfg.pllmDiv = 2;   //HRC 16M / 2
    stcMpllCfg.plln =42;      //8M*42 = 336M
    stcMpllCfg.PllpDiv = 2;   //MLLP = 168M
    stcMpllCfg.PllqDiv = 2;   //MLLQ = 168M
    stcMpllCfg.PllrDiv = 2;   //MLLR = 168M
    CLK_SetPllSource(ClkPllSrcHRC);
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

void Config_Adc(void)
{
    en_result_t             enIrqRegResult;
    stc_adc_init_t          stcAdcInit;
    stc_adc_ch_cfg_t        stcBaseCfg;
    stc_irq_regi_conf_t     stcAdcIrqCfg;
    stc_adc_trg_cfg_t       stcAdcTrigCfg;
    stc_port_init_t         stcPortInit;
    uint8_t                 u8Adc1SampTime[3];
    uint8_t                 u8Adc2SampTime[3];

    MEM_ZERO_STRUCT(stcAdcInit);
    MEM_ZERO_STRUCT(stcBaseCfg);
    MEM_ZERO_STRUCT(stcAdcIrqCfg);
    MEM_ZERO_STRUCT(stcAdcTrigCfg);
    MEM_ZERO_STRUCT(stcPortInit);

    memset(u8Adc1SampTime, 32, sizeof(u8Adc1SampTime));
    memset(u8Adc2SampTime, 32, sizeof(u8Adc2SampTime));

    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1, Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS, Enable);

    stcPortInit.enPinMode = Pin_Mode_Ana;
    stcPortInit.enPullUp = Disable;
    PORT_Init(PortA, Pin00, &stcPortInit);
    PORT_Init(PortA, Pin01, &stcPortInit);
    PORT_Init(PortA, Pin02, &stcPortInit);

    /* Init ADC1 */
    stcAdcInit.enResolution   = AdcResolution_12Bit;
    stcAdcInit.enDataAlign    = AdcDataAlign_Right;
    stcAdcInit.enAutoClear    = AdcClren_Disable;
    stcAdcInit.enScanMode     = AdcMode_SAOnce;
    ADC_Init(M4_ADC1 ,&stcAdcInit);

    stcBaseCfg.u32Channel     = ADC1_CH0 | ADC1_CH1 | ADC1_CH2;
    stcBaseCfg.u8Sequence     = AdcSequence_A;
    stcBaseCfg.pu8SampTime    = u8Adc1SampTime;
    ADC_ConfigAdcChannel(M4_ADC1, &stcBaseCfg);

#ifdef SCMA_ValidPeriod
    stcAdcTrigCfg.u8Sequence  = AdcSequence_A;
    stcAdcTrigCfg.enTrgSel    = AdcTrgsel_TRGX0;
    stcAdcTrigCfg.enInTrg0    = EVT_TMR61_SCMA;
    ADC_ConfigTriggerSrc(M4_ADC1, &stcAdcTrigCfg);
    ADC_TriggerSrcCmd(M4_ADC1,AdcSequence_A, Enable);
#else
    stcAdcTrigCfg.u8Sequence  = AdcSequence_A;
    stcAdcTrigCfg.enTrgSel    = AdcTrgsel_TRGX0;
    stcAdcTrigCfg.enInTrg0    = EVT_TMR61_GUDF;
    ADC_ConfigTriggerSrc(M4_ADC1, &stcAdcTrigCfg);
    ADC_TriggerSrcCmd(M4_ADC1,AdcSequence_A, Enable);
#endif

    /* Enable ADC1 sequence A interrupt */
    ADC_SeqITCmd(M4_ADC1, AdcSequence_A, Enable);

    /* Config ADC1 interrupt */
    stcAdcIrqCfg.enIntSrc    = INT_ADC1_EOCA;
    stcAdcIrqCfg.enIRQn      = Int004_IRQn;        ///< [Int000_IRQn, Int031_IRQn] [Int116_IRQn, Int121_IRQn] [Int142_IRQn]
    stcAdcIrqCfg.pfnCallback = ADC1A_CallBack;
    enIrqRegResult = enIrqRegistration(&stcAdcIrqCfg);

    if (Ok != enIrqRegResult)
    {
        while (1u);
    }

    NVIC_ClearPendingIRQ(stcAdcIrqCfg.enIRQn);
    NVIC_SetPriority(stcAdcIrqCfg.enIRQn, DDL_IRQ_PRIORITY_03);
    NVIC_EnableIRQ(stcAdcIrqCfg.enIRQn);
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
    uint16_t                         u16Period;
    uint16_t                         u16Compare;
    stc_timer6_basecnt_cfg_t         stcTIM6BaseCntCfg;
    stc_timer6_port_output_cfg_t     stcTIM6PWMxCfg;
    stc_timer6_gcmp_buf_cfg_t        stcGCMPBufCfg;
    stc_irq_regi_conf_t              stcIrqRegiConf;
    stc_timer6_deadtime_cfg_t        stcDeadTimeCfg;
    stc_timer6_validper_cfg_t        stcValidPerCfg;
    stc_timer6_spcl_buf_cfg_t        stcSpclBufCfg;
    stc_port_init_t                  stcPortInit;

    MEM_ZERO_STRUCT(stcTIM6BaseCntCfg);
    MEM_ZERO_STRUCT(stcTIM6PWMxCfg);
    MEM_ZERO_STRUCT(stcGCMPBufCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcDeadTimeCfg);
    MEM_ZERO_STRUCT(stcValidPerCfg);
    MEM_ZERO_STRUCT(stcSpclBufCfg);
    MEM_ZERO_STRUCT(stcPortInit);

    SysClkIni();

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM61, Enable);

    /*initiallize LED port*/
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPullUp = Enable;
    /* LED0 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

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
    //Timer6_SetGeneralBuf(M4_TMR61, Timer6PWMB, &stcGCMPBufCfg);        //GCMBR buffer transfer set


    u16Compare = 0x3000;
    Timer6_SetSpecialCmpValue(M4_TMR61, Timer6SpclCompA, u16Compare);
    Timer6_SetSpecialCmpValue(M4_TMR61, Timer6SpclCompC, u16Compare);

    stcSpclBufCfg.bEnSpclTransBuf    = true;
    stcSpclBufCfg.enSpclBufTransType = Timer6SpclSingleBuf;
    stcSpclBufCfg.enSpclBufOptType   = Timer6SplcOptUnderFlow;
    Timer6_SetSpecialBuf(M4_TMR61, Timer6SpclCompA, &stcSpclBufCfg);


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
    //Timer6_SetDeadTimeValue(M4_TMR61, Timer6DeadTimDwnAR, 3360);  // Set dead time value (down count)

    stcDeadTimeCfg.bEnDeadtime     = true;  //Enable Hardware DeadTime
    stcDeadTimeCfg.bEnDtBufUp      = false; //Disable buffer transfer
    stcDeadTimeCfg.bEnDtBufDwn     = false; //Disable buffer transfer
    stcDeadTimeCfg.bEnDtEqualUpDwn = true;  //Make the down count dead time value equal to the up count dead time setting
    Timer6_ConfigDeadTime(M4_TMR61, &stcDeadTimeCfg);        // Hardware dead time function config

#ifdef SCMA_ValidPeriod
    /* Enable timer61 under flow interrupt */
    stcValidPerCfg.enValidCntNum = Timer6PeriodCnts1;  //Valid period: Enable every other one period
    stcValidPerCfg.enValidCdtEn = Timer6PeriodCnteMax; //Count condition: voer flow point of Triangular wave mode
    stcValidPerCfg.bPeriodSCMA  = true;
    Timer6_SetValidPeriod(M4_TMR61, &stcValidPerCfg);

    Timer6_ConfigIrq(M4_TMR61, Timer6INTENSAU, true);

#else

    /* Enable timer61 under flow interrupt */
    stcValidPerCfg.enValidCntNum = Timer6PeriodCnts1;  //Valid period: Enable every other one period
    stcValidPerCfg.enValidCdtEn = Timer6PeriodCnteMax; //Count condition: voer flow point of Triangular wave mode
    Timer6_SetValidPeriod(M4_TMR61, &stcValidPerCfg);

    //Timer6_ConfigIrq(M4_TMR61, Timer6INTENUDF, true);
#endif

    /*config interrupt*/
    /* Enable timer61 under flow interrupt */
    Timer6_ConfigIrq(M4_TMR61, Timer6INTENUDF, true);

    stcIrqRegiConf.enIRQn = Int002_IRQn;                    //Register INT_TMR61_GUDF Int to Vect.No.002
    stcIrqRegiConf.enIntSrc = INT_TMR61_GUDF;               //Select Event interrupt function
    stcIrqRegiConf.pfnCallback = Timer6_CallBack;           //Callback function
    enIrqRegistration(&stcIrqRegiConf);                     //Registration IRQ

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);            //Clear Pending
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_03);//Set priority
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);                   //Enable NVIC

    Config_Adc();

    /*start timer6*/
    Timer6_StartCount(M4_TMR61);

    while(1)
    {

    }

}




/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
