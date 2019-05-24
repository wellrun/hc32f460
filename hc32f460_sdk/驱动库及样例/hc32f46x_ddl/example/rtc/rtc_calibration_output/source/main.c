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
 ** \brief The example of Rtc calibration output function
 **
 **   - 2018-11-28  1.0  Yangjp First version for Device Driver Library of Rtc.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"
#include <math.h>

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

/* RTC 1Hz output Port/Pin definition */
#define RTC_ONEHZ_OUTPUT_PORT           PortC
#define RTC_ONEHZ_OUTPUT_PIN            Pin13

/* XTAL32 measure window lower and upper definition */
#define XTAL32_MEASURE_LOWER            20000
#define XTAL32_MEASURE_UPPER            40000

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static const float EPSINON = 0.000001;
static uint8_t u8SecIntFlag = 0;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Rtc period callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void RtcPeriod_IrqCallback(void)
{
    u8SecIntFlag = 1u;
}

/**
 *******************************************************************************
 ** \brief Xtal32 clock config
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Xtal32_ClockConfig(void)
{
    stc_clk_xtal32_cfg_t stcXtal32Cfg;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcXtal32Cfg);

    /* Stop xtal32 */
    CLK_Xtal32Cmd(Disable);
    Ddl_Delay1ms(100);
    /* Configuration xtal32 structure */
    stcXtal32Cfg.enFastStartup = Disable;
    stcXtal32Cfg.enDrv = ClkXtal32HighDrv;
    stcXtal32Cfg.enFilterMode = ClkXtal32FilterModeFull;
    CLK_Xtal32Config(&stcXtal32Cfg);
    /* Startup xtal32 */
    CLK_Xtal32Cmd(Enable);
    /* wait for xtal32 running */
    Ddl_Delay1ms(3000);
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
    stc_clk_sysclk_cfg_t    stcSysClkCfg;
    stc_clk_xtal_cfg_t      stcXtalCfg;

    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcXtalCfg);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv = ClkSysclkDiv1;   // Max 168MHz
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;  // Max 168MHz
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;  // Max 60MHz
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;  // Max 42MHz
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;  // Max 84MHz
    CLK_SysClkConfig(&stcSysClkCfg);

    /* Switch system clock source to XTAL. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* Wait XTAL ready. */
    while (Set != CLK_GetFlagStatus(ClkFlagXTALRdy));
    /* Switch system clock source to XTAL. */
    CLK_SetSysClkSource(ClkSysSrcXTAL);
}

/**
 *******************************************************************************
 ** \brief clock measure configuration
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Clock_MeasureConfig(void)
{
    stc_clk_fcm_cfg_t stcClkFcmCfg;
    stc_clk_fcm_window_cfg_t stcClkFcmWinCfg;
    stc_clk_fcm_measure_cfg_t stcClkFcmMeasureCfg;
    stc_clk_fcm_reference_cfg_t stcClkFcmReferCfg;
    stc_clk_fcm_interrupt_cfg_t stcClkFcmIntCfg;

    /* Configure structure initialization */
    MEM_ZERO_STRUCT(stcClkFcmCfg);
    MEM_ZERO_STRUCT(stcClkFcmWinCfg);
    MEM_ZERO_STRUCT(stcClkFcmMeasureCfg);
    MEM_ZERO_STRUCT(stcClkFcmReferCfg);
    MEM_ZERO_STRUCT(stcClkFcmIntCfg);

    /* Enable FCM clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_FCM, Enable);

    /* use xtal measure xtal32 */
    stcClkFcmWinCfg.windowLower = XTAL32_MEASURE_LOWER; /* zero error = 48828 */
    stcClkFcmWinCfg.windowUpper = XTAL32_MEASURE_UPPER;

    stcClkFcmMeasureCfg.enSrc = ClkFcmSrcXtal;
    stcClkFcmMeasureCfg.enSrcDiv = ClkFcmMeaDiv1;

    stcClkFcmReferCfg.enRefSel = ClkFcmInterRef;
    stcClkFcmReferCfg.enExtRef = Disable;
    stcClkFcmReferCfg.enIntRefSrc = ClkFcmSrcXtal32;
    stcClkFcmReferCfg.enIntRefDiv = ClkFcmIntrefDiv128;
    stcClkFcmReferCfg.enEdge = ClkFcmEdgeRising;
    stcClkFcmReferCfg.enFilterClk = ClkFcmFilterClkNone;

    stcClkFcmCfg.pstcFcmIntCfg = &stcClkFcmIntCfg;
    stcClkFcmCfg.pstcFcmMeaCfg = &stcClkFcmMeasureCfg;
    stcClkFcmCfg.pstcFcmRefCfg = &stcClkFcmReferCfg;
    stcClkFcmCfg.pstcFcmWindowCfg = &stcClkFcmWinCfg;
    /* enable clock measure */
    CLK_FcmConfig(&stcClkFcmCfg);
}

/**
 *******************************************************************************
 ** \brief Get rtc compensation value
 **
 ** \param [in]  None
 **
 ** \retval uint16_t                            Rtc compensation value
 **
 ******************************************************************************/
uint16_t Rtc_GetCompenValue(void)
{
    float clkMeasureVal;
    uint16_t integerVal = 0, decimalsVal = 0;
    uint16_t clkCompenVal = 0;
    stc_clk_freq_t stcClkFreq;

    MEM_ZERO_STRUCT(stcClkFreq);
    /* start measure */
    CLK_FcmCmd(Enable);
    do
    {
        /* counter overflow or trigger frequency abnormal */
        if ((Set == CLK_GetFcmFlag(ClkFcmFlagOvf)) ||
            (Set == CLK_GetFcmFlag(ClkFcmFlagErrf)))
        {
            CLK_FcmCmd(Disable);
            CLK_ClearFcmFlag(ClkFcmFlagOvf);
            CLK_ClearFcmFlag(ClkFcmFlagErrf);
            return 0;
        }
    } while (Reset == CLK_GetFcmFlag(ClkFcmFlagMendf));

    /* Get measure result */
    CLK_GetClockFreq(&stcClkFreq);
    clkMeasureVal = CLK_GetFcmCounter();
    clkMeasureVal = stcClkFreq.sysclkFreq * 128 / clkMeasureVal;
    /* stop measure */
    CLK_FcmCmd(Disable);
    CLK_ClearFcmFlag(ClkFcmFlagMendf);

    /* calculate clock compensation value */
    if (!((clkMeasureVal >= -EPSINON) && (clkMeasureVal <= EPSINON)))
    {
        clkMeasureVal = (clkMeasureVal - XTAL32_VALUE) / XTAL32_VALUE * 1000000;
        clkMeasureVal = clkMeasureVal * XTAL32_VALUE / 1000000;

        if (clkMeasureVal < -EPSINON)    /* negative */
        {
            clkMeasureVal = fabs(clkMeasureVal);
            integerVal = (~((uint32_t)clkMeasureVal) + 1) & 0x0F;
            /* Magnify one thousand times */
            clkMeasureVal = (clkMeasureVal - (uint32_t)clkMeasureVal) * 1000;
            decimalsVal = (((~((uint32_t)clkMeasureVal)) & 0x3E0) >> 5) + 1;
        }
        else                            /* positive */
        {
            clkMeasureVal += 1.0f;
            integerVal = ((uint32_t)clkMeasureVal) & 0x0F;
            /* Magnify one thousand times */
            clkMeasureVal = (clkMeasureVal - (uint32_t)clkMeasureVal) * 1000;
            decimalsVal = ((uint32_t)clkMeasureVal & 0x3E0) >> 5;
        }
    }
    clkCompenVal = ((integerVal << 5) | decimalsVal) & 0x1FF;

    return clkCompenVal;
}

/**
 *******************************************************************************
 ** \brief Configure Rtc peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Rtc_Config(void)
{
    stc_rtc_init_t stcRtcInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    uint16_t clkCompenVal = 0;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcRtcInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Configuration RTC output pin */
    PORT_SetFunc(RTC_ONEHZ_OUTPUT_PORT, RTC_ONEHZ_OUTPUT_PIN, Func_Rtcout, Disable);

    /* Reset rtc counter */
    if (RTC_DeInit() == ErrorTimeout)
    {
        printf("reset rtc failed!\r\n");
        return;
    }

    clkCompenVal = Rtc_GetCompenValue();
    /* Configuration rtc structure */
    stcRtcInit.enClkSource = RtcClkXtal32;
    stcRtcInit.enPeriodInt = RtcPeriodIntOneSec;
    stcRtcInit.enTimeFormat = RtcTimeFormat24Hour;
    stcRtcInit.enCompenWay = RtcOutputCompenUniform;
    stcRtcInit.enCompenEn = Enable;
    stcRtcInit.u16CompenVal = clkCompenVal;
    RTC_Init(&stcRtcInit);
    RTC_OneHzOutputCmd(Enable);

    /* Configure interrupt of rtc period */
    stcIrqRegiConf.enIntSrc = INT_RTC_PRD;
    stcIrqRegiConf.enIRQn = Int006_IRQn;
    stcIrqRegiConf.pfnCallback = RtcPeriod_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Enable period interrupt */
    RTC_IrqCmd(RtcIrqPeriod, Enable);
    /* Startup rtc count */
    RTC_Cmd(Enable);
}

/**
 *******************************************************************************
 ** \brief  main function for Rtc calibration output function
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

    /* Configure XTAL32 clock */
    Xtal32_ClockConfig();
    /* Configure system clock frequency */
    SystemClk_Init();

    /* LED0 Port/Pin initialization */
    LED0_OFF();
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* Debug uart init */
    Ddl_UartInit();
    /* Configure clock measure */
    Clock_MeasureConfig();
    /* Configure Rtc */
    Rtc_Config();

    while (1)
    {
        if (1u == u8SecIntFlag)
        {
            u8SecIntFlag = 0u;
            LED0_TOGGLE();
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
