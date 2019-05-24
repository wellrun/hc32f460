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
 ** \brief The example of Rtc low power function
 **
 **   - 2018-11-27  1.0  Yangjp First version for Device Driver Library of Rtc.
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

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
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
 ** \brief Configure Rtc calendar function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Rtc_CalendarConfig(void)
{
    stc_rtc_date_time_t stcRtcDateTimeCfg;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcRtcDateTimeCfg);

    /* calendar configuration */
    stcRtcDateTimeCfg.u8Year = 18;
    stcRtcDateTimeCfg.u8Month = 10;
    stcRtcDateTimeCfg.u8Day = 10;
    stcRtcDateTimeCfg.u8Weekday = RtcWeekdayWednesday;
    stcRtcDateTimeCfg.u8Hour = 23;
    stcRtcDateTimeCfg.u8Minute = 59;
    stcRtcDateTimeCfg.u8Second = 55;
    if (RTC_SetDateTime(RtcDataFormatDec, &stcRtcDateTimeCfg, Enable, Enable) != Ok)
    {
        printf("write calendar failed!\r\n");
    }
}

/**
 *******************************************************************************
 ** \brief Rtc display weekday
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Rtc_DisplayWeekday(uint8_t u8Weekday)
{
    switch (u8Weekday)
    {
        case RtcWeekdaySunday:
            printf("Sunday\r\n");
            break;
        case RtcWeekdayMonday:
            printf("Monday\r\n");
            break;
        case RtcWeekdayTuesday:
            printf("Tuesday\r\n");
            break;
        case RtcWeekdayWednesday:
            printf("Wednesday\r\n");
            break;
        case RtcWeekdayThursday:
            printf("Thursday\r\n");
            break;
        case RtcWeekdayFriday:
            printf("Friday\r\n");
            break;
        case RtcWeekdaySaturday:
            printf("Saturday\r\n");
            break;
        default:
            break;
    }
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

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcRtcInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Reset rtc counter */
    if (RTC_DeInit() == ErrorTimeout)
    {
        printf("reset rtc failed!\r\n");
        return;
    }

    /* Configuration rtc structure */
    stcRtcInit.enClkSource = RtcClkLrc;
    stcRtcInit.enPeriodInt = RtcPeriodIntOneMin;
    stcRtcInit.enTimeFormat = RtcTimeFormat24Hour;
    stcRtcInit.enCompenWay = RtcOutputCompenDistributed;
    stcRtcInit.enCompenEn = Disable;
    stcRtcInit.u16CompenVal = 0;
    RTC_Init(&stcRtcInit);

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
 ** \brief  main function for Rtc low power function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_port_init_t stcPortInit;
    stc_rtc_date_time_t stcCurrDateTime;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcCurrDateTime);

    /* LED0 Port/Pin initialization */
    LED0_OFF();
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* Debug uart init */
    Ddl_UartInit();
    /* Waiting for LCR to stabilize */
    Ddl_Delay1ms(1000);
    /* Configure Rtc */
    Rtc_Config();
    /* wait for rtc running */
    Ddl_Delay1ms(1);
    /* Update time after RTC startup */
    Rtc_CalendarConfig();

    /* Wait for RTC to work properly before switching to low power */
    if (RTC_LowPowerSwitch() != Ok)
    {
        printf("switch to low power failed!\r\n");
    }
    PWC_EnterSleepMd();
    /* Configure one seconds trigger interrupt */
    RTC_PeriodIntConfig(RtcPeriodIntOneSec);

    while (1)
    {
        if (1u == u8SecIntFlag)
        {
            u8SecIntFlag = 0;
            LED0_TOGGLE();
            /* Get current time */
            if (RTC_GetDateTime(RtcDataFormatBcd, &stcCurrDateTime) != Ok)
            {
                printf("get calendar failed!\r\n");
            }
            else
            {
                printf("20%02x/%02x/%02x %02x:%02x:%02x ", stcCurrDateTime.u8Year,
                       stcCurrDateTime.u8Month, stcCurrDateTime.u8Day,
                       stcCurrDateTime.u8Hour, stcCurrDateTime.u8Minute,
                       stcCurrDateTime.u8Second);
                Rtc_DisplayWeekday(stcCurrDateTime.u8Weekday);
            }
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
