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
 ** \brief The example of Rtc alarm clock function
 **
 **   - 2018-11-26  1.0  Yangjp First version for Device Driver Library of Rtc.
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
static uint8_t u8AlarmIntFlag = 0, u8AlarmCnt = 0;

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
 ** \brief Rtc alarm clock callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void RtcAlarm_IrqCallback(void)
{
    u8AlarmCnt = 10u;
    u8AlarmIntFlag = 1u;
    RTC_ClearAlarmFlag();
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
    stcRtcDateTimeCfg.u8Hour = 11;
    stcRtcDateTimeCfg.u8Minute = 59;
    stcRtcDateTimeCfg.u8Second = 55;
    stcRtcDateTimeCfg.enAmPm = RtcHour12Pm;
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
 ** \brief Rtc display date and time information
 **
 ** \param [in] enFormat                Date and time data format
 ** \arg RtcDataFormatDec               Decimal format
 ** \arg RtcDataFormatBcd               BCD format
 **
 ** \param [in] pTitle                  Display title of information
 **
 ** \param [in] pRtcDateTime            Pointer to RTC current date time struct
 ** \arg See the struct #stc_rtc_date_time_t
 **
 ** \retval None
 **
 ******************************************************************************/
void Rtc_DisplayDataTimeInfo(en_rtc_data_format_t enFormat, char *pTitle,
                             stc_rtc_date_time_t *pRtcDateTime)
{
    if (RtcDataFormatBcd == enFormat)
    {
        printf("%s 20%02x/%02x/%02x %02x:%02x:%02x ", pTitle, pRtcDateTime->u8Year,
               pRtcDateTime->u8Month, pRtcDateTime->u8Day, pRtcDateTime->u8Hour,
               pRtcDateTime->u8Minute, pRtcDateTime->u8Second);
    }
    else
    {
        printf("%s 20%02d/%02d/%02d %02d:%02d:%02d ", pTitle, pRtcDateTime->u8Year,
               pRtcDateTime->u8Month, pRtcDateTime->u8Day, pRtcDateTime->u8Hour,
               pRtcDateTime->u8Minute, pRtcDateTime->u8Second);
    }

    if (pRtcDateTime->enAmPm == RtcHour12Am)
    {
        printf("Am ");
    }
    else
    {
        printf("Pm ");
    }
    Rtc_DisplayWeekday(pRtcDateTime->u8Weekday);
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
    stc_rtc_alarm_time_t stcRtcAlarmCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcRtcInit);
    MEM_ZERO_STRUCT(stcRtcAlarmCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Reset rtc counter */
    if (RTC_DeInit() == ErrorTimeout)
    {
        printf("reset rtc failed!\r\n");
        return;
    }

    /* Configuration rtc structure */
    stcRtcInit.enClkSource = RtcClkXtal32;
    stcRtcInit.enPeriodInt = RtcPeriodIntOneSec;
    stcRtcInit.enTimeFormat = RtcTimeFormat12Hour;
    stcRtcInit.enCompenWay = RtcOutputCompenDistributed;
    stcRtcInit.enCompenEn = Disable;
    stcRtcInit.u16CompenVal = 0;
    RTC_Init(&stcRtcInit);

    /* Configuration alarm clock time: Monday to friday£¬PM 12:00 */
    stcRtcAlarmCfg.u8Hour = 0x12;
    stcRtcAlarmCfg.u8Minute = 0x00;
    stcRtcAlarmCfg.u8Weekday = RtcAlarmWeekdayMonday    | RtcAlarmWeekdayTuesday  |
                               RtcAlarmWeekdayWednesday | RtcAlarmWeekdayThursday |
                               RtcAlarmWeekdayFriday;
    stcRtcAlarmCfg.enAmPm = RtcHour12Am;
    RTC_SetAlarmTime(RtcDataFormatBcd, &stcRtcAlarmCfg);
    RTC_AlarmCmd(Enable);

    /* Configure interrupt of rtc period */
    stcIrqRegiConf.enIntSrc = INT_RTC_PRD;
    stcIrqRegiConf.enIRQn = Int006_IRQn;
    stcIrqRegiConf.pfnCallback = RtcPeriod_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Configuration interrupt of rtc alarm clock */
    stcIrqRegiConf.enIntSrc = INT_RTC_ALM;
    stcIrqRegiConf.enIRQn = Int007_IRQn;
    stcIrqRegiConf.pfnCallback = RtcAlarm_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Enable period and alarm interrupt */
    RTC_IrqCmd(RtcIrqPeriod, Enable);
    RTC_IrqCmd(RtcIrqAlarm, Enable);

    /* Startup rtc count */
    RTC_Cmd(Enable);
}

/**
 *******************************************************************************
 ** \brief  main function for Rtc alarm clock function
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

    /* Configure XTAL32 clock */
    Xtal32_ClockConfig();
    /* Debug uart init */
    Ddl_UartInit();
    /* Configure Rtc */
    Rtc_Config();
    /* wait for rtc running */
    Ddl_Delay1ms(1);
    /* Update time after RTC startup */
    Rtc_CalendarConfig();

    while (1)
    {
        if (1u == u8SecIntFlag)
        {
            u8SecIntFlag = 0;
            /* Print alarm information */
            if ((1u == u8AlarmIntFlag) && (u8AlarmCnt > 0))
            {
                /* Alarm LED flicker */
                LED0_TOGGLE();
                u8AlarmCnt--;
                if (0 == u8AlarmCnt)
                {
                    u8AlarmIntFlag = 0u;
                    LED0_OFF();
                }
                /* Get and print alarm time */
                if (RTC_GetDateTime(RtcDataFormatBcd, &stcCurrDateTime) != Ok)
                {
                    printf("get alarm clock time failed!\r\n");
                }
                else
                {
                    Rtc_DisplayDataTimeInfo(RtcDataFormatBcd, "alarm", &stcCurrDateTime);
                }

            }
            /* Print current date and time information */
            else
            {
                /* Get current time */
                if (RTC_GetDateTime(RtcDataFormatDec, &stcCurrDateTime) != Ok)
                {
                    printf("get calendar failed!\r\n");
                }
                else
                {
                    Rtc_DisplayDataTimeInfo(RtcDataFormatDec, "normal", &stcCurrDateTime);
                }
            }
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
