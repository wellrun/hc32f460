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
 ** \brief power voltage detected interrupt sample
 **
 **   - 2018-11-06  1.0  Chengy First version for Device Driver Library of LPM.
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
#define PORT_IRQn               Int003_IRQn

#define KEY1_PORT               PortD
#define KEY1_PIN                Pin04

/* LED0 Port/Pin definition */
#define  LED0_PORT              PortE
#define  LED0_PIN               Pin06

/* LED1 Port/Pin definition */
#define  LED1_PORT              PortA
#define  LED1_PIN               Pin07

/* LED0~1 off definition */
#define LED0_OFF()              PORT_ResetBits(LED0_PORT, LED0_PIN)
#define LED1_OFF()              PORT_ResetBits(LED1_PORT, LED1_PIN)

/* LED0~1 toggle definition */
#define  LED0_TOGGLE()          PORT_Toggle(LED0_PORT, LED0_PIN)
#define  LED1_TOGGLE()          PORT_Toggle(LED1_PORT, LED1_PIN)

#define  DLY_MS                 1000

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
uint8_t u8IntCnt = 0;
/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Led init.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Led_Init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;

    LED0_OFF();
    LED1_OFF();

    /* LED0 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* LED1 Port/Pin initialization */
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief  SW4 interrupt callback function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt04_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh04))
    {
        /* NVIC recover after wakeup from stop mode. */
        enNvicRecover();
        u8IntCnt++;
        if(u8IntCnt >= 6)
        {
            u8IntCnt = 0;
        }
        EXINT_IrqFlgClr(ExtiCh04);
    }
}

/**
 *******************************************************************************
 ** \brief KEY1(SW4) init function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SW4_Init(void)
{
    stc_port_init_t stcPortInit;
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /**************************************************************************/
    /* External Int Ch.4                                                     */
    /**************************************************************************/
    /* Set PD04 as External Int Ch.4 input */
    stcPortInit.enExInt = Enable;
    PORT_Init(KEY1_PORT, KEY1_PIN, &stcPortInit);

    stcExtiConfig.enExitCh = ExtiCh04;
    /* Filter setting */
    stcExtiConfig.enFilterEn = Disable;
    /* Rising edge */
    stcExtiConfig.enExtiLvl = ExIntFallingEdge;
    EXINT_Init(&stcExtiConfig);

    /* Select External Int Ch.4 */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ4;
    /* Register External Int to Vect.No.007 */
    stcIrqRegiConf.enIRQn = Int007_IRQn;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = ExtInt04_Callback;
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
 ** \brief Stop mode config function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void StopMode_Init()
{
    stc_pwc_stop_mode_cfg_t stcPwcStopCfg;

    MEM_ZERO_STRUCT(stcPwcStopCfg);

    /* Config stop mode. */
    stcPwcStopCfg.enStpDrvAbi = StopHighspeed;
    stcPwcStopCfg.enStopClk = ClkFix;
    stcPwcStopCfg.enStopFlash = Wait;
    PWC_StopModeCfg(&stcPwcStopCfg);

    /* Set wake up source EIRQ4. */
    enIntWakeupEnable(Extint4WU);
}
/**
 *******************************************************************************
 ** \brief Power down mode config function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void PowerDownMode_Init()
{
    stc_pwc_pwr_mode_cfg_t  stcPwcPwrMdCfg;
    stc_pwc_wkup_edge_cfg_t stcPwcWkupEdgCfg;

    MEM_ZERO_STRUCT(stcPwcPwrMdCfg);
    MEM_ZERO_STRUCT(stcPwcWkupEdgCfg);


    /* Config power down mode. */
    stcPwcPwrMdCfg.enPwrDownMd = PowerDownMd3;
    stcPwcPwrMdCfg.enRLdo = Enable;
    stcPwcPwrMdCfg.enIoRetain = IoPwrDownRetain;
    stcPwcPwrMdCfg.enRetSram = Disable;
    stcPwcPwrMdCfg.enVHrc = Disable;
    stcPwcPwrMdCfg.enVPll = Disable;
    stcPwcPwrMdCfg.enDynVol =  Voltage09;
    stcPwcPwrMdCfg.enDrvAbility = Ulowspeed;
    stcPwcPwrMdCfg.enPwrDWkupTm = Vcap0047;

    PWC_PowerModeCfg(&stcPwcPwrMdCfg);

    /* Clear flag. */
    PWC_ClearWakeup0Flag(PWC_NMI_WKUPFLAG);
    /* Falling edge. */
    stcPwcWkupEdgCfg.enNmiEdge = EdgeFalling;
    PWC_PdWkupEdgeCfg(&stcPwcWkupEdgCfg);
    /* Set wake up source NMI. */
    PWC_PdWakeup2Cmd(PWC_PDWKEN2_NMI,Enable);
}
/**
 *******************************************************************************
 ** \brief  Main function of switch system clock source to MPLL project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    Led_Init();
    SW4_Init();
    StopMode_Init();
    PowerDownMode_Init();

    LED0_TOGGLE();
    Ddl_Delay1ms(DLY_MS);
    LED0_TOGGLE();
    Ddl_Delay1ms(DLY_MS);
    LED0_TOGGLE();
    Ddl_Delay1ms(DLY_MS);
    LED0_TOGGLE();
    Ddl_Delay1ms(DLY_MS);

    while(1)
    {
        if(1u == u8IntCnt)
        {
            PWC_EnterSleepMd();
        }
        else if(2u == u8IntCnt)
        {
            LED0_TOGGLE();
            Ddl_Delay1ms(DLY_MS);
        }
        else if(3u == u8IntCnt)
        {
            /* NVIC backup and disable before entry from stop mode.*/
            enNvicBackup();
            PWC_EnterStopMd();
        }
        else if(4u == u8IntCnt)
        {
            LED1_TOGGLE();
            Ddl_Delay1ms(DLY_MS);
        }
        else if(5u == u8IntCnt)
        {
            PWC_EnterPowerDownMd();
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
