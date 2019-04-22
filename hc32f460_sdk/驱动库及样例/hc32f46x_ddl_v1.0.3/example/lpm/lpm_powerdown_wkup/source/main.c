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
#define PVD_IRQn                        Int141_IRQn

/* LED0 Port/Pin definition */
#define LED0_PORT                       PortE
#define LED0_PIN                        Pin06

/* LED0 off definition */
#define LED0_OFF()                      PORT_ResetBits(LED0_PORT, LED0_PIN)

/* LED0 toggle definition */
#define LED0_TOGGLE()                   PORT_Toggle(LED0_PORT, LED0_PIN)

#define DLY_MS                          1000

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

    /* LED0 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief  Port init.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Port_Init(void)
{
    PORT_Unlock();

    M4_PORT->PCRA0_f.INTE = 1;

    PORT_Lock();
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
    stc_pwc_pwr_mode_cfg_t  stcPwcPwrMdCfg;
    stc_pwc_wkup_edge_cfg_t stcPwcWkupEdgCfg;

    MEM_ZERO_STRUCT(stcPwcPwrMdCfg);
    MEM_ZERO_STRUCT(stcPwcWkupEdgCfg);

    Led_Init();
    Port_Init();

    Ddl_Delay1ms(DLY_MS);

    /* SW2 */
    while(0 != PORT_GetBit(PortD, Pin03));

    /* Config power down mode. */
    stcPwcPwrMdCfg.enPwrDownMd = PowerDownMd1;
    stcPwcPwrMdCfg.enRLdo = Enable;
    stcPwcPwrMdCfg.enIoRetain = IoPwrDownRetain;
    stcPwcPwrMdCfg.enRetSram = Disable;
    stcPwcPwrMdCfg.enVHrc = Disable;
    stcPwcPwrMdCfg.enVPll = Disable;
    stcPwcPwrMdCfg.enDynVol =  Voltage09;
    stcPwcPwrMdCfg.enDrvAbility = Ulowspeed;
    stcPwcPwrMdCfg.enPwrDWkupTm = Vcap0047;

    PWC_PowerModeCfg(&stcPwcPwrMdCfg);

/********************Config WKUP 0_0 event*************************************

    PWC_ClearWakeup0Flag(PWC_PTWK0_WKUPFLAG);

    stcPwcWkupEdgCfg.enPtwk0Edge = EdgeRising;
    PWC_PdWkupEdgeCfg(&stcPwcWkupEdgCfg);

    PWC_PdWakeup0Cmd(PWC_PDWKEN0_WKUP00,Enable);

*******************************************************************************/

    /* NMI wake_up. */
    PWC_ClearWakeup0Flag(PWC_NMI_WKUPFLAG);
    stcPwcWkupEdgCfg.enNmiEdge = EdgeFalling;
    PWC_PdWkupEdgeCfg(&stcPwcWkupEdgCfg);
    PWC_PdWakeup2Cmd(PWC_PDWKEN2_NMI,Enable);

    LED0_TOGGLE();
    Ddl_Delay1ms(DLY_MS);
    LED0_TOGGLE();
    Ddl_Delay1ms(DLY_MS);
    LED0_TOGGLE();
    Ddl_Delay1ms(DLY_MS);
    LED0_TOGGLE();
    Ddl_Delay1ms(DLY_MS);

    PWC_EnterPowerDownMd();

    while(1);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
