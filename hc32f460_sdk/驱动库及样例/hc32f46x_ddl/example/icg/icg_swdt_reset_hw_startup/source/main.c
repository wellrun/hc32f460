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
 ** \brief The example of ICG SWDT Reset function
 **
 **   - 2018-10-23  1.0  Yangjp First version for Device Driver Library of ICG.
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

/* KEY0 Port/Pin definition */
#define KEY0_PORT                       PortD
#define KEY0_PIN                        Pin03

/* SWDT count cycle definition */
#define SWDT_COUNT_CYCLE                (16384u)

/* Reset source definition */
#define RESET_SWDT_TRIGGER              (0u)
#define RESET_OTHER_TRIGGER             (1u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t u8SysWorkSta;
static uint8_t u8ExIntFlag = 0;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief ExtInt3 callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt03_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh03))
    {
        u8ExIntFlag = 1u;
        EXINT_IrqFlgClr(ExtiCh03);
    }
}

/**
 *******************************************************************************
 ** \brief KEY0(SW2) init function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Sw2_Init(void)
{
    stc_port_init_t stcPortInit;
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Set PD03 as External Int Ch.3 input */
    stcPortInit.enExInt = Enable;
    PORT_Init(KEY0_PORT, KEY0_PIN, &stcPortInit);

    stcExtiConfig.enExitCh = ExtiCh03;
    /* Filter setting */
    stcExtiConfig.enFilterEn = Enable;
    stcExtiConfig.enFltClk = Pclk3Div8;
    /* Both edge */
    stcExtiConfig.enExtiLvl = ExIntFallingEdge;
    EXINT_Init(&stcExtiConfig);

    /* Select External Int Ch.3 */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ3;
    /* Register External Int to Vect.No.007 */
    stcIrqRegiConf.enIRQn = Int007_IRQn;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = ExtInt03_Callback;
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
 ** \brief  main function for ICG SWDT Reset function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    /**
     ***************************************************************************
     ** Modify hc32f46x_icg.h file of defines
     ** #define ICG0_SWDT_HARDWARE_START        ICG_FUNCTION_ON
     **
     ** #define ICG0_SWDT_AUTS                  SWDT_AUTO_START_AFTER_RESET
     ** #define ICG0_SWDT_ITS                   SWDT_RESET_REQUEST
     ** #define ICG0_SWDT_PERI                  SWDT_COUNT_UNDERFLOW_CYCLE_16384
     ** #define ICG0_SWDT_CKS                   SWDT_COUNT_SWDTCLK_DIV1
     ** #define ICG0_SWDT_WDPT                  SWDT_0To25PCT
     ** #define ICG0_SWDT_SLTPOFF               SWDT_SPECIAL_MODE_COUNT_STOP
     **************************************************************************/
    uint16_t u16CmpVal;
    stc_port_init_t stcPortInit;
    stc_rmu_rstcause_t stcRmuRstCause;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcRmuRstCause);

    /* LED0 Port/Pin initialization */
    LED0_OFF();
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* Get RMU information */
    RMU_GetResetCause(&stcRmuRstCause);
    if (Set == stcRmuRstCause.enSwdt)
    {
        u8SysWorkSta = RESET_SWDT_TRIGGER;
        LED0_ON();
    }
    else
    {
        u8SysWorkSta = RESET_OTHER_TRIGGER;
    }
    RMU_ClrResetFlag();

    /* Key0 Port/Pin initialization */
    Sw2_Init();
    /* Wait for SWDT module to complete initial load */
    Ddl_Delay1ms(200);
    /* Count cycle=16384,range=0%-25% */
    u16CmpVal = SWDT_COUNT_CYCLE / 4;

    while (1)
    {
        if (1u == u8ExIntFlag)
        {
            u8ExIntFlag = 0u;
            u16CmpVal = SWDT_COUNT_CYCLE / 2;
        }

        if (SWDT_GetCountValue() < u16CmpVal)
        {
            SWDT_RefreshCounter();
            /* wait for the count value to update */
            Ddl_Delay1ms(10);
            if (RESET_OTHER_TRIGGER == u8SysWorkSta)
            {
                LED0_TOGGLE();
            }
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
