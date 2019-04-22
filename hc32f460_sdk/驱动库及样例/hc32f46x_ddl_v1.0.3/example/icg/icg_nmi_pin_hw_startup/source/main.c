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
 ** \brief The example of ICG NMI function
 **
 **   - 2018-10-22  1.0  Yangjp First version for Device Driver Library of ICG.
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

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief NMI interrupt callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Nmi_IrqCallback(void)
{
    /* NMI Pin */
    if (Set == NMI_IrqFlgGet(NmiSrcNmi))
    {
        NMI_IrqFlgClr(NmiSrcNmi);
        LED0_TOGGLE();
    }
}

/**
 *******************************************************************************
 ** \brief  main function for ICG NMI function
 **
 ** \param [in] None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    /**
     ***************************************************************************
     ** Modify hc32f46x_icg.h file of defines
     ** #define ICG1_NMI_HARDWARE_START         ICG_FUNCTION_ON
     **
     ** #define ICG1_NMI_SMPCLK                 NMI_PIN_FILTER_PCLK3_DIV1
     ** #define ICG1_NMI_TRG                    NMI_PIN_TRIGGER_EDGE_FALLING
     ** #define ICG1_NMI_IMR                    NMI_PIN_IRQ_ENABLE
     ** #define ICG1_NMI_NFEN                   NMI_DIGITAL_FILTER_DISABLE
     ** #define ICG1_NMI_ICGENA                 NMI_PIN_ICG_FUNCTION_ENABLE
     **************************************************************************/
    stc_port_init_t stcPortInit;
    stc_nmi_config_t stcNmiConfig;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcNmiConfig);

    /* LED0 Port/Pin initialization */
    LED0_OFF();
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* NMI(PB11) Int Callback function */
    stcNmiConfig.pfnNmiCallback = Nmi_IrqCallback;
    stcNmiConfig.u16NmiSrc = NmiSrcNmi;
    NMI_Init(&stcNmiConfig);

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
