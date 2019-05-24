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
 ** \brief Event port sample
 **
 **   - 2018-12-07  1.0     zhangxl First version for Device Driver Library of
 **                         event port.
 **
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
/* uncomment this line if wants to print information to Terminal I/O window */
//#define  __PRINT_TO_TERMINAL

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
 ** \brief  Main function of event port project
 **
 ** \param  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_event_port_init_t stcEPConfig;

    /* Set PB5 as Event Port 2.5 */
    PORT_SetFunc(PortB, (Pin05 | Pin06), Func_Evnpt, Disable);

    /* Set PD3, PD4 as Event Port 4.3, Event Port 4.4 */
    PORT_SetFunc(PortD, (Pin03 | Pin04), Func_Evnpt, Disable);

    /* Enable Event port operation clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS, Enable);

    /* Set Event Port 4.3 falling edge detect enable */
    MEM_ZERO_STRUCT(stcEPConfig);
    stcEPConfig.enFallingDetect = Enable;
    EVENTPORT_Init(EventPort4, EventPin03, &stcEPConfig);

    /* Set Event Port 4.4 rising edge detect enable */
    MEM_ZERO_STRUCT(stcEPConfig);
    stcEPConfig.enRisingDetect = Enable;
    EVENTPORT_Init(EventPort4, EventPin04, &stcEPConfig);

    /* Set Event Port 4 event as the trigger source for Event Port 2*/
    EVENTPORT_SetTriggerSrc(EventPort2, EVT_EVENT_PORT4);

    /* Set Event Port 2.5 as output function */
    MEM_ZERO_STRUCT(stcEPConfig);
    stcEPConfig.enDirection = EventPortOut;
    stcEPConfig.enSet = Enable;
    stcEPConfig.enReset = Enable;
    EVENTPORT_Init(EventPort2, EventPin05, &stcEPConfig);

    /* Set Event Port 2.6 as input function */
    MEM_ZERO_STRUCT(stcEPConfig);
    stcEPConfig.enDirection = EventPortIn;
    EVENTPORT_Init(EventPort2, EventPin06, &stcEPConfig);

    while(1)
    {
        /* Event Port2.06 is set after trigger */
        if (Set == EVENTPORT_GetBit(EventPort2, EventPin06))
        {
#ifdef __PRINT_TO_TERMINAL
            printf("EventPort2_06 is set.\n");
#endif
            break;
        }
    }
    /* de-init event port if necessary */
//    EVENTPORT_DeInit();
    while (1);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
