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
 ** \brief The example for SRAM function demonstration
 **
 **   - 2018-10-24  1.0  Zhangxl First version for sample of sram module.
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
#define  LED0_PORT          PortE
#define  LED0_PIN           Pin06

/* LED1 Port/Pin definition */
#define  LED1_PORT          PortA
#define  LED1_PIN           Pin07

/* LED2 Port/Pin definition */
#define  LED2_PORT          PortB
#define  LED2_PIN           Pin05

/* LED3 Port/Pin definition */
#define  LED3_PORT          PortB
#define  LED3_PIN           Pin09

/* LED0~3 toggle definition */
#define  LED0_TOGGLE()      PORT_Toggle(LED0_PORT, LED0_PIN)
#define  LED1_TOGGLE()      PORT_Toggle(LED1_PORT, LED1_PIN)
#define  LED2_TOGGLE()      PORT_Toggle(LED2_PORT, LED2_PIN)
#define  LED3_TOGGLE()      PORT_Toggle(LED3_PORT, LED3_PIN)

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
 ** \brief LED 0~3 init function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Led_Init(void)
{
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;

    /* LED0 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* LED1 Port/Pin initialization */
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);

    /* LED2 Port/Pin initialization */
    PORT_Init(LED2_PORT, LED2_PIN, &stcPortInit);

    /* LED3 Port/Pin initialization */
    PORT_Init(LED3_PORT, LED3_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief SRAM ECC/Parity error NMI IRQ handler
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SramErr_IrqHandler(void)
{
    /* SRAM3 1 bit ECC error */
    if (true == SRAM_GetStatus(Sram3EccErr1))
    {
        SRAM_ClrStatus(Sram3EccErr1);
    }

    /* SRAM3 2 bit ECC error */
    if (true == SRAM_GetStatus(Sram3EccErr2))
    {
        SRAM_ClrStatus(Sram3EccErr2);
        while (1)
        {
            LED0_TOGGLE();
            Ddl_Delay1ms(100);
        }
    }

    /* SRAM12 parity error */
    if (true == SRAM_GetStatus(Sram12ParityErr))
    {
        SRAM_ClrStatus(Sram12ParityErr);
    }
    /* High speed SRAM parity error */
    if (true == SRAM_GetStatus(SramHSParityErr))
    {
        SRAM_ClrStatus(SramHSParityErr);
    }
    /* Retention SRAM parity error */
    if (true == SRAM_GetStatus(SramRetParityErr))
    {
        SRAM_ClrStatus(SramRetParityErr);
    }
}

/**
 *******************************************************************************
 ** \brief  main function for SRAM function
 **
 ** \param  None
 **
 ** \return int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    uint32_t temp;
    stc_sram_config_t stcSramConfig;
    stc_nmi_config_t stcNmiConfig;

    //Ddl_UartInit();

    /* LED init */
    Led_Init();

    /* enable HS RAM source clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_RAMHS, Enable);

    /* enable RAM0 source clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_RAM0, Enable);

    /* enable ECCRAM source clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_ECCRAM, Enable);

    /* enable Retention RAM source clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_RetRAM, Enable);

    stcSramConfig.u8SramIdx = Sram12Idx | Sram3Idx | SramHsIdx | SramRetIdx;
    stcSramConfig.enSramRC = SramCycle5;
    stcSramConfig.enSramWC = SramCycle6;
    stcSramConfig.enSramEccMode = EccMode3;
    stcSramConfig.enSramEccOp = SramNmi;
    stcSramConfig.enSramPyOp = SramNmi;

    SRAM_Init(&stcSramConfig);

    stcNmiConfig.pfnNmiCallback = SramErr_IrqHandler;
    stcNmiConfig.u16NmiSrc = NmiSrcSramDE | NmiSrcSramPE;

    NMI_Init(&stcNmiConfig);
#if 1
    /* This section raw sample to generate the ECC error */
    M4_SRAMC->CKPR = 0x77;
    M4_SRAMC->CKCR = 0x03000000;
    SRAM3_BASE_ADDR = 0x1234567;
    temp = SRAM3_BASE_ADDR;

    M4_SRAMC->CKPR = 0x77;
    M4_SRAMC->CKCR = 0x00;
    SRAM3_BASE_ADDR = 0x1234562;

    M4_SRAMC->CKPR = 0x77;
    M4_SRAMC->CKCR = 0x03000000;
    temp = SRAM3_BASE_ADDR;
    if (temp == 0)
    {
        // avoid warning
    }
#endif
    while (1);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
