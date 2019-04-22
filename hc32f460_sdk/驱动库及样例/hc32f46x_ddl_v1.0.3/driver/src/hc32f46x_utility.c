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
/** \file hc32f46x_utility.c
 **
 ** A detailed description is available at
 ** @link DdlUtilityGroup Ddl Utility description @endlink
 **
 **   - 2018-11-02  1.0  Zhangxl First version for Device Driver Library Utility.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32f46x_utility.h"

#if (DDL_UTILITY_ENABLE == DDL_ON)

/**
 *******************************************************************************
 ** \addtogroup DdlUtilityGroup
 ******************************************************************************/
//@{

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define UART_DEBUG_PRINTF

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
#ifdef UART_DEBUG_PRINTF
/**
 *******************************************************************************
 ** \brief Data printf via Uart Ch.3
 **
 ** \param [in]  u8Data                 Data to be sent
 **
 ******************************************************************************/
void DebugOutput(uint8_t u8Data)
{
    M4_USART3->DR = u8Data;
    while (0 == M4_USART3->SR_f.TC);
}

/**
 *******************************************************************************
 ** \brief Re-target putchar function
 **
 ******************************************************************************/
int32_t fputc(int32_t ch, FILE *f)
{
    DebugOutput((uint8_t)ch);

    return (ch);
}

/**
 *******************************************************************************
 ** \brief Set synchronous clock mode baudrate
 **
 ** \param [in] u32Baudrate             Baudrate
 **
 ** \retval Ok                          Configure successfully.
 ** \retval ErrorInvalidParameter       USARTx is invalid
 **
 ******************************************************************************/
static en_result_t SetUartBaudrate(uint32_t u32Baudrate)
{
    uint32_t B;
    uint32_t C;
    uint32_t OVER8;
    float32_t DIV = 0.0;
    uint64_t u64Tmp = 0u;
    uint32_t DIV_Integer = 0u;
    uint32_t DIV_Fraction = 0xFFFFFFFFul;

    uint32_t u32PClk1 = 0u;
    uint32_t u32UartClk = 0u;

    u32PClk1 = SystemCoreClock / (1u << M4_SYSREG->CMU_SCFGR_f.PCLK1S);
    u32UartClk = u32PClk1 / (1u << (2 * M4_USART3->PR_f.PSC));

    B = u32Baudrate;
    C = u32UartClk;
    OVER8 = M4_USART3->CR1_f.OVER8;

    /* FBME = 0 Calculation formula */
    /* B = C / (8 * (2 - OVER8) * (DIV_Integer + 1)) */
    /* DIV_Integer = (C / (B * 8 * (2 - OVER8))) - 1 */
    DIV = (C / (B * 8.0 * (2.0 - OVER8))) - 1.0;
    DIV_Integer = (uint32_t)(DIV);

    if ((DIV < 0) || (DIV_Integer > 0xFFu))
    {
        DDL_ASSERT(false);
        return ErrorInvalidParameter;
    }

    if (DIV != DIV_Integer)
    {
        /* FBME = 1 Calculation formula */
        /* B = C * (128 + DIV_Fraction) / (8 * (2 - OVER8) * (DIV_Integer + 1) * 256) */
        /* DIV_Fraction = ((8 * (2 - OVER8) * (DIV_Integer + 1) * 256 * B) / C) - 128 */
        /* E = (C * (128 + DIV_Fraction) / (8 * (2 - OVER8) * (DIV_Integer + 1) * 256 * B)) - 1 */
        /* DIV_Fraction = (((2 - OVER8) * (DIV_Integer + 1) * 2048 * B) / C) - 128 */
        u64Tmp = (2 - OVER8) * (DIV_Integer + 1) * B;
        DIV_Fraction = 2048 * u64Tmp/C - 128;
    }
    else
    {
    }

    M4_USART3->CR1_f.FBME = (0xFFFFFFFFul == DIV_Fraction) ? 0u : 1u;
    M4_USART3->BRR_f.DIV_FRACTION = DIV_Fraction;
    M4_USART3->BRR_f.DIV_INTEGER = DIV_Integer;
    return Ok;
}


/**
 *******************************************************************************
 ** \brief Debug printf initialization function
 **
 ** \retval Ok                          Process successfully done
 **
 ******************************************************************************/
en_result_t Ddl_UartInit(void)
{
    en_result_t enRet = Ok;
    volatile uint8_t u8ReloadVal = 0;

    /* unlock */
    M4_PORT->PWPR = 0xA501;
    /* usart3_tx gpio  PE5 */
    M4_PORT->PFSRE5_f.FSEL  = 32;
    /* lock */
    M4_PORT->PWPR = 0xA500;
    /* enable usart3 */
    M4_MSTP->FCG1_f.USART3 = 0;
    /* usart3 init */

    M4_USART3->CR1_f.ML = 0;    // LSB
    M4_USART3->CR1_f.MS = 0;    // UART mode
    M4_USART3->CR1_f.OVER8 = 1; // 8bit sampling mode
    M4_USART3->CR1_f.M = 0;     // 8 bit data length
    M4_USART3->CR1_f.PCE = 0;   // no parity bit

    /* baudrate set */
    if( Ok != SetUartBaudrate(115200))
    {
        return Error;
    }

    /* 1 stop bit, single uart mode */
    M4_USART3->CR2 = 0;

    /* CTS disable, Smart Card mode disable */
    M4_USART3->CR3 = 0;

    M4_USART3->CR1_f.TE = 1;    // TX enable

    return enRet;
}
#endif /* UART_DEBUG_PRINTF_ENABLE */

/**
 *******************************************************************************
 ** \brief Delay function, delay 1ms approximately
 **
 ** \param [in]  u32Cnt                 ms
 **
 ** \retval Ok                          Process successfully done
 **
 ******************************************************************************/
en_result_t Ddl_Delay1ms(uint32_t u32Cnt)
{
    en_result_t enRet = Ok;
    volatile uint32_t i = 0;
    uint32_t u32Cyc = 0;

    u32Cyc = SystemCoreClock;
    u32Cyc = u32Cyc / 10000;
    while (u32Cnt-- > 0)
    {
        i = u32Cyc;
        while (i-- > 0)
        {
            ;
        }
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief ddl assert error handle function
 **
 ** \param [in]  file                   Point to the current assert the wrong file
 ** \param [in]  line                   Point line assert the wrong file in the current
 **
 ******************************************************************************/
#ifdef __DEBUG
__WEAKDEF void Ddl_AssertHandler(uint8_t *file, uint32_t line)
{
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
    while (1);
}
#endif /* __DEBUG */

//@} // DdlUtilityGroup

#endif /* DDL_UTILITY_ENABLE */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
