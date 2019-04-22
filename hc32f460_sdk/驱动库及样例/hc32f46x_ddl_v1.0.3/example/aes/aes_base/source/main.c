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
 ** \brief AES sample
 **
 **   - 2018-10-20  1.0  Wuze First version for Device Driver Library o
 **     AES
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

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void AesConfig(void);
static void AesFillData(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
const static uint8_t m_au8AesKey[AES_KEYLEN] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xCD, 0xEF,
                                                0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7};

static uint8_t m_au8SrcData[57u];

static uint8_t m_au8Plaintext[64u] = {0u};
static uint8_t m_au8Ciphertext[64u];

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Main function.
 **
 ** \param  None.
 **
 ** \retval int32_t return value, if needed.
 **
 ******************************************************************************/
int32_t main(void)
{
    uint32_t i;

    /* Config AES. */
    AesConfig();

    /* Config UART for printing. Baud rate 115200. */
    Ddl_UartInit();

    /***************** Configuration end, application start **************/

    AesFillData();

    while (1u)
    {
        /* AES encryption. */
        AES_Encrypt(m_au8Plaintext, sizeof(m_au8Plaintext), m_au8AesKey, m_au8Ciphertext);

        printf("\nAES encryption.");
        printf("\nPlaintext:\n");
        for (i = 0u; i < sizeof(m_au8Plaintext); i++)
        {
            printf("%.2x ", m_au8Plaintext[i]);
        }
        printf("\nCiphertext:\n");
        for (i = 0u; i < sizeof(m_au8Ciphertext); i++)
        {
            printf("%.2x ", m_au8Ciphertext[i]);
        }

        /* AES decryption. */
        AES_Decrypt(m_au8Ciphertext, sizeof(m_au8Ciphertext), m_au8AesKey, m_au8Plaintext);

        printf("\nCiphertext:\n");
        for (i = 0u; i < sizeof(m_au8Ciphertext); i++)
        {
            printf("%.2x ", m_au8Ciphertext[i]);
        }
        printf("\nPlaintext:\n");
        for (i = 0u; i < sizeof(m_au8Plaintext); i++)
        {
            printf("%.2x ", m_au8Plaintext[i]);
        }

        /* Main loop cycle 500ms. */
        Ddl_Delay1ms(500u);
    }
}

/**
 *******************************************************************************
 ** \brief  AES initial configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void AesConfig(void)
{
    /* 1. Enable AES. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AES, Enable);

    /* 2. Initialize AES. */
    AES_Init();
}

/**
 *******************************************************************************
 ** \brief  Fill data.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void AesFillData(void)
{
    uint32_t i;

    for (i = 0u; i < sizeof(m_au8SrcData); i++)
    {
        m_au8SrcData[i] = i + 1u;
    }

    memcpy(m_au8Plaintext, m_au8SrcData, sizeof(m_au8SrcData));
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
