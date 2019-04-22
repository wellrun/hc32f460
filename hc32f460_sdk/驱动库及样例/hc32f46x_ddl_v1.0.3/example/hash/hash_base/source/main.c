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
 ** \brief HASH sample
 **
 **   - 2018-10-18  1.0  Wuze First version for Device Driver Library of
 **     Hash
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
#define HASH_MSG_DIGEST_SIZE        (32u)

#define TIMEOUT_10MS                (10u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void HashConfig(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t m_au8HashMsgDigest1[HASH_MSG_DIGEST_SIZE];
static uint8_t m_au8HashMsgDigest2[HASH_MSG_DIGEST_SIZE];
static uint8_t m_au8HashMsgDigest3[HASH_MSG_DIGEST_SIZE];

static uint8_t *m_su8SrcData1 = "abcde";

static uint8_t *m_su8SrcData2 = \
"abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\
abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\
abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\
abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\
abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

static uint8_t *m_su8SrcData3 = \
"abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\
!@#$%^&*( ()(*()*&&*^*& &^%^%^%dsf@#^%$#(*&^_)#<>?>?><>. ~;/':dd////ghe/\
abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\
?></.,;;://}{[]}{///sdfas\"dfas;;;;;d\"\"\"\"\"}{|\\][?><\":56789ABCDEFGH\
abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\
01234567890123456789";

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
    /* Config HASH. */
    HashConfig();

    /* Config UART for printing. Baud rate 115200. */
    Ddl_UartInit();

    /***************** Configuration end, application start **************/

    while (1u)
    {
        /* Use HASH. */
        HASH_Start(m_su8SrcData1, strlen((char *)m_su8SrcData1), \
                    m_au8HashMsgDigest1, TIMEOUT_10MS);
        printf("\nString \"abcde\" message digest:\n");
        for (uint8_t i = 0u; i < sizeof(m_au8HashMsgDigest1); i++)
        {
            printf("%.2x ", m_au8HashMsgDigest1[i]);
        }

        HASH_Start(m_su8SrcData2, strlen((char *)m_su8SrcData2), \
                    m_au8HashMsgDigest2, TIMEOUT_10MS);

        HASH_Start(m_su8SrcData3, strlen((char *)m_su8SrcData3), \
                    m_au8HashMsgDigest3, TIMEOUT_10MS);

        /* Main loop cycle is 500ms. */
        Ddl_Delay1ms(500u);
    }
}

/**
 *******************************************************************************
 ** \brief  HASH initial configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void HashConfig(void)
{
    /* 1. Enable HASH. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_HASH, Enable);

    /* 2. Initialize HASH. */
    HASH_Init();
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
