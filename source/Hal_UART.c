/*******************************************************************************
 * Include
 ******************************************************************************/
#include "Hal_UART.h"
#include "MKL46Z4.h"
#include "string.h"
#include "clock.h"
#include "Queue.h"

/*******************************************************************************
 * define
 ******************************************************************************/
/* value of osr */
#define OSR_MIN 4U
#define OSR_MAX 31U
#define UART_RESET_VALUE     0x00u
/* value of Data register is fail */
#define UART_DATA_FAIL       0x00u
/* value of SBR max*/
#define SBR_MAX_VALUE        (0x1FFF -1)/* max 13 bit */

/*******************************************************************************
 * Prototype
 ******************************************************************************/
/*!
 * @briefP: set parity mode
 * @paramP: base. UART peripheral base address.
 * @paramP: parityMode. UART parity mode.
 */
static void UART_SetParityMode(UART0_Type *base, Uart_ParityMode_t parityMode);

/*!
 * @brief: Sets the UART instance baud rate.
 * @param: base UART peripheral base address.
 * @param: baudrate UART baudrate to be set.
 * @param: sourceClock UART clock source freqency in HZ.
 * @retval: true is baudrate is true
            flase is baudrate isn't support
 */
static bool UART_SetBaudRate(UART0_Type *base, uint32_t u32baudrate, uint32_t u32sourceClock);
/*!
 * @brief: Sets UART stop bit count..
 * @param: base UART peripheral base address.
 * @param: stopBitCount  Number of stop bits, 1 stop bit (default) or 2 stop bits
 * @retval : none
 */
static void UART_SetStopBitCount(UART0_Type *base, Uart_StopBitCount_t stopBitCount);

/*******************************************************************************
 * Code
 ******************************************************************************/
/* inititial UART*/
bool UART_Init(UART0_Type *base, const Uart_Config_t *uartConfig)
{
    assert(NULL != uartConfig);

    /* enable clock for uart 0 */
    __HAL_SIM_UART0_CLK_ENABLE();
    QUEUE_Init();
    /* Disable UART TX RX before setting. */
    UART_GetChar(base);
    base->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
    /*set 8bit tranmit and receiver */
    base->C1 &= ~(UART_C1_M_MASK);
    UART_SetBaudRate(base, uartConfig->u32Baudrate,uartConfig->u32SourceClock);
    UART_SetParityMode(base, uartConfig->ParityMode);
    UART_SetStopBitCount(base, uartConfig->StopBitCount);

    UART_EnableInterrupts(base);

    /* Enable TX/RX base on configure structure. */
    base->C2 |= UART_C2_TE_MASK | UART_C2_RE_MASK;
    NVIC_EnableIRQ(UART0_IRQn);
    return true;
}

/* set baudrate */
static bool UART_SetBaudRate(UART0_Type *base, uint32_t u32baudrate, uint32_t u32sourceClock)
{
    bool retVal = true;
    uint8_t oldCtrl;
    uint8_t u8osrCount;
    uint8_t temp;
    uint16_t sbr = 0u;
    uint32_t baudDiff = 0u;

    assert(u32baudrate);
    assert(u32sourceClock);

    for (u8osrCount = OSR_MIN; u8osrCount < OSR_MAX; u8osrCount++)
    {
        sbr = u32sourceClock / ((u8osrCount + 1u) * u32baudrate);
        /* code */
        if(sbr < SBR_MAX_VALUE)
        {
            temp = (base->C4 & (~UART0_C4_OSR_MASK));
            temp |= u8osrCount << UART0_C4_OSR_SHIFT;
            base->C4 = temp;
            break;
        }
    }
    if(u8osrCount >= OSR_MAX)
    {
        retVal = false;
    }
    /* Calculate the baud rate based on the temporary SBR values */
    baudDiff = (u32sourceClock / (sbr * (u8osrCount + 1u))) - u32baudrate;

    /* Select the better value between sbr and (sbr + 1) */
    if (baudDiff > (u32baudrate - (u32sourceClock / ((sbr + 1) * (u8osrCount + 1)))))
    {
        sbr++;
    }

    /* Store C2 before disable Tx and Rx */
    oldCtrl = base->C2;

    /* Disable UART TX RX before setting. */
    base->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

    /* Write the sbr value to the BDH and BDL registers*/
    base->BDH = (base->BDH & ~UART_BDH_SBR_MASK) | (uint8_t)(sbr >> 8);/*get 5 high bits */
    base->BDL = (uint8_t)sbr;/*get 8 low bits */

    /* Restore C2. */
    base->C2 = oldCtrl;

    return retVal;
}

/* set parity mode */
static void UART_SetParityMode(UART0_Type *base, Uart_ParityMode_t parityMode)
{
    if (parityMode == UART_PARITY_NONE)
    {
        base->C1 &= ~(UART0_C1_PE_MASK | UART0_C1_PT_MASK);/* clear 2 pit parity enable and parity mode*/
    }
    else
    {
        base->C1 |= parityMode << UART0_C1_PT_MASK;
    }
}

/* set stop bit count */
static void UART_SetStopBitCount(UART0_Type *base, Uart_StopBitCount_t stopBitCount)
{
    base->BDH &= ~UART0_BDH_SBNS_MASK;/* reset mask bit */
    base->BDH |= stopBitCount << UART0_BDH_SBNS_SHIFT;
}

/* transmit a string data */
void UART_AsyncTransmit(UART0_Type *base,const uint8_t *dataPtr, uint16_t u16dataLen)
{
    /* check paragram */
    assert(NULL != dataPtr);
    assert(u16dataLen > 0);
    /* check paragram */
    while(u16dataLen--)
    {
        while (!(base->S1 & UART_S1_TDRE_MASK))/* check data register empty*/
        {
        }
        base->D = *(dataPtr++);/* send data */
    }
}

/* get string data from queue */
bool  UART_ReceivedQueue(UART0_Type *base, uint8_t **u8AddData)
{
    /* get string data from queue*/
    return QUEUE_ReadData(u8AddData);
}

/* handle interrupt and push data into queue*/
void UART0_IRQHandler()
{
    /* get data from Data register */
    uint8_t u8byteReceive = UART_GetChar(UART0);
    /* clear flag interrupt */
    UART_ClearInterruptsFlag(UART0);
    /* push data into queue */
    if(UART_DATA_FAIL != u8byteReceive)
    {
        QUEUE_ReceiveBuffer(u8byteReceive);
    }
}
/*******************************************************************************
 * END_FILE
 ******************************************************************************/

