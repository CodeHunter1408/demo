#ifndef __HAL_UART_H__
#define __HAL_UART_H__
/*******************************************************************************
 * Include
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "MKL46Z4.h"

/*******************************************************************************
 * Struct
 ******************************************************************************/
/*! @brief UART stop bit count. */
typedef enum __Uart_StopBitCount_t
{
    UART_STOP_BIT_1 = 0x0U,/*!< One stop bit */
    UART_STOP_BIT_2 = 0x2U,/*!< Two stop bits */
} Uart_StopBitCount_t;

/*! @brief UART parity mode. */
typedef enum __Uart_ParityMode_t
{
    UART_PARITY_NONE    = 0x0U,/*!< Parity disabled */
    UART_PARITY_EVEN    = 0x2U,/*!< Parity enabled, type even, bit setting: PE|PT = 10 */
    UART_PARITY_ODD     = 0x3U,/*!< Parity enabled, type odd,  bit setting: PE|PT = 11 */
} Uart_ParityMode_t;

/*! @brief UART configuration structure. */
typedef struct __Uart_ChannelConfigType
{
    uint32_t                    u32SourceClock;/*clock source frequency in HZ.*/
    uint32_t                    u32Baudrate;/*!< UART baud rate  */
    Uart_ParityMode_t           ParityMode;/*!< Parity mode, disabled (default), even, odd *//*!< Number of stop bits, 1 stop bit (default) or 2 stop bits  */
    Uart_StopBitCount_t         StopBitCount;/*!< Number of stop bits, 1 stop bit (default) or 2 stop bits  */
} Uart_Config_t;


typedef enum
{
    E_Error = 0u,
    E_Success = 1u,
}Std_ReturnType;


/*******************************************************************************
 * Define
 ******************************************************************************/
/*!
 * @brief Initializes a UART instance with user configuration structure and peripheral clock.
 * example
 *  uart_config_t uartConfig;
 *  uartConfig.u32SourceClock = DEFAULT_SYSTEM_CLOCK;
 *  uartConfig.baudRate = 115200U;
 *  uartConfig.parityMode = UART_PARITY_NONE;
 *  uartConfig.stopBitCount = UART_STOP_BIT_1;
 *  UART_Init(UART0, &uartConfig);
 * @endcode
 * @param base UART peripheral base address.
 * @param config Pointer to user-defined configuration structure.
 * @retval true Status UART initialize succeed
           false status UART initialize didn't succee
 */
bool UART_Init(UART0_Type *base, const Uart_Config_t *uartConfig);

/*!
 * @brief Writes to the TX register using a blocking method.
 *
 * This function polls the TX register, waits for the TX register to be empty or for the TX FIFO
 * to have room and writes data to the TX buffer.
 * @param base UART peripheral base address.
 * @param data Start address of the data to write.
 * @param length Size of the data to write.
 */
void UART_AsyncTransmit(UART0_Type *base,const uint8_t *dataPtr, uint16_t u16dataLen);

/*!
 * @brief Read RX data register using a queue method.
 *
 * @param base UART peripheral base address.
 * @param address string data pointer of the data
 */
bool  UART_ReceivedQueue(UART0_Type *base, uint8_t **u8AddData);

/*!
 * @brief Reads the RX register directly.
 *
 * This function reads data from the TX register directly. The upper layer must
 * ensure that the RX register is full or that the TX FIFO has data before calling this function.
 *
 * @param base UART peripheral base address.
 * @return The byte read from UART data register.
 */
static inline uint8_t UART_GetChar(UART0_Type *base)
{
    return (base->D);
}

/*!
 * @brief clear UART interrupts flag for receive data.
 * @param base UART peripheral base address.
 * @return none
 */
static inline void UART_ClearInterruptsFlag(UART0_Type *base)
{
    base->S2 |= UART0_S2_RXEDGIF_MASK;
}

/*!
 * @brief Gets the enabled UART interrupts for receive data.
 * @param base UART peripheral base address.
 * @return none
 */
static void UART_EnableInterrupts(UART0_Type *base)
{
    /* The interrupt mask is combined by control bits from several register: ((CFIFO<<24) | (C3<<16) | (C2<<8) |(BDH))
     */
    base->BDH |= UART0_BDH_RXEDGIE_MASK;
    base->C2 |= UART0_C2_RIE_MASK ;
    base->C3 |= UART0_C3_FEIE_MASK;
}
#endif /*__HAL_UART_H__ */