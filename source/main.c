/******************************************************************************
 * Include
 ******************************************************************************/
#include "Driver_GPIO.h"
#include "Hal_UART.h"
#include "SrecParse.h"
#include "clock.h"

/******************************************************************************
 * define
 ******************************************************************************/
/* gpio pin and port uart */
#define GPIO_UART0_RX_PIN 1U
#define GPIO_UART0_TX_PIN 2U
#define GPIO_UART0_RX_PORT GPIOA
#define GPIO_UART0_TX_PORT GPIOA
/* select source clock for uart */
#define SOURCE_CLOCK_MCGFLLCLK_OR_MCGFLLCLK_2 1U
#define SOURCE_CLOCK_PLLFLLSEL_MCGFLLCLK 0U
/******************************************************************************
 * prototype
 ******************************************************************************/
/**
 * @brief:  init GPIO for UART0 and UART0
 * @param:  none
 * @return: none
 */
static void InitAll(void);
 /******************************************************************************
 * code
 ******************************************************************************/
int main()
{
    //bool check = true;
    uint8_t *u8PtrData;

    InitAll();/*init GPIO for UART0 and UART0*/

    while(1)
    {
        if(true == UART_ReceivedQueue(UART0,&u8PtrData))/* check queue has data */
        {
            if(true == SREC_RecordCheck(u8PtrData))/* check SREC parse is true */
            {
                UART_AsyncTransmit(UART0,"<<\r",3); /* printf on moniter is true */
            }
            else
            {
                UART_AsyncTransmit(UART0,"\r",1);/* verify a error implemented */
                //check = false;
            }
        }
    }
}
/* init GPIO for UART0 and UART0 */
static void InitAll(void)
{
    Gpio_OutputPinUserConfig_t initStructPGIO;
    Uart_Config_t hwConfig;

    /* inititial GPIO for uart RX*/
    initStructPGIO.gpioBase = GPIO_UART0_RX_PORT;
    initStructPGIO.pinName = GPIO_UART0_RX_PIN;
    initStructPGIO.config.pinState = GPIO_PIN_RESET;
    GPIO_DRV_OutputPinInit(&initStructPGIO);
    GPIO_DRV_SetMuxMode(GPIO_UART0_RX_PORT,GPIO_UART0_RX_PIN,PORT_MUX_ALT2);

    /* inititial GPIO for uart TX*/
    initStructPGIO.pinName = GPIO_UART0_TX_PIN;
    initStructPGIO.gpioBase = GPIO_UART0_TX_PORT;
    GPIO_DRV_OutputPinInit(&initStructPGIO);
    GPIO_DRV_SetMuxMode(GPIO_UART0_TX_PORT,GPIO_UART0_TX_PIN,PORT_MUX_ALT2);

    /* select source clock for uart 0*/
    __HAL_SIM_UART0_CLK_UART0SRC_SELECT(SOURCE_CLOCK_MCGFLLCLK_OR_MCGFLLCLK_2);
    __HAL_SIM_UART0_CLK_PLLFLLSEL_SELECT(SOURCE_CLOCK_PLLFLLSEL_MCGFLLCLK);

    /* Inititial for uart config */
    hwConfig.u32Baudrate = 115200;
    hwConfig.StopBitCount = UART_STOP_BIT_1;
    hwConfig.ParityMode = UART_PARITY_NONE;
    hwConfig.u32SourceClock = DEFAULT_SYSTEM_CLOCK;
    UART_Init(UART0,&hwConfig);
}