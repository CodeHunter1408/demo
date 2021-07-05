/*******************************************************************************
 * Include
 ******************************************************************************/
#include "clock.h"

/*******************************************************************************
 * code
 ******************************************************************************/
 /* @brief  Enable or disable clock for PORT */
void HAL_SIM_EnableClockPort(GPIO_Type * GPIOx)
{
    if (GPIOx == GPIOA)
    {
        __HAL_SIM_PORTA_CLK_ENABLE();/* ennable port A */
    }
    else if (GPIOx == GPIOB)
    {
        __HAL_SIM_PORTB_CLK_ENABLE();/* ennable port B */
    }
    else if (GPIOx == GPIOC)
    {
        __HAL_SIM_PORTC_CLK_ENABLE();/* ennable port C */
    }
    else if (GPIOx == GPIOD)
    {
        __HAL_SIM_PORTD_CLK_ENABLE();/* ennable port D */
    }
    else if (GPIOx == GPIOE)
    {
        __HAL_SIM_PORTE_CLK_ENABLE();/* ennable port E */
    }
}
/*******************************************************************************
 * EndFile
 ******************************************************************************/