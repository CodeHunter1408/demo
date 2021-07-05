/*******************************************************************************
 * Include
 ******************************************************************************/
#include "Hal_GPIO.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Set individual gpio pin to general input or output.*/
void GPIO_HAL_SetPinDir(GPIO_Type * base, uint32_t pin, Gpio_PinDirection_t direction)
{
    /*check paragram*/
    assert(pin < ARRANGE_PIN);

    if (direction == GPIO_DIGITAL_OUTPUT)
    {
        base->PDDR |=  (1U << pin);
    }
    else
    {
        base->PDDR &= ~(1U << pin);
    }
}

/* Set output level of individual gpio pin to logic 1 or 0.*/
void GPIO_HAL_WritePinOutput(GPIO_Type * base, uint32_t pin, Gpio_PinState_t pinState)
{
    /*check paragram*/
    assert(pin < ARRANGE_PIN);

    if (pinState != GPIO_PIN_RESET)
    {
        base->PSOR |= (1U << pin); /* Set pin output to high level.*/
    }
    else
    {
        base->PCOR |= (1U << pin); /* Set pin output to low level.*/
    }
}

/*******************************************************************************
 * EndFile
 ******************************************************************************/