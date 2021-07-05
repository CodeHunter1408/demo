#ifndef __HAL_GPIO_H__
#define __HAL_GPIO_H__
/*******************************************************************************
 * Include
 ******************************************************************************/
#include "MKL46Z4.h"
#include <assert.h>
#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* @brief GPIO direction definition*/
typedef enum _GPIO_PinDirection
{
    GPIO_DIGITAL_INPUT = 0U,  /*!< Set current pin as digital input*/
    GPIO_DIGITAL_OUTPUT = 1U, /*!< Set current pin as digital output*/
} Gpio_PinDirection_t;

/*! @brief  GPIO Bit SET and Bit RESET enumeration*/
typedef enum _GPIO_PinState
{
    GPIO_PIN_RESET = 0u,
    GPIO_PIN_SET   = 1U,
} Gpio_PinState_t;

/**
 * @brief Sets the individual GPIO pin to general input or output.
 * @param base  GPIO base pointer(PTA, PTB, PTC, etc.)
 * @param pin  GPIO port pin number
 * @param direction  GPIO directions
 */
void GPIO_HAL_SetPinDir(GPIO_Type * base, uint32_t pin,
                        Gpio_PinDirection_t direction);

/**
 * @brief Sets the GPIO port pins to general input or output.
 * This function  operates all 32 port pins.
 * @param base  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param pinDirectionMap  GPIO directions bit map
 */
static inline void GPIO_HAL_SetPortDir(GPIO_Type * base, uint32_t pinDirectionMap)
{
    base->PDDR = pinDirectionMap;
}

/**
 * @brief Gets the current direction of the individual GPIO pin.
 * @param base  GPIO base pointer(PTA, PTB, PTC, etc.)
 * @param pin  GPIO port pin number
 * @return GPIO directions
 */
static inline Gpio_PinDirection_t GPIO_HAL_GetPinDir(GPIO_Type * base, uint32_t pin)
{
    assert(pin < ARRANGE_PIN);
    return (Gpio_PinDirection_t)((base->PDDR >> pin) & 1U);
}

/**
 * @brief Gets the GPIO port pins direction.
 * @param base  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @return GPIO directions. Each bit represents one pin. For each bit:
 */
static inline uint32_t GPIO_HAL_GetPortDir(GPIO_Type * base)
{
    return (base->PDDR);
}

/**
 * @brief Sets the output level of the individual GPIO pin to logic 1 or 0.
 * @param base  GPIO base pointer(PTA, PTB, PTC, etc.)
 * @param pin  GPIO port pin number
 * @param pinState  pin output logic level
 */
void GPIO_HAL_WritePinOutput(GPIO_Type * base, uint32_t pin, Gpio_PinState_t pinState);

/**
 * @brief Reads the current pin output.
 * @param base  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param pin  GPIO port pin number
 * @return current pin output status. 0 - Low logic, 1 - High logic
 */
static inline uint32_t GPIO_HAL_ReadPinOutput(GPIO_Type * base, uint32_t pin)
{
    assert(pin < ARRANGE_PIN);
    return (((base->PDOR) >> pin) & 0x1U);
}

/**
 * @brief Sets the output level of the individual GPIO pin to logic 1.
 * @param base  GPIO base pointer(PTA, PTB, PTC, etc.)
 * @param pin  GPIO port pin number
 */
static inline void GPIO_HAL_SetPinOutput(GPIO_Type * base, uint32_t pin)
{
    assert(pin < ARRANGE_PIN);
    base->PSOR |= (1U << pin);
}

/**
 * @brief Clears the output level of the individual GPIO pin to logic 0.
 * @param base  GPIO base pointer(PTA, PTB, PTC, etc.)
 * @param pin  GPIO port pin number
 */
static inline void GPIO_HAL_ClearPinOutput(GPIO_Type * base, uint32_t pin)
{
    assert(pin < ARRANGE_PIN);
    base->PCOR |= (1U << pin);
}

/**
 * @brief Reverses the current output logic of the individual GPIO pin.
 * @param base  GPIO base pointer(PTA, PTB, PTC, etc.)
 * @param pin  GPIO port pin number
 */
static inline void GPIO_HAL_TogglePinOutput(GPIO_Type * base, uint32_t pin)
{
    assert(pin < ARRANGE_PIN);
    base->PTOR |= (1U << pin);
}

/**
 * @brief Sets the output of the GPIO port pins to a specific logic value.
 * This function  operates all 32 port pins.
 * @param base  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param portOutput  data to configure the GPIO output. Each bit represents one pin. For each bit:
 */
static inline void GPIO_HAL_WritePortOutput(GPIO_Type * base, uint32_t portOutput)
{
    base->PDOR = portOutput;
}

/**
 * @brief Reads out all pin output status of the current port.
 * This function  operates all 32 port pins.
 * @param base  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @return current port output status. Each bit represents one pin. For each bit:
 */
static inline uint32_t GPIO_HAL_ReadPortOutput(GPIO_Type * base)
{
    return (base->PDOR);
}

/**
 * @brief Sets the output level of the GPIO port pins to logic 1.
 * This function  operates all 32 port pins.
 * @param base  GPIO base pointer(PTA, PTB, PTC, etc.)
 * @param portOutput GPIO output port pin mask. Each bit represents one pin. For each bit:
 */
static inline void GPIO_HAL_SetPortOutput(GPIO_Type * base, uint32_t portOutput)
{
    base->PSOR = portOutput;
}

/**
 * @brief Clears the output level of the GPIO port pins to logic 0.
 * This function  operates all 32 port pins.
 * @param base  GPIO base pointer(PTA, PTB, PTC, etc.)
 * @param portOutput  mask of GPIO output pins. Each bit represents one pin. For each bit:
 */
static inline void GPIO_HAL_ClearPortOutput(GPIO_Type * base, uint32_t portOutput)
{
    base->PCOR = portOutput;
}

/**
 * @brief Reverses the current output logic of the GPIO port pins.
 * This function  operates all 32 port pins.
 * @param base  GPIO base pointer(PTA, PTB, PTC, etc.)
 * @param portOutput  mask of GPIO output pins. Each bit represents one pin. For each bit:
 */
static inline void GPIO_HAL_TogglePortOutput(GPIO_Type * base, uint32_t portOutput)
{
    base->PTOR = portOutput;
}

/**
 * @brief Reads the current input value of the individual GPIO pin.
 * @param base  GPIO base pointer(PTA, PTB, PTC, etc.)
 * @param pin  GPIO port pin number
 * @return GPIO port input value
 */
static inline uint32_t GPIO_HAL_ReadPinInput(GPIO_Type * base, uint32_t pin)
{
    assert(pin < ARRANGE_PIN);
    return (((base->PDIR) >> pin) & 1U);
}

/**
 * @brief Gets the current input value of a specific FGPIO port.
 * @param base  GPIO base pointer(FPTA, FPTB, FPTC, etc.).
 * @return FGPIO port input data. Each bit represents one pin. For each bit:
 */
static inline uint32_t GPIO_HAL_ReadPortInput(GPIO_Type * base)
{
    return (base->PDIR);
}

#endif /* __HAL_GPIO_H__ */
