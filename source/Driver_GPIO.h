#ifndef __DRIVER_GPIO_H__
#define __DRIVER_GPIO_H__
/*******************************************************************************
 * Include
 ******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "Hal_GPIO.h"
#include "Hal_PORT.h"
#include "clock.h"
/*******************************************************************************
 * Define
 ******************************************************************************/

/*! * @brief The GPIO input pin configuration structure.*/
typedef struct _Gpio_InputPin_t
{
    bool isPullEnable;                  /*!< Enable or disable pull. */
    Port_Pull_t pullSelect;             /*!< Select internal pull(up/down) resistor.*/
    Port_Interrupt_t interrupt;
}Gpio_InputPin_t;

/*! * @brief The GPIO output pin configuration structure. */
typedef struct _Gpio_OutputPin_t
{
    Gpio_PinState_t pinState;            /*!< Set default output logic.*/
}Gpio_OutputPin_t;

/*! @brief The GPIO input pin structure. */
typedef struct Gpio_InputPinUserConfig_t
{
    uint32_t pinName;        /*!< Virtual pin name from enumeration defined by the user.*/
    Gpio_InputPin_t config; /*!< Input pin configuration structure.*/
    GPIO_Type * gpioBase;
} Gpio_InputPinUserConfig_t;

/*@brief The GPIO output pin structure.*/
typedef struct GpioOutputPinUserConfig
{
    uint32_t pinName;        /*!< Virtual pin name from enumeration defined by the user.*/
    Gpio_OutputPin_t config;/*!< Input pin configuration structure.*/
    GPIO_Type * gpioBase;
} Gpio_OutputPinUserConfig_t;

/*******************************************************************************
 * prototype
 ******************************************************************************/

/**
  *@brief: Init GPIO input
  *@param[in]: inputPin for Init GPIO as Input
  *@retval: GPIO InitState
**/
void GPIO_DRV_InputPinInit(const Gpio_InputPinUserConfig_t *inputPin);

/**
  *@brief: Init GPIO out
  *@param[in]: putputPin for Init GPIO as output
  *@retval: GPIO InitState
**/
void GPIO_DRV_OutputPinInit(const Gpio_OutputPinUserConfig_t *outputPin);

/**
 * @brief: Gets the current direction of the individual GPIO pin.
 *
 * @param: pinName GPIO pin name defined by the user in the GPIO pin enumeration list.
 * @return: GPIO directions.
**/
Gpio_PinDirection_t GPIO_DRV_GetPinDir(GPIO_Type * base,uint32_t pinName);

/**
  *@brier: Set PIN Direction for GPIO
  *@param[in]: GPIOx: base of GPIOx (x as A...E).
  *@param[in]: PIN name for GPIO.
  *@param[in]: Direction of PIN input or output
  *@retval: GPIO InitState
**/
void GPIO_DRV_SetPinDir(GPIO_Type * base, uint32_t pinName, Gpio_PinDirection_t direction);

/**
  *@brier: Set logic level of PIN as 1 or 0.
  *@param[in]: FGPIOx: base of GPIOx (x as A...E).
  *@param[in]: PIN name for GPIO.
  *@param[in]: Logic level
         PIN_SET: Set logic level of PIN as 1
         PIN_RESET: Set logic level of PIN as 0
  *@retval: None
**/
void GPIO_DRV_WritePinOutput(GPIO_Type * base, uint32_t pinName, Gpio_PinState_t pinState);

/**
 * @brief: Sets the output level of the individual GPIO pin to the logic 1.
 * @param: pinName GPIO pin name defined by the user in the GPIO pin enumeration list.
**/
void GPIO_DRV_SetPinOutput(GPIO_Type * base, uint32_t pinName);

/**
 * @brief: Sets the output level of the individual GPIO pin to the logic 0.
 *
 * @param: pinName GPIO pin name defined by the user in the GPIO pin enumeration list.
**/
void GPIO_DRV_ClearPinOutput(GPIO_Type * base, uint32_t pinName);

/**
 * @brief: Configures the pin muxing.
 * @param[in]: GPIOx: base of GPIOx (x as A...E).
 * @param[in]: PIN name for GPIO.
 * @param: mux   pin muxing slot selection.
**/
void GPIO_DRV_SetMuxMode(GPIO_Type * base,uint32_t pinName,Port_Mux_t mux);

/**
  *@brier: Toggle logic level of PIN
  *@param[in]: FGPIOx: base of GPIOx (x as A...E).
  *@param[in]: GPIO_Pin: PIN name for GPIO.
  *@retval: None
**/
void GPIO_DRV_TogglePinOutput(GPIO_Type * base, uint32_t pinName);

/**
 * @brief: Reads the current input value of the individual GPIO pin.
 * @param: pinName GPIO pin name defined by the user in the GPIO pin enumeration list.
 * @return: GPIO port input value.
 *         - 0: Pin logic level is 0, or is not configured for use by digital function.
 *         - 1: Pin logic level is 1.
**/
uint32_t GPIO_DRV_ReadPinInput(GPIO_Type * base, uint32_t pinName);

/**
 * @brief: Reads the individual pin-interrupt status flag.
 * @param: pinName GPIO pin name defined by the user in the GPIO pin enumeration list.
 * @return: current pin interrupt status flag
 *         - 0: interrupt is not detected.
 *         - 1: interrupt is detected.
**/
bool GPIO_DRV_IsPinIntPending(GPIO_Type * base, uint32_t pinName);

/**
 * @brief: Clears the individual GPIO pin interrupt status flag.
 * @param: pinName GPIO pin name defined by the user in the GPIO pin enumeration list.
**/
void GPIO_DRV_ClearPinIntFlag(GPIO_Type * base, uint32_t pinName);

/**
 * @brief Sets the output of the GPIO port pins to a specific logic value.
 * This function  operates all 32 port pins.
 * @param base  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param portOutput  data to configure the GPIO output. Each bit represents one pin. For each bit:
**/
static inline void GPIO_DRV_WritePortOutput(GPIO_Type * base, uint32_t portOutput)
{
    GPIO_HAL_WritePortOutput(base,portOutput);
}
#endif /* __DRIVER_GPIO_H__*/