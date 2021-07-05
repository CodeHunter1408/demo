#ifndef __HAL_PORT_H__
#define __HAL_PORT_H__
/*******************************************************************************
 * Include
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "MKL46Z4.h"

/*******************************************************************************
 * Define
 ******************************************************************************/
/* @brief Internal resistor pull feature selection */
typedef enum _port_pull
{
    PORT_PULL_DOWN = 0U,  /*!< Internal pull-down resistor is enabled */
    PORT_PULL_UP   = 1U   /*!< Internal pull-up resistor is enabled. */
}Port_Pull_t;

/*! @brief Slew rate selection*/
typedef enum _port_slew_rate {
    PORT_FAST_SLEW_RATE = 0U,  /*!< Fast slew rate is configured. */
    PORT_SLOW_SLEW_RATE = 1U   /*!< Slow slew rate is configured. */
}PORT_SlewRate_t;

/*! @brief Configures the drive strength.*/
typedef enum _port_drive_strength {
    PORT_LOW_DRIVE_STRENGTH  = 0U, /*!< Low drive strength is configured. */
    PORT_HIGH_DRIVE_STRENGTH = 1U  /*!< High drive strength is configured.*/
}PORT_DriverStrength_t;

/*! @brief Pin mux selection */
typedef enum _port_mux
{
    PORT_PIN_DISABLED_OR_ANALOG = 0U, /*!< Corresponding pin is disabled, but is used as an analog pin. */
    PORT_MUX_AS_GPIO = 1U,           /*!< Corresponding pin is configured as GPIO. */
    PORT_MUX_ALT2 = 2U,             /*!< Chip-specific */
    PORT_MUX_ALT3 = 3U,             /*!< Chip-specific */
    PORT_MUX_ALT4 = 4U,             /*!< Chip-specific */
    PORT_MUX_ALT5 = 5U,             /*!< Chip-specific */
    PORT_MUX_ALT6 = 6U,             /*!< Chip-specific */
    PORT_MUX_ALT7 = 7U,             /*!< Chip-specific */
}Port_Mux_t;

/* @brief Configures the interrupt generation condition. */
typedef enum _port_interrupt
{
    PORT_INT_DISABLED    = 0x0U,  /*!< Interrupt/DMA request is disabled.*/
    PORT_DMA_RISING_EDGE  = 0x1U,  /*!< DMA request on rising edge.*/
    PORT_DMA_FALLING_EDGE = 0x2U,  /*!< DMA request on falling edge.*/
    PORT_DMA_EITHER_EDGE  = 0x3U,  /*!< DMA request on either edge.*/
    PORT_INT_LOGIC_LOW   = 0x8U,  /*!< Interrupt when logic zero. */
    PORT_INT_RISING_EDGE  = 0x9U,  /*!< Interrupt on rising edge. */
    PORT_INT_FALLING_EDGE = 0xAU,  /*!< Interrupt on falling edge. */
    PORT_INT_EITHER_EDGE  = 0xBU,  /*!< Interrupt on either edge. */
    PORT_INT_LOGIC_ONE    = 0xCU   /*!< Interrupt when logic one. */
} Port_Interrupt_t;

/**
 * @brief: Selects the internal resistor as pull-down or pull-up.
 * @param: base  port base pointer.
 * @param: pin       port pin number
 * @param: pullSelect  internal resistor pull feature selection
 */
static inline void PORT_HAL_SetPullMode(PORT_Type * base,
                                        uint32_t pin,
                                        Port_Pull_t pullSelect)
{
    assert(pin < ARRANGE_PIN);
    base->PCR[pin] &= ~PORT_PCR_PS_MASK;
    base->PCR[pin] |= PORT_PCR_PS(pullSelect);
}

/**
 * @brief:  Enables or disables the internal pull resistor.
 * @param:  base  port base pointer
 * @param:  pin       port pin number
 * @param:  isPullEnabled  internal pull resistor enable or disable
 *        - true : internal pull resistor is enabled.
 *        - false: internal pull resistor is disabled.
 */
static inline void PORT_HAL_SetPullCmd(PORT_Type * base,
                                       uint32_t pin,
                                       bool isPullEnabled)
{
    assert(pin < ARRANGE_PIN);
    base->PCR[pin] |= PORT_PCR_PE(isPullEnabled);
}

/**
 * @brief: Configures the pin muxing.
 * @param: base  PORT peripheral base pointer.
 * @param: pin   PORT pin number.
 * @param: mux   pin muxing slot selection.
 */
static inline void PORT_HAL_SetMuxMode(PORT_Type *base, \
                                       uint32_t pin,\
                                       Port_Mux_t mux)
{
    base->PCR[pin] &= ~PORT_PCR_MUX_MASK;
    base->PCR[pin] |= PORT_PCR_MUX(mux);
}
/**
 * @brief: Configures the low half of the pin control register for the same settings.
 * @param: base  port base pointer
 * @param: lowPinSelect  update corresponding pin control register or not. For a specific bit:
 * @param: config  value  is written to a low half port control register bits[15:0].
 */
void PORT_HAL_SetLowGlobalPinCtrl(PORT_Type * base, uint16_t lowPinSelect, uint16_t config);

/*!
 * @brief: Configures the high half of the pin control register for the same settings.
 * @param: base  port base pointer
 * @param: highPinSelect  update corresponding pin control register or not. For a specific bit:
 * @param: config  value is  written to a high half port control register bits[15:0].
 */
void PORT_HAL_SetHighGlobalPinCtrl(PORT_Type * base, uint16_t highPinSelect, uint16_t config);

/**
 * @brief: Configures the port pin interrupt/DMA request.
 * @param: base    PORT peripheral base pointer.
 * @param: pin     PORT pin number.
 * @param: config  PORT pin interrupt configuration.
 */
static inline void PORT_HAL_SetPinIntMode(PORT_Type * base,
                                          uint32_t pin,
                                          Port_Interrupt_t intConfig)
{
    assert(pin < ARRANGE_PIN);
    base->PCR[pin] &= ~PORT_PCR_IRQC_MASK;
    base->PCR[pin] |= PORT_PCR_IRQC(intConfig);
}

/**
 * @brief: Gets the current port pin interrupt/DMA request configuration.
 * @param: base  port base pointer
 * @param: pin  port pin number
 * @return:  interrupt configuration
 */
static inline Port_Interrupt_t PORT_HAL_GetPinIntMode(PORT_Type * base, uint32_t pin)
{
    assert(pin < ARRANGE_PIN);
    return ((Port_Interrupt_t)((base->PCR[pin] & PORT_PCR_IRQC_MASK) >> PORT_PCR_IRQC_SHIFT));
}

/**
 * @brief: Reads the individual pin-interrupt status flag.
 * @param: base  port base pointer
 * @param: pin  port pin number
 * @return: current pin interrupt status flag
 *         - 0: interrupt is not detected.
 *         - 1: interrupt is detected.
**/
static inline bool PORT_HAL_IsPinIntPending(PORT_Type * base, uint32_t pin)
{
    assert(pin < ARRANGE_PIN);
    return ((base->PCR[pin] & PORT_PCR_ISF_MASK) >> PORT_PCR_ISF_SHIFT);
}

/**
 * @brief: Clears the individual pin-interrupt status flag.
 * @param: base  port base pointer
 * @param: pin  port pin number
**/
static inline void PORT_HAL_ClearPinIntFlag(PORT_Type * base, uint32_t pin)
{
    assert(pin < ARRANGE_PIN);
    base->PCR[pin] |= PORT_PCR_ISF(1U);
}

/**
 * @brief: Reads the entire port interrupt status flag.
 * @param: base  port base pointer
 * @return: all 32 pin interrupt status flags. For specific bit:
 *         - 0: interrupt is not detected.
 *         - 1: interrupt is detected.
**/
static inline uint32_t PORT_HAL_GetPortIntFlag(PORT_Type * base)
{
    return (base->ISFR);
}

/**
 * @brief: Clears the entire port interrupt status flag.
 * @param: base  port base pointer
**/
static inline void PORT_HAL_ClearPortIntFlag(PORT_Type * base)
{
    base->ISFR = PORT_ISFR_ISF_MASK;
}

#endif /*__HAL_PORT_H__ */