/*******************************************************************************
 * Include
 ******************************************************************************/
#include "Driver_GPIO.h"
#include "MKL46Z4.h"
#include "MKL46Z4_features.h"

/*******************************************************************************
 * Variable
 ******************************************************************************/
static PORT_Type *const s_portBases[] = PORT_BASE_PTRS;
static GPIO_Type *const s_gpioBases[] = GPIO_BASE_PTRS;

/*******************************************************************************
 * Prototype
 ******************************************************************************/
/**
* @brief: Gets the GPIO instance according to the GPIO base
*
* @param: base    GPIO peripheral base pointer(PTA, PTB, PTC, etc.)
* @retval: GPIO instance
**/
static uint32_t GPIO_DRV_GetInstance(GPIO_Type *base);

/*******************************************************************************
 * code
 ******************************************************************************/
/* @brief Gets the GPIO instance according to the GPIO base */
static uint32_t GPIO_DRV_GetInstance(GPIO_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < FSL_FEATURE_SOC_GPIO_COUNT; instance++)
    {
        if (s_gpioBases[instance] == base)
        {
            break;
        }
    }
    /* check port is correct*/
    assert(instance < FSL_FEATURE_SOC_GPIO_COUNT);

    return instance;
}

/* Initialize one GPIO input pin used by board. */
void GPIO_DRV_InputPinInit(const Gpio_InputPinUserConfig_t *inputPin)
{
    /* Get actual port and pin number.*/
    uint8_t instance;
    GPIO_Type * gpio;
    PORT_Type * port;
    uint32_t pin = inputPin->pinName;
    instance = GPIO_DRV_GetInstance(inputPin->gpioBase);
    gpio = s_gpioBases[instance];
    port = s_portBases[instance];

    /* Un-gate port clock*/
    HAL_SIM_EnableClockPort(gpio);

    /* Set current pin as gpio.*/
    PORT_HAL_SetMuxMode(port, pin, PORT_MUX_AS_GPIO);

    /* Set current pin as digital input.*/
    GPIO_HAL_SetPinDir(gpio, pin, GPIO_DIGITAL_INPUT);
    /* set pull select for pin*/
    PORT_HAL_SetPullCmd(port, pin, inputPin->config.isPullEnable);
    /* set pull mode for pin*/
    PORT_HAL_SetPullMode(port, pin, inputPin->config.pullSelect);
    /* set interrupt for pin*/
    PORT_HAL_SetPinIntMode(port, pin, inputPin->config.interrupt);
        /* Configure NVIC */
    if ((inputPin->config.interrupt !=PORT_INT_DISABLED ) && (gpio == GPIOA))
    {
        /* Enable GPIO interrupt.*/
        NVIC_EnableIRQ(PORTA_IRQn);
    }
    else if ((inputPin->config.interrupt !=PORT_INT_DISABLED ) && (gpio == GPIOC || gpio == GPIOD))
    {
        /* Enable GPIO interrupt.*/
        NVIC_EnableIRQ(PORTC_PORTD_IRQn);
    }
}

void GPIO_DRV_OutputPinInit(const Gpio_OutputPinUserConfig_t *outputPin)
{
    /* Get actual port and pin number.*/
    uint8_t instance;
    GPIO_Type * gpio;
    PORT_Type * port;
    uint32_t pin = outputPin->pinName;
    instance = GPIO_DRV_GetInstance(outputPin->gpioBase);
    gpio = s_gpioBases[instance];
    port = s_portBases[instance];

    /* Un-gate port clock*/
    HAL_SIM_EnableClockPort(gpio);

    /* Set current pin as gpio.*/
    PORT_HAL_SetMuxMode(port, pin, PORT_MUX_AS_GPIO);

    /* Set current pin as digital output.*/
    GPIO_HAL_SetPinDir(gpio, pin, GPIO_DIGITAL_OUTPUT);

    /* Configure GPIO output features. */
    GPIO_HAL_WritePinOutput(gpio,pin, outputPin->config.pinState);

}

/* @brief Configures the pin muxing.*/
void GPIO_DRV_SetMuxMode(GPIO_Type * base,uint32_t pinName,Port_Mux_t mux)
{
    uint8_t instance;
    PORT_Type *portBase;

    instance = GPIO_DRV_GetInstance(base);
    portBase = s_portBases[instance];
    PORT_HAL_SetMuxMode(portBase,pinName,mux);
}

/*Get current direction of individual GPIO pin*/
Gpio_PinDirection_t GPIO_DRV_GetPinDir(GPIO_Type * base,uint32_t pinName)
{
    return GPIO_HAL_GetPinDir(base, pinName);
}

/* Set current direction of individual GPIO pin.*/
void GPIO_DRV_SetPinDir(GPIO_Type * base, uint32_t pinName, Gpio_PinDirection_t direction)
{
    GPIO_HAL_SetPinDir(base, pinName, direction);
}

/* Set output level of individual GPIO pin to logic 1 or 0.*/
void GPIO_DRV_WritePinOutput(GPIO_Type * base, uint32_t pinName, Gpio_PinState_t pinState)
{
    GPIO_HAL_WritePinOutput(base, pinName, pinState);
}

/* Set output level of individual GPIO pin to logic 1.*/
void GPIO_DRV_SetPinOutput(GPIO_Type * base, uint32_t pinName)
{
    GPIO_HAL_SetPinOutput(base, pinName);
}

/*Set output level of individual GPIO pin to logic 0.*/
void GPIO_DRV_ClearPinOutput(GPIO_Type * base,uint32_t pinName)
{
    GPIO_HAL_ClearPinOutput(base, pinName);
}

/* Reverse current output logic of individual GPIO pin.*/
void GPIO_DRV_TogglePinOutput(GPIO_Type * base,uint32_t pinName)
{
    GPIO_HAL_TogglePinOutput(base, pinName);
}

/*Read current input value of individual GPIO pin.*/
uint32_t GPIO_DRV_ReadPinInput(GPIO_Type * base, uint32_t pinName)
{
    return GPIO_HAL_ReadPinInput(base, pinName);
}

/* @brier: Read logic level of PIN */
bool GPIO_DRV_IsPinIntPending(GPIO_Type * base,uint32_t pinName)
{
    uint32_t instance;
    PORT_Type *portBase;

    instance = GPIO_DRV_GetInstance(base);/* get instance of port */
    portBase = s_portBases[instance];
    return PORT_HAL_IsPinIntPending(portBase, pinName);
}

/* Clear individual GPIO pin interrupt status flag.*/
void GPIO_DRV_ClearPinIntFlag(GPIO_Type * base, uint32_t pinName)
{
    uint8_t instance;
    PORT_Type *portBase;

    instance = GPIO_DRV_GetInstance(base);/* get instance of port */
    portBase = s_portBases[instance];
    PORT_HAL_ClearPinIntFlag(portBase, pinName);
}

/*******************************************************************************
 * EndFile
 ******************************************************************************/