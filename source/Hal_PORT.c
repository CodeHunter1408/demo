/*******************************************************************************
 * Include
 ******************************************************************************/
#include "Hal_PORT.h"

/*******************************************************************************
 * Code
 ******************************************************************************/
/*Configure high half of pin control register for the same*/
void PORT_HAL_SetLowGlobalPinCtrl(PORT_Type * base, uint16_t lowPinSelect, uint16_t config)
{
    uint32_t combine = lowPinSelect;
    combine = (combine << 16) + config;
    base->GPCLR |= combine;
}

/*Configure high half of pin control register for the same
 *settings, this function operates pin 16 -31 of one specific port.*/
void PORT_HAL_SetHighGlobalPinCtrl(PORT_Type * base, uint16_t highPinSelect, uint16_t config)
{
    uint32_t combine = highPinSelect;
    combine = (combine << 16) + config;
    base->GPCHR |= combine;
}

/*******************************************************************************
 * END_FILE
 ******************************************************************************/

