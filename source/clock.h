#ifndef __CLOCK_H__
#define __CLOCK_H__
/*******************************************************************************
 * Include
 ******************************************************************************/
#include <stdint.h>
#include <assert.h>
#include "MKL46Z4.h"

/*******************************************************************************
 * Define
 ******************************************************************************/


#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x00))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define ENABLE                 (1U)


/** @defgroup SIM_SCGC5_Clock_Enable_Disable
  * @brief  Enable or disable clock for Gating Control Register 5 ;
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __HAL_SIM_PORTA_CLK_ENABLE()   do { \
                                        uint32_t tmpreg; \
                                        SET_BIT(SIM->SCGC5, SIM_SCGC5_PORTA(ENABLE));\
                                        /* check clock port enable */\
                                        tmpreg = READ_BIT(SIM->SCGC5, SIM_SCGC5_PORTA_MASK);\
                                        assert(tmpreg); \
                                      } while(0U)

#define __HAL_SIM_PORTB_CLK_ENABLE()   do { \
                                        uint32_t tmpreg; \
                                        SET_BIT(SIM->SCGC5, SIM_SCGC5_PORTB(ENABLE));\
                                        /* check clock port enable */\
                                        tmpreg = READ_BIT(SIM->SCGC5, SIM_SCGC5_PORTB_MASK);\
                                        assert(tmpreg); \
                                      } while(0U)

#define __HAL_SIM_PORTC_CLK_ENABLE()   do { \
                                        uint32_t tmpreg; \
                                        SET_BIT(SIM->SCGC5, SIM_SCGC5_PORTC(ENABLE));\
                                        /* check clock port enable */\
                                        tmpreg = READ_BIT(SIM->SCGC5, SIM_SCGC5_PORTC_MASK);\
                                        assert(tmpreg); \
                                      } while(0U)

#define __HAL_SIM_PORTD_CLK_ENABLE()   do { \
                                        uint32_t tmpreg; \
                                        SET_BIT(SIM->SCGC5, SIM_SCGC5_PORTD(ENABLE));\
                                        /* check clock port enable */\
                                        tmpreg = READ_BIT(SIM->SCGC5, SIM_SCGC5_PORTD_MASK);\
                                        assert(tmpreg); \
                                      } while(0U)

#define __HAL_SIM_PORTE_CLK_ENABLE()   do { \
                                        uint32_t tmpreg; \
                                        SET_BIT(SIM->SCGC5, SIM_SCGC5_PORTE(ENABLE));\
                                        /* check clock port enable */\
                                        tmpreg = READ_BIT(SIM->SCGC5, SIM_SCGC5_PORTE_MASK);\
                                        assert(tmpreg); \
                                      } while(0U)

#define __HAL_SIM_GPIOA_CLK_DISABLE()     (SIM->SCGC5 &= ~(SIM_SCGC5_PORTA_MASK))
#define __HAL_SIM_GPIOB_CLK_DISABLE()     (SIM->SCGC5 &= ~(SIM_SCGC5_PORTB_MASK))
#define __HAL_SIM_GPIOC_CLK_DISABLE()     (SIM->SCGC5 &= ~(SIM_SCGC5_PORTC_MASK))
#define __HAL_SIM_GPIOD_CLK_DISABLE()     (SIM->SCGC5 &= ~(SIM_SCGC5_PORTD_MASK))
#define __HAL_SIM_GPIOE_CLK_DISABLE()     (SIM->SCGC5 &= ~(SIM_SCGC5_PORTE_MASK))


/** @defgroup SIM_SCGC6_Clock_Enable_Disable
  * @brief  Enable or disable clock for Gating Control Register 5 ;
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __HAL_SIM_UART0_CLK_ENABLE()     (SIM->SCGC4 |= (SIM_SCGC4_UART0_MASK))

#define __HAL_SIM_UART0_CLK_DISABLE()    (SIM->SCGC4 &= ~(SIM_SCGC4_UART0_MASK))

/* select source clock for uart0*/
#define __HAL_SIM_UART0_CLK_UART0SRC_SELECT(clockSource) (SIM->SOPT2 |= SIM_SOPT2_UART0SRC(clockSource))

#define __HAL_SIM_UART0_CLK_PLLFLLSEL_SELECT(clockSource) (SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL(clockSource))


/*******************************************************************************
 * Prototpype
 ******************************************************************************/
/**
  * @brief  Enable or disable clock for PORT
  *
  * @param[in] PORTx : specifies the port bit to be written.
  *
  * @retval None
  */
void HAL_SIM_EnableClockPort(GPIO_Type * GPIOx);
#endif /* __CLOCK_H__ */
