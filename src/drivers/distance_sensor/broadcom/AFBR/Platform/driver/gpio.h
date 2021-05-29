/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides driver functionality for GPIO.
 *
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *
 *****************************************************************************/

#ifndef GPIO_H
#define GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup	GPIO GPIO: General Purpose Input/Output
 * @ingroup		driver
 * @brief		GPIO Hardware Module
 * @details		Provides functionality for GPIO operation.
 * @addtogroup 	GPIO
 * @{
 *****************************************************************************/

#include <stdint.h>

/*!***************************************************************************
 * @brief	GPIO layer interrupt service routine function type.
 * @param	param A void pointer that is passed to the function.
 *****************************************************************************/
typedef void (* GPIO_isr_t)(void * param);

/*! GPIO layer pin type. */
typedef void * GPIO_pin_t;

/*! GPIO direction definition*/
typedef enum
{
	/*! Set current pin as input. */
    Pin_Input  = 0U,

	/*! Set current pin as output. */
    Pin_Output = 1U,

} GPIO_pin_direction_t;

#if defined(CPU_MKL17Z256VFM4)

extern GPIO_pin_t Pin_PTA01;			/*!< Pin: Port A, Pin 1 */
extern GPIO_pin_t Pin_PTA02;			/*!< Pin: Port A, Pin 2 */

extern GPIO_pin_t Pin_PTE16;			/*!< Pin: Port E, Pin 16 */
extern GPIO_pin_t Pin_PTE17;			/*!< Pin: Port E, Pin 17 */
extern GPIO_pin_t Pin_PTE18;			/*!< Pin: Port E, Pin 18 */
extern GPIO_pin_t Pin_PTE19;			/*!< Pin: Port E, Pin 19 */

/* S2PI Interface Pins */
extern GPIO_pin_t Pin_S2PI_MISO; 		/*!< Pin: S2PI: MISO, master-in-slave-out */
extern GPIO_pin_t Pin_S2PI_MOSI; 		/*!< Pin: S2PI: MOSI, master-out-slave-in */
extern GPIO_pin_t Pin_S2PI_CLK; 		/*!< Pin: S2PI: CLK, clock */
extern GPIO_pin_t Pin_S2PI_IRQ1; 		/*!< Pin: S2PI: IRQ1, slave 1 interrupt */
extern GPIO_pin_t Pin_S2PI_CS1;			/*!< Pin: S2PI: CS1, slave 1 chip select */



#elif defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)

/* Board Specific Pins */
#if defined (BRIDGE)
extern GPIO_pin_t Pin_Jumper1;			/*!< Pin: Jumper 1 */
extern GPIO_pin_t Pin_Jumper2;			/*!< Pin: Jumper 2 */
#endif

extern GPIO_pin_t Pin_LED_Green;		/*!< Pin: Green LED */
extern GPIO_pin_t Pin_LED_Red;			/*!< Pin: Red LED   */
extern GPIO_pin_t Pin_SW1;				/*!< Pin: Switch 1  */
extern GPIO_pin_t Pin_SW3;				/*!< Pin: Switch 2  */

extern GPIO_pin_t Pin_PTB0;				/*!< Pin: PortB Pin0 */
extern GPIO_pin_t Pin_PTB1;				/*!< Pin: PortB Pin1 */
extern GPIO_pin_t Pin_PTB2;				/*!< Pin: PortB Pin2 */
extern GPIO_pin_t Pin_PTB3;				/*!< Pin: PortB Pin3 */
extern GPIO_pin_t Pin_PTC1;				/*!< Pin: PortC Pin1 */
extern GPIO_pin_t Pin_PTC2;				/*!< Pin: PortC Pin2 */

/* S2PI Interface Pins */
extern GPIO_pin_t Pin_S2PI_MISO; 		/*!< Pin: S2PI: MISO, master-in-slave-out */
extern GPIO_pin_t Pin_S2PI_MOSI; 		/*!< Pin: S2PI: MOSI, master-out-slave-in */
extern GPIO_pin_t Pin_S2PI_CLK; 		/*!< Pin: S2PI: CLK, clock */
extern GPIO_pin_t Pin_S2PI_IRQ1; 		/*!< Pin: S2PI: IRQ1, slave 1 interrupt */
extern GPIO_pin_t Pin_S2PI_IRQ2; 		/*!< Pin: S2PI: IRQ2, slave 2 interrupt */
extern GPIO_pin_t Pin_S2PI_IRQ3; 		/*!< Pin: S2PI: IRQ3, slave 3 interrupt */
extern GPIO_pin_t Pin_S2PI_IRQ4; 		/*!< Pin: S2PI: IRQ4, slave 4 interrupt */
extern GPIO_pin_t Pin_S2PI_CS1;			/*!< Pin: S2PI: CS1, slave 1 chip select */
extern GPIO_pin_t Pin_S2PI_CS2;			/*!< Pin: S2PI: CS2, slave 2 chip select */
extern GPIO_pin_t Pin_S2PI_CS3;			/*!< Pin: S2PI: CS3, slave 3 chip select */
extern GPIO_pin_t Pin_S2PI_CS4;			/*!< Pin: S2PI: CS4, slave 4 chip select */

#endif

/*!***************************************************************************
 * @brief	Initializes the GPIO driver and does pin muxing.
 * @details	The first call to this function does the initialization. Every
 * 			further call does not have an effect and just returns.
 * 			The clock to all ports are ungated and the interrupts are enabled.
 * 			The pins are categorized into interrupt-enabled pins and common
 * 			input output pins. They are are initialized with the following setup:
 * 				- Low drive strength.
 * 				- Slow slew rate.
 * 				- Passive filter disabled.
 * 				- Pull disable (common) or pull up (interrupt-enable).
 * 				- Pin muxing as GPIO.
 * 				- Default state for output pins is low (0).
 * 				- IRQ for interrupt-enable pins disabled (use #GPIO_SetISR to
 * 				  enable and intall a callback).
 * 				.
 *****************************************************************************/
void GPIO_Init(void);

/*!***************************************************************************
 * @brief	Set the output of an GPIO pin to high level.
 * @param	pin The address of the GPIO pin.
 *****************************************************************************/
void GPIO_SetPinOutput(GPIO_pin_t pin);

/*!***************************************************************************
 * @brief	Clears the output of an GPIO pin to low level.
 * @param	pin The address of the GPIO pin.
 *****************************************************************************/
void GPIO_ClearPinOutput(GPIO_pin_t pin);

/*!***************************************************************************
 * @brief	Toggles the output of an GPIO pin to the inverse level.
 * @param	pin The address of the GPIO pin.
 *****************************************************************************/
void GPIO_TogglePinOutput(GPIO_pin_t pin);

/*!***************************************************************************
 * @brief	Writes the output of an GPIO pin corresponding to the parameter.
 * @param	pin The address of the GPIO pin.
 * @param	value==0: clear pin; !=0: set pin.
 *****************************************************************************/
void GPIO_WritePinOutput(GPIO_pin_t pin, uint32_t value);

/*!***************************************************************************
 * @brief	Read the input of an GPIO pin.
 * @param	pin The address of the GPIO pin.
 * @return	0 for low level, 1 for high level input signal.
 *****************************************************************************/
uint32_t GPIO_ReadPinInput(GPIO_pin_t pin);


/*!***************************************************************************
 * @brief	Sets the direction (input/output) of an GPIO pin.
 * @param	pin The address of the GPIO pin.
 * @param	dir The direction.
 *****************************************************************************/
void GPIO_SetPinDir(GPIO_pin_t pin, GPIO_pin_direction_t dir);

/*!***************************************************************************
 * @brief	Sets an interrupt callback for an specified pin.
 * @param	pin The address of the GPIO pin.
 * @param	f The callback function pointer.
 * @param	p A void pointer to be passed to the callback function.
 *****************************************************************************/
void GPIO_SetISR(GPIO_pin_t pin, GPIO_isr_t f, void * p);

/*!***************************************************************************
 * @brief	Removes the interrupt callback.
 * @param	pin The address of the GPIO pin.
 *****************************************************************************/
void GPIO_RemoveISR(GPIO_pin_t pin);

/*!***************************************************************************
 * @brief	Sets the pin muxing for a specified pin.
 * @param	pin The address of the GPIO pin.
 * @param	mux The pin muxing slot selection.
 *  				- 0: Pin disabled or work in analog function.
 *        			- 1: Set as GPIO.
 *        			- 2-7: chip-specific.
 *****************************************************************************/
void GPIO_SetPinMux(GPIO_pin_t pin, uint32_t mux);

/*!***************************************************************************
 * @brief	Selects the internal pin pull-up for a specified pin.
 * @param	pin The address of the GPIO pin.
 *****************************************************************************/
void GPIO_SetPinPullUp(GPIO_pin_t pin);

/*!***************************************************************************
 * @brief	Selects the internal pin pull-down for a specified pin.
 * @param	pin The address of the GPIO pin.
 *****************************************************************************/
void GPIO_SetPinPullDown(GPIO_pin_t pin);

/*!***************************************************************************
 * @brief	Selects the internal pin pull-disable for a specified pin.
 * @param	pin The address of the GPIO pin.
 *****************************************************************************/
void GPIO_SetPinPullDisable(GPIO_pin_t pin);

#ifdef __cplusplus
}
#endif

/*! @} */
#endif /* GPIO_H */
