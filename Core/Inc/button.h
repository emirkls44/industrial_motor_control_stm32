#ifndef BUTTON_H
#define BUTTON_H

#include "stm32f0xx_hal.h"

typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
	GPIO_PinState active_state;
	GPIO_PinState stable_state;
	GPIO_PinState last_raw_state;
	uint32_t last_change_tick;
	uint32_t debounce_ms;
	uint8_t pressed_event;
} Button_t;

void Button_Init(Button_t *button, GPIO_TypeDef *port, uint16_t pin,
		GPIO_PinState active_state, uint32_t debounce_ms);

void Button_Update(Button_t *button);
uint8_t Button_IsPressed(const Button_t *button);
uint8_t Button_WasPressed(Button_t *button);

#endif /* BUTTON_H */
