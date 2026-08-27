#include "button.h"

void Button_Init(Button_t *button, GPIO_TypeDef *port, uint16_t pin,
		GPIO_PinState active_state, uint32_t debounce_ms) {
	if ((button == NULL) || (port == NULL)) {
		return;
	}

	const GPIO_PinState initial_state = HAL_GPIO_ReadPin(port, pin);

	button->port = port;
	button->pin = pin;
	button->active_state = active_state;
	button->stable_state = initial_state;
	button->last_raw_state = initial_state;
	button->last_change_tick = HAL_GetTick();
	button->debounce_ms = debounce_ms;
	button->pressed_event = 0U;
}

void Button_Update(Button_t *button) {
	if ((button == NULL) || (button->port == NULL)) {
		return;
	}

	const uint32_t now = HAL_GetTick();
	const GPIO_PinState raw_state = HAL_GPIO_ReadPin(button->port, button->pin);

	if (raw_state != button->last_raw_state) {
		button->last_raw_state = raw_state;
		button->last_change_tick = now;
	}

	if (((now - button->last_change_tick) >= button->debounce_ms)
			&& (button->stable_state != raw_state)) {
		button->stable_state = raw_state;

		if (button->stable_state == button->active_state) {
			button->pressed_event = 1U;
		}
	}
}

uint8_t Button_IsPressed(const Button_t *button) {
	if (button == NULL) {
		return 0U;
	}

	return (button->stable_state == button->active_state) ? 1U : 0U;
}

uint8_t Button_WasPressed(Button_t *button) {
	if (button == NULL) {
		return 0U;
	}

	const uint8_t event = button->pressed_event;
	button->pressed_event = 0U;

	return event;
}
