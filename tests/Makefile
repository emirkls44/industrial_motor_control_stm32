# Host-side unit tests (no hardware needed): `make -C tests`
CC     ?= cc
CFLAGS += -std=c11 -Wall -Wextra -Werror -pedantic -g -fsanitize=address,undefined

TESTS := test_protocol test_motor_control

test: $(TESTS)
	@for t in $(TESTS); do echo "== $$t"; ./$$t || exit 1; echo; done

test_protocol: test_protocol.c ../Core/Src/protocol.c ../Core/Inc/protocol.h
	$(CC) $(CFLAGS) -I../Core/Inc test_protocol.c ../Core/Src/protocol.c -o $@

# fakes/ provides a minimal stm32f0xx_hal.h so the module builds on a PC
test_motor_control: test_motor_control.c ../Core/Src/motor_control.c ../Core/Inc/motor_control.h
	$(CC) $(CFLAGS) -Ifakes -I../Core/Inc test_motor_control.c ../Core/Src/motor_control.c -o $@

clean:
	rm -f $(TESTS)

.PHONY: test clean
