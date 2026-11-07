#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

/* Independent watchdog (IWDG, clocked by the ~40 kHz LSI). Once started it
 * cannot be stopped; the main loop must call Watchdog_Kick() regularly. */
void Watchdog_Start(uint32_t timeout_ms);
void Watchdog_Kick(void);

/* Returns true if the last reset was caused by the IWDG, then clears all
 * reset flags. Call once at start-up. */
bool Watchdog_ConsumeResetFlag(void);

#endif /* WATCHDOG_H */
