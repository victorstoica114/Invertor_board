#pragma once

// Some ESP-IDF headers rely on these synchronization primitives.
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0

#define portENTER_CRITICAL(mux) ((void)(mux))
#define portEXIT_CRITICAL(mux) ((void)(mux))

