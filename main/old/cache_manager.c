#include <stdio.h>
#include <stdlib.h>
#include <time.h>

typedef struct {
    int register_value;
    time_t timestamp;
} CachedValue;

CachedValue cache;

void store_register_value(int value) {
    cache.register_value = value;
    cache.timestamp = time(NULL);
}

int retrieve_register_value() {
    return cache.register_value;
}

int validate_register_value() {
    time_t current_time = time(NULL);
    // If more than 10 seconds have passed since the value was stored, consider it invalid
    if (difftime(current_time, cache.timestamp) > 10) {
        printf("Register value is outdated.\n");
        return 0; // invalid
    }
    return 1; // valid
}

int main() {
    store_register_value(42);
    if (validate_register_value()) {
        printf("Valid register value: %d\n", retrieve_register_value());
    } else {
        printf("Register value is invalid.\n");
    }
    return 0;
}