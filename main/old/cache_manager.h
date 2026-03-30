# Cache Manager Header

#ifndef CACHE_MANAGER_H
#define CACHE_MANAGER_H

#include <stdbool.h>

// Define the structure for a cached register
typedef struct {
    int register_id;
    void *value;
    bool is_fresh;
    time_t last_updated;
} CachedRegister;

// API function to initialize the cache
void initialize_cache();

// API function to retrieve a register from the cache
CachedRegister* get_register(int register_id);

// API function to update a register in the cache
void update_register(int register_id, void *new_value);

// API function to check the freshness of a register
bool is_register_fresh(int register_id);

// API function to clear cache
void clear_cache();

#endif // CACHE_MANAGER_H
