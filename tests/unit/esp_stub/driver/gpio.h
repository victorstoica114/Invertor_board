#pragma once

typedef int gpio_num_t;

static inline int gpio_set_level(gpio_num_t gpio_num, int level)
{
    (void)gpio_num;
    (void)level;
    return 0;
}

