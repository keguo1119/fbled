#ifndef _LINUX_FBLED_ALGO_BIT_H
#define _LINUX_FBLED_ALGO_BIT_H

#include "fbled-gpio.h"

struct fbled_algo_bit_data {
    struct fbled_gpio_platform_data *data;
    void (*gpio_setdir_in)( unsigned gpio);
    void (*gpio_setdir_out)(unsigned gpio);
    void (*gpio_setval)(unsigned gpio, int state);
    int (*gpio_getval)(unsigned gpio);
    void (*fbled_test)(struct fbled_algo_bit_data *bit_data);

    int udelay;
    int timeout;
};

void fbled_init(struct fbled_algo_bit_data *pdata);
int fbled_transfer(struct fbled_algo_bit_data *pdata, struct fbled_msg *msgs);	
void fbled_test(struct fbled_algo_bit_data *bit_data);

#endif

