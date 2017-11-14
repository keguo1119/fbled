#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include "fbled-gpio.h"

#define BUS_PARAM_ID		0
#define BUS_PARAM_DAT		1
#define BUS_PARAM_WR		2
#define BUS_PARAM_RD		3
#define BUS_PARAM_CS		4
#define BUS_PARAM_UDELAY	5
#define BUS_PARAM_TIMEOUT	6

#define BUS_PARAM_COUNT		7
#define BUS_PARAM_REQUIRED	5
#define BUS_COUNT_MAX		4

#define DRV_NAME   "fbled-gpio-custom"
#define DRV_DESC	"Custom GPIO-based FBLED driver"
#define DRV_VERSION	"0.1.1" 

#define PFX		DRV_NAME ": "

#define  BUS_PARM_DESC \
	" config -> id,dat,wr,rd,cs[,udelay,timeout]"

static unsigned int bus0[BUS_PARAM_COUNT] __initdata;
static unsigned int bus_nump[BUS_COUNT_MAX] __initdata;

module_param_array(bus0, uint, &bus_nump[0], 0);
MODULE_PARM_DESC(bus0, "bus0" BUS_PARM_DESC);

static struct platform_device *devices[BUS_COUNT_MAX];
static unsigned int nr_devices;

static void fbled_gpio_custom_cleanup(void)
{
	int i;

	for (i = 0; i < nr_devices; i++)
		if (devices[i])
			platform_device_unregister(devices[i]);
}


static int __init fbled_gpio_custom_probe(void)
{
    struct platform_device *pdev;
	struct fbled_gpio_platform_data pdata;

	int err;

	printk(KERN_INFO DRV_DESC " version " DRV_VERSION "\n");

    if (bus_nump[0] < BUS_PARAM_REQUIRED) {
		printk(KERN_ERR PFX "not enough parameters for bus\n");
		err = -EINVAL;
		goto err;
	}

    pdev = platform_device_alloc("fbled-gpio", bus0[BUS_PARAM_ID]);
	if (!pdev) {
		err = -ENOMEM;
		goto err;
	}

    pdata.dat_pin  =  bus0[BUS_PARAM_DAT];
    pdata.wr_pin   =  bus0[BUS_PARAM_WR];
    pdata.rd_pin   =  bus0[BUS_PARAM_RD];
    pdata.cs_pin   =  bus0[BUS_PARAM_CS];
    pdata.udelay   =  bus0[BUS_PARAM_UDELAY];
    pdata.timeout  =  bus0[BUS_PARAM_TIMEOUT];

    err = platform_device_add_data(pdev, &pdata, sizeof(pdata));
	if (err)
		goto err_put;

	err = platform_device_add(pdev);
	if (err)
		goto err_put;

	devices[nr_devices++] = pdev;
	return 0;

err_put:
	platform_device_put(pdev);
err:
	return err;
}

static int __init fbled_gpio_custom_init(void)
{
	return fbled_gpio_custom_probe();
}
module_init(fbled_gpio_custom_init);

static void __exit fbled_gpio_custom_exit(void)
{
	fbled_gpio_custom_cleanup();
}
module_exit(fbled_gpio_custom_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Gabor Juhos <juhosg@openwrt.org >");
MODULE_DESCRIPTION(DRV_DESC);
MODULE_VERSION(DRV_VERSION);