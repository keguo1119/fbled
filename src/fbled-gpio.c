#include "fbled-gpio.h"
#include "fbled-algo-bit.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/of_device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/delay.h>

#define FBLED_MAJOR 235

struct fbled_gpio_private_data {
    struct fbled_gpio_platform_data pdata;
    struct fbled_algo_bit_data bit_data;
};

struct fbled_dev {
	struct list_head list;
//	struct fbled_adapter *adap;
	struct device *dev;
	struct cdev cdev;
	struct fbled_gpio_platform_data pdata;
	struct fbled_algo_bit_data bit_data;
	struct mutex mutex;
};

struct fbled_dev         *fbled_dev;


static void led_gpio_setdir_in(unsigned gpio)
{
	gpio_direction_input(gpio);
}

static void led_gpio_setdir_out(unsigned gpio)
{
	gpio_direction_output(gpio, 0);
}

static void led_gpio_setval(unsigned gpio, int state)
{
	gpio_set_value(gpio, state);
}

static int led_gpio_getval(unsigned gpio)
{
	return gpio_get_value(gpio);
}

static int test_out(struct fbled_dev *dev)
{
    int i;
	struct fbled_gpio_platform_data *gpio = &dev->pdata;
	struct fbled_algo_bit_data		*bit_data = &dev->bit_data;
//    if(!gpio->mode)
//        gpio_direction_output(gpio->gpiod, 0);
	printk(KERN_INFO "using pin %u (DAT), %u (WR), %u (RD) and %u (CS), %d (udelay) \n", \
		gpio->dat_pin, gpio->wr_pin, gpio->rd_pin, gpio->cs_pin, gpio->udelay);

    for(i = 0; i < 32; i++) {
		gpio_set_value(gpio->cs_pin, 1);
		gpio_set_value(gpio->dat_pin,1);
		gpio_set_value(gpio->wr_pin, 1);
        udelay(gpio->udelay);
		gpio_set_value(gpio->cs_pin, 0);
		gpio_set_value(gpio->dat_pin,0);
		gpio_set_value(gpio->wr_pin, 0);
        udelay(gpio->udelay);
	}
	
	udelay(100);

	for(i = 0; i < 32; i++) {
		bit_data->gpio_setval(bit_data->data->cs_pin, 1);
		bit_data->gpio_setval(bit_data->data->dat_pin, 1);
		bit_data->gpio_setval(bit_data->data->wr_pin, 1);
		udelay(bit_data->udelay);
		bit_data->gpio_setval(bit_data->data->cs_pin, 0);
		bit_data->gpio_setval(bit_data->data->dat_pin, 0);
		bit_data->gpio_setval(bit_data->data->wr_pin, 0);
		udelay(bit_data->udelay);
    }

	udelay(100);

	bit_data->fbled_test(bit_data);

    return 0;
}

static int of_fbled_gpio_get_pins(struct device_node *np, 
                unsigned int *dat_pin, unsigned int *wr_pin,
                unsigned int *rd_pin,  unsigned int *cs_pin)
{
    if (of_gpio_count(np) < 4)
		return -ENODEV;

	*dat_pin = of_get_gpio(np, 0);
	*wr_pin  = of_get_gpio(np, 1);
    *rd_pin  = of_get_gpio(np, 2);
	*cs_pin  = of_get_gpio(np, 3);

	if (*dat_pin == -EPROBE_DEFER || *wr_pin == -EPROBE_DEFER 
          || *rd_pin == -EPROBE_DEFER || *cs_pin == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if (!gpio_is_valid(*dat_pin) || !gpio_is_valid(*wr_pin)) {
		pr_err("%s: invalid GPIO pins, dat_pin=%d/wr_pin=%d\n",
		       np->full_name, *dat_pin, *wr_pin);
		return -ENODEV;
	}

    if (!gpio_is_valid(*rd_pin) || !gpio_is_valid(*cs_pin)) {
		pr_err("%s: invalid GPIO pins, rd_pin=%d/cs_pin=%d\n",
		       np->full_name, *rd_pin, *cs_pin);
		return -ENODEV;
	}

	return 0;

}

static void of_fbled_gpio_get_props(struct device_node *np,
                struct fbled_gpio_platform_data *pdata)
{
    u32 reg;

    of_property_read_u32(np, "fbled-gpio,delay-us", &pdata->udelay);
    
    if (!of_property_read_u32(np, "fbled-gpio,timeout-ms", &reg))
		pdata->timeout = msecs_to_jiffies(reg);
}

//////////////////////////////////////////////////////////////////
static ssize_t fbled_read(struct file *filp, char __user *buf,
			       size_t count, loff_t *ppos)
{
	char read_buf[256];

	sprintf(read_buf, "fbled read test %s", "fjisap");
	printk(KERN_INFO "%s-read, read_buf=%s, len=%d\n", __func__, read_buf, strlen(read_buf));
	copy_to_user(buf, read_buf, strlen(read_buf));
    return 0;
}

static ssize_t fbled_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *ppos)
{
    return 0;
}

static int fbled_ioctl_rdwr(struct fbled_dev *fbled_devp, unsigned long arg)
{
		struct fbled_msg msg;

		printk(KERN_INFO "0-%s-set\n", __func__);
		if(copy_from_user(&msg, (struct fbled_msg *)arg, sizeof(msg)))
			return -EFAULT;
		printk(KERN_INFO "1-%s-set\n", __func__);
		fbled_transfer(&fbled_devp->bit_data, &msg);

		return 0;

}

static long fbled_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{

	switch (cmd) {
	case FBLED_RDWR:
		return fbled_ioctl_rdwr(fbled_dev, arg);
	}
    return 0;
}

static int fbled_open(struct inode *inode, struct file *filp)
{

	printk(KERN_INFO "%s-open 1\n", __func__);
	return 0;
}

static int fbled_release(struct inode *inode, struct file *filp)
{
	return 0;
}


static const struct file_operations fbleddev_fops = {
	.owner      = THIS_MODULE,
//	.llseek		= no_llseek,
	.read		= fbled_read,
	.write		= fbled_write,
	.unlocked_ioctl	= fbled_ioctl,
	.open		= fbled_open,
	.release	= fbled_release,
};

static ssize_t name_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "fbled");
}
static DEVICE_ATTR_RO(name);

static struct attribute *fbled_name_attrs[] = {
	&dev_attr_name.attr,
	NULL,
};

static const struct attribute_group fbled_name_group = {
	.attrs = fbled_name_attrs,
};


ssize_t trigger_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	int cnt = -1;

	struct second_dev *sdev = dev_get_drvdata(dev);

 	return sprintf(buf, "fbled_trigger_show: \n");
//    return sprintf(buf, "second_trigger_show: count = %d, counter_addr=%p\n", *counter, counter);
}

ssize_t trigger_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct fbled_dev *data = dev_get_drvdata(dev);
	struct fbled_gpio_platform_data *gpio = &data->pdata;
	unsigned long state = 0;
	ssize_t			status;
	int ret;

	printk(KERN_INFO "%s: 0 - mutex lock\n", __func__);
	mutex_lock(&data->mutex);
	
	if (sysfs_streq(buf, "test"))
		test_out(data);
/*
		status = gpiod_direction_output_raw(desc, 1);
	else if (sysfs_streq(buf, "out") || sysfs_streq(buf, "low"))
		status = gpiod_direction_output_raw(desc, 0);
	else if (sysfs_streq(buf, "in"))
		status = gpiod_direction_input(desc);
	else
		status = -EINVAL;
*/
	printk(KERN_INFO "%s: 1 - mutex lock, state = %d\n", __func__, state);
	unlock:
	mutex_unlock(&data->mutex);

return status ? : size;
}

static DEVICE_ATTR_RW(trigger);
//static DEVICE_ATTR(trigger, 0644, trigger_show, trigger_store);
static struct attribute *fbled_trigger_attrs[] = {
	&dev_attr_trigger.attr,
	NULL,
};
static const struct attribute_group fbled_trigger_group = {
	.attrs = fbled_trigger_attrs,
};

static const struct attribute_group *fbled_groups[] = {
	&fbled_trigger_group,
	&fbled_name_group,
    NULL,
};

static struct class *fbled_dev_class;
////////////////////////////////////////////////////////////////


static int fbled_gpio_probe(struct platform_device *pdev)
{
//    struct fbled_gpio_private_data  *priv;
    struct fbled_gpio_platform_data *pdata;
    struct fbled_algo_bit_data      *bit_data;
//    struct fbled_dev                *fbled_dev;
    unsigned int dat_pin, wr_pin, rd_pin, cs_pin;
    int ret;

    if(pdev->dev.of_node) {
        ret = of_fbled_gpio_get_pins(pdev->dev.of_node, 
                        &dat_pin, &wr_pin, &rd_pin, &cs_pin);
        if(ret)
            return ret;
    } else {
        if(!dev_get_platdata(&pdev->dev))
            return -ENXIO;       
        pdata = dev_get_platdata(&pdev->dev);
        dat_pin = pdata->dat_pin;
        wr_pin  = pdata->wr_pin;
        rd_pin  = pdata->rd_pin;
        cs_pin  = pdata->cs_pin;
    }

    ret = devm_gpio_request(&pdev->dev, dat_pin, "dat");
	if (ret) {
		if (ret == -EINVAL)
			ret = -EPROBE_DEFER;	/* Try again later */
		return ret;
	}
	ret = devm_gpio_request(&pdev->dev, wr_pin, "wr");
	if (ret) {
		if (ret == -EINVAL)
			ret = -EPROBE_DEFER;	/* Try again later */
		return ret;
	}
    ret = devm_gpio_request(&pdev->dev, rd_pin, "rd");
	if (ret) {
		if (ret == -EINVAL)
			ret = -EPROBE_DEFER;	/* Try again later */
		return ret;
	}
	ret = devm_gpio_request(&pdev->dev, cs_pin, "cs");
	if (ret) {
		if (ret == -EINVAL)
			ret = -EPROBE_DEFER;	/* Try again later */
		return ret;
	}

//    priv =  devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
 //   if(!priv)
   //     return -ENOMEM;
    fbled_dev = devm_kzalloc(&pdev->dev, sizeof(*fbled_dev), GFP_KERNEL);
    if(!fbled_dev) {
        ret = -ENOMEM;
//        goto fail_malloc;
    }
    
    bit_data = &fbled_dev->bit_data;
    pdata    = &fbled_dev->pdata;

    if(pdev->dev.of_node) {
        pdata->dat_pin  =  dat_pin;
        pdata->wr_pin   =  wr_pin;
        pdata->rd_pin   = rd_pin;
        pdata->cs_pin   = cs_pin; 
        of_fbled_gpio_get_props(pdev->dev.of_node, pdata);  
    } else {
        memcpy(pdata, dev_get_platdata(&pdev->dev), sizeof(*pdata));
    }

    if(pdata->udelay)
        bit_data->udelay = pdata->udelay;
    else {
		bit_data->udelay = 10;
		pdata->udelay    = 10;
	}
     
    if (pdata->timeout)
		bit_data->timeout = pdata->timeout;
	else
		bit_data->timeout = HZ / 10;		/* 100 ms */

    bit_data->data = pdata;
    
    bit_data->gpio_setdir_in  = led_gpio_setdir_in;
    bit_data->gpio_setdir_out = led_gpio_setdir_out;
    bit_data->gpio_setval     = led_gpio_setval;
    bit_data->gpio_getval     = led_gpio_getval;
	bit_data->fbled_test	  = fbled_test;

    platform_set_drvdata(pdev, fbled_dev);

    dev_info(&pdev->dev, "using pin %u (DAT), %u (WR), %u (RD) and %u (CS), %d (udelay) \n", \
		pdata->dat_pin, pdata->wr_pin, pdata->rd_pin, pdata->cs_pin, bit_data->udelay);
    
//////////////////////////////////////
////fbled dev set
/////////////////////////////////////

 //   fbled_dev = devm_kzalloc(&pdev->dev, sizeof(*fbled_dev), GFP_KERNEL);
  //  if(!fbled_dev) {
  //      ret = -ENOMEM;
//        goto fail_malloc;
  //  }
  	mutex_init(&fbled_dev->mutex);
    cdev_init(&fbled_dev->cdev, &fbleddev_fops);
    fbled_dev->cdev.owner = THIS_MODULE;
    ret = cdev_add(&fbled_dev->cdev, MKDEV(FBLED_MAJOR, 0), 1);

    fbled_dev->dev = device_create(fbled_dev_class, &pdev->dev, 
    MKDEV(FBLED_MAJOR, 0), fbled_dev, "fbled");	
	
	fbled_init(bit_data);
    
	dev_info(&pdev->dev, "fbled drv proded\n");
    return 0;
}

static int fbled_gpio_remove(struct platform_device *pdev)
{
//	struct fbled_gpio_private_data *priv;   

	return 0;
}

static struct platform_driver fbled_gpio_driver = {
	.driver		= {
		.name	= "fbled-gpio",
        .owner = THIS_MODULE, 
	},
	.probe		= fbled_gpio_probe,
	.remove		= fbled_gpio_remove,
};

static int __init fbled_gpio_init(void)
{
    int res, ret;

	printk(KERN_INFO "fbled /dev entries driver\n");

	res = register_chrdev_region(MKDEV(FBLED_MAJOR, 0), 1, "fbled-gpio");
	if (res)
		goto out;

	fbled_dev_class = class_create(THIS_MODULE, "fbled");
	if (IS_ERR(fbled_dev_class)) {
		res = PTR_ERR(fbled_dev_class);
		goto out_unreg_chrdev;
	}
	fbled_dev_class->dev_groups = fbled_groups;

    ret = platform_driver_register(&fbled_gpio_driver);
	if (ret)
		printk(KERN_ERR "fbled-gpio: probe failed: %d\n", ret);

	return 0;

out_unreg_chrdev:
	unregister_chrdev_region(MKDEV(FBLED_MAJOR, 0), 1);
out:
	printk(KERN_ERR "%s: Driver Initialisation failed\n", __FILE__);
	return res;
}
subsys_initcall(fbled_gpio_init);

static void __exit fbled_gpio_exit(void)
{
    if(fbled_dev)
       cdev_del(&fbled_dev->cdev);
	device_destroy(fbled_dev_class, MKDEV(FBLED_MAJOR, 0));
	class_destroy(fbled_dev_class);
	unregister_chrdev_region(MKDEV(FBLED_MAJOR, 0), 1);
	platform_driver_unregister(&fbled_gpio_driver);
}
module_exit(fbled_gpio_exit);

MODULE_AUTHOR("KG (Uooly)");
MODULE_DESCRIPTION("Platform-independent bitbanging fbled driver");
MODULE_LICENSE("GPL");
