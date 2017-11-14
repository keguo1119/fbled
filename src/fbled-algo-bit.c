#include "fbled-gpio.h"
#include "fbled-algo-bit.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>


#define MOD_READ			0xC0			//¶ÁÊý¾ÝÄ£Ê½
#define MOD_WRITE			0xA0			//Ð´Êý¾ÝÄ£Ê½
#define MOD_RMW				0xA0			//¶ÁÐÞ¸ÄÐ´Ä£Ê½
#define MOD_CMD				0x80			//ÃüÁîÄ£Ê½


#define SYS_DIS				0x00			//¹Ø±ÕÏµÍ³Ê±ÖÓ
#define SYS_EN				0x01			//´ò¿ªÏµÍ³Ê±ÖÓ
#define LED_OFF				0x02			//¹Ø±ÕLEDÏÔÊ¾
#define LED_ON				0x03			//´ò¿ªLEDÏÔÊ¾
#define BLINK_OFF			0x08		   	//¹Ø±ÕÉÁË¸
#define	BLINK_ON			0X09		   	//´ò¿ªÉÁË¸
#define	SLAVE_MODE			0X10		   	//´ÓÄ£Ê½
#define	RC_MASTER_MODE		0X18	   		//ÄÚ²¿RCÊ±ÖÓ
#define	EXT_CLK_MASTER_MODE	0X1C   			//Íâ²¿Ê±ÖÓ
#define COM_OPTION			0X24		   	//16COM£¬NMOS¿ªÂ©Ä£Ê½ 0x2C£º16COM£¬PMOS¿ªÂ©Êä³ö
#define PWM_DUTY			0XA0			//PWMÁÁ¶È¿ØÖÆ


#ifdef DEBUG
#define bit_dbg(level, dev, format, args...) \
	do { \
		if (i2c_debug >= level) \
			dev_dbg(dev, format, ##args); \
	} while (0)
#else
#define bit_dbg(level, dev, format, args...) \
	do {} while (0)
#endif /* DEBUG */


void PrintHex(
   void *      pBuffer,
   u16         bufSize )
{
   char * pPrintBuf;
   u16 pos;
   int status;
   
   pPrintBuf = kmalloc( bufSize * 3 + 1, GFP_ATOMIC );
   if (pPrintBuf == NULL)
   {
      DBG( "Unable to allocate buffer\n" );
      return;
   }
   memset( pPrintBuf, 0 , bufSize * 3 + 1 );
   
   for (pos = 0; pos < bufSize; pos++)
   {
      status = snprintf( (pPrintBuf + (pos * 3)), 
                         4, 
                         "%02X ", 
                         *(u8 *)(pBuffer + pos) );
      if (status != 3)
      {
         DBG( "snprintf error %d\n", status );
         kfree( pPrintBuf );
         return;
      }
   }
   
   DBG( "   : %s\n", pPrintBuf );

   kfree( pPrintBuf );
   pPrintBuf = NULL;
   return;   
}

static int bit_test;
module_param(bit_test, int, S_IRUGO);
MODULE_PARM_DESC(bit_test, "lines testing - 0 off; 1 report; 2 fail if stuck");

#ifdef DEBUG
static int fbled_debug = 1;
module_param(i2c_debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(i2c_debug,
		 "debug level - 0 off; 1 normal; 2 verbose; 3 very verbose");
#endif

static void fbled_io_init(struct fbled_algo_bit_data *pdata)
{
    pdata->gpio_setdir_out(pdata->data->cs_pin);
    pdata->gpio_setval(pdata->data->cs_pin, 1);
    
    pdata->gpio_setdir_out(pdata->data->dat_pin);
    pdata->gpio_setval(pdata->data->dat_pin, 1);

    pdata->gpio_setdir_out(pdata->data->wr_pin);
    pdata->gpio_setval(pdata->data->wr_pin, 1);

    pdata->gpio_setdir_in(pdata->data->rd_pin);
//        pdata->gpio_setval(pdata->cs_pin, 1);
}

static inline void cslo(struct fbled_algo_bit_data *pdata)
{
    pdata->gpio_setval(pdata->data->cs_pin, 0);
    udelay((pdata->udelay + 1) / 2);
}

static inline void cshi(struct fbled_algo_bit_data *pdata)
{
    pdata->gpio_setval(pdata->data->cs_pin, 1);
    udelay((pdata->udelay + 1) / 2);
}

static inline void datlo(struct fbled_algo_bit_data *pdata)
{
    pdata->gpio_setval(pdata->data->dat_pin, 0);
    udelay((pdata->udelay + 1) / 2);
}

static inline void dathi(struct fbled_algo_bit_data *pdata)
{
    pdata->gpio_setval(pdata->data->dat_pin, 1);
    udelay((pdata->udelay + 1) / 2);
}

static inline void wrlo(struct fbled_algo_bit_data *pdata)
{
    pdata->gpio_setval(pdata->data->wr_pin, 0);
    udelay((pdata->udelay + 1) / 2);
}

static inline void wrhi(struct fbled_algo_bit_data *pdata)
{
    pdata->gpio_setval(pdata->data->wr_pin, 1);
    udelay((pdata->udelay + 1) / 2);
}

static int fbled_outb(struct fbled_algo_bit_data *pdata, unsigned char c, int cnt)
{
    int i;

    for(i = 0; i < cnt; i++) {
        wrlo(pdata);
        if(c & 0x80)
            dathi(pdata);
        else
            datlo(pdata);
        c<<=1;
        wrhi(pdata);
    }

    return 0;
}

static int fbled_cmd(struct fbled_algo_bit_data *pdata, unsigned char cmd)
{
    cslo(pdata);
    fbled_outb(pdata, MOD_CMD, 3);
    fbled_outb(pdata, cmd, 9);
    cshi(pdata);

    return 0;
}

void fbled_row_upload(struct fbled_algo_bit_data *pdata, unsigned char row,unsigned char *p, int n)	
{
    int i;

    if(row < 1)
        return ;

	row=(row-1)<<2;							//??????

	cslo(pdata);
	fbled_outb(pdata, MOD_WRITE,3);
	fbled_outb(pdata, row<<1,7);
	for(i=0; i < n; i++)
        fbled_outb(pdata, *(p + i), 8);
	cshi(pdata);
}

void fbled_page_upload(struct fbled_algo_bit_data *pdata, unsigned char *p)
{
	unsigned char i;

	cslo(pdata);
	fbled_outb(pdata, MOD_WRITE,3);
	fbled_outb(pdata, 0,7);
	for(i=0;i<48;i++)
    {
        fbled_outb(pdata, *p,8);
        p++;
    }
	cshi(pdata);
} 

static int fbled_xfer(struct fbled_algo_bit_data *bit_data, struct fbled_msg *msgs)
{
    struct fbled_msg *pmsg = msgs;

    printk(KERN_INFO "0-%s-set\n", __func__);
    if(pmsg->flags & FBLED_M_WR_PAGE) {
        printk(KERN_INFO "%s-read,  len=%d\n", __func__, pmsg->len);
        fbled_page_upload(bit_data, pmsg->buf);
        PrintHex(pmsg->buf, pmsg->len);
    } else if(pmsg->flags & FBLED_M_WR_ROW) {
        printk(KERN_INFO "%s-read,  len=%d\n", __func__, pmsg->len);
        fbled_row_upload(bit_data, pmsg->row_num, pmsg->buf, pmsg->len);
    }

    return 0;
}

int fbled_transfer(struct fbled_algo_bit_data *bit_data, struct fbled_msg *msgs)
{
    printk(KERN_INFO "0-%s-set\n", __func__);
    return fbled_xfer(bit_data, msgs);
}
EXPORT_SYMBOL(fbled_transfer);

void fbled_init(struct fbled_algo_bit_data *pdata)		//HT1632C?????
{
	fbled_io_init(pdata);
	
	fbled_cmd(pdata, SYS_DIS);
	fbled_cmd(pdata, COM_OPTION);
	fbled_cmd(pdata, RC_MASTER_MODE);
	fbled_cmd(pdata, SYS_EN);
	fbled_cmd(pdata, PWM_DUTY);
	fbled_cmd(pdata, BLINK_OFF);
	fbled_cmd(pdata, LED_ON);
}
EXPORT_SYMBOL(fbled_init);

void fbled_test(struct fbled_algo_bit_data *bit_data)
{
    int i;
    printk(KERN_INFO "%s-0:\n", __func__);
    for (i = 0; i < 32; i++) {
        cslo(bit_data);
        datlo(bit_data);
        wrlo(bit_data);
        cshi(bit_data);
        dathi(bit_data);
        wrhi(bit_data);
    }

    udelay(200);
    
    cslo(bit_data);
    fbled_outb(bit_data, MOD_CMD, 3);
    fbled_outb(bit_data, SYS_DIS, 9);
    fbled_outb(bit_data, COM_OPTION, 9);
    fbled_outb(bit_data, EXT_CLK_MASTER_MODE, 9);
    cshi(bit_data);

}
EXPORT_SYMBOL(fbled_test);

MODULE_DESCRIPTION("LED-Bus main module");
MODULE_LICENSE("GPL");