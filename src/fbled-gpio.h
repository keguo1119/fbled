#ifndef _FBLED_GPIO_H
#define _FBLED_GPIO_H
#include <linux/types.h>
struct fbled_gpio_platform_data {
	unsigned int dat_pin;
	unsigned int wr_pin;
	unsigned int rd_pin;
	unsigned int cs_pin;
	int udelay;
	int timeout;
};

struct fbled_msg {
	__u16 addr;	/* slave address			*/
	__u16 flags;
#define FBLED_M_WR_PAGE		0x0001	/* read data, from slave to master */
#define FBLED_M_WR_ROW		0x0010
					/* FBLED_M_RD is guaranteed to be 0x0001! */
//#define FBLED_M_TEN		0x0010	/* this is a ten bit chip address */
//#define FBLED_M_RECV_LEN		0x0400	/* length will be first received byte */
//#define FBLED_M_NO_RD_ACK		0x0800	/* if FBLED_FUNC_PROTOCOL_MANGLING */
//#define FBLED_M_IGNORE_NAK	0x1000	/* if FBLED_FUNC_PROTOCOL_MANGLING */
//#define FBLED_M_REV_DIR_ADDR	0x2000	/* if FBLED_FUNC_PROTOCOL_MANGLING */
//#define FBLED_M_NOSTART		0x4000	/* if FBLED_FUNC_NOSTART */
//#define FBLED_M_STOP		0x8000	/* if FBLED_FUNC_PROTOCOL_MANGLING */
	__u16 len;		/* msg length				*/
	__u8 *buf;		/* pointer to msg data			*/
	__u8 row_num;
};

#define FBLED_RDWR 0x0001

#define DBG( format, arg... ) \
   if (1 == 1) \
   { \
      printk( KERN_INFO "FBLED::%s " format, __FUNCTION__, ## arg ); \
   } \


#endif
