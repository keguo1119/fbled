KVERS = $(shell uname -r)

#Kernel modules
obj-m += fbled-gpio.o
obj-m += fbled-gpio-custom.o
obj-m += fbled-algo-bit.o

build: kernel_modules

kernel_modules:
	make -C /lib/modules/$(KVERS)/build M=$(CURDIR) modules

#user_test:
#	gcc test.c -o test

clean:
	rm *.ko *.mod.c  *.o
