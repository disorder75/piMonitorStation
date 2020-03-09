ifndef KERNEL_SRC
	$(error KERNEL_SRC must be set in the command line)
endif

PWD := $(shell pwd)
ARCH ?= arm
CROSS_COMPILE ?= ${CCPREFIX_PI1_Zero_W_Module}
 
obj-m = dht11unimi_drv.o
dht11unimi_drv-objs = dht11unimi.o gpio_lib.o
obj-m += dht11unimi-req.o

all:
	make -C $(KERNEL_SRC) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -Wall $(CFLAGS) SUBDIRS=$(PWD) modules

modules clean:
	rm -fr dht11unimi_chrdev $(obj-m) *.ko *.mod.o *.mod.c ${m-objs} ${dht11unimi_drv-objs}
