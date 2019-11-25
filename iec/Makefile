ifneq ($(KERNELRELEASE),)
obj-m	:= iec.o

else
KDIR	:= /usr/src/linux
PWD		:= $(shell pwd)

default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

install:
	ln -s `pwd`/iec.ko /lib/modules/`uname -r`/iec.ko
endif

