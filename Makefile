CROSS_COMPILE ?= arm-linux-gnu-
ARCH ?= arm
KSRC := ../linux

obj-m += eKTF2127.o

all:
	make -C $(KSRC) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules


clean:
	make -C $(KSRC) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) clean
