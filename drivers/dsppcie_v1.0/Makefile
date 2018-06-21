KDIR=/usr/src/linux-headers-$(shell uname -r)

PWD=$(shell pwd)

obj-m = dsppcie.o

all:
	$(MAKE) -C $(KDIR) M=$(PWD) 
clean:
	rm -fr *.o *.mod.c *.ko