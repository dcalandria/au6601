TARGET_MODULE := au6601-pci

obj-m += $(TARGET_MODULE).o

$(TARGET_MODULE)-objs := au6601.o


KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

