obj-m := cam-ov5640-control.o cam-dvp-bridge.o

SRC := $(shell pwd)

all modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean
