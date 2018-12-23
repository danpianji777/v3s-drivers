KERN_DIR = /work/v3s/linux

all:
	make -C $(KERN_DIR) M=`pwd` modules CC=arm-linux-gnueabihf-gcc LD=arm-linux-gnueabi-ld ARCH=arm 
	@cp gpio_init_example.ko /work/
clean:
	make -C $(KERN_DIR) M=`pwd` modules CC=arm-linux-gnueabihf-gcc LD=arm-linux-gnueabi-ld ARCH=arm clean
	rm -rf modules.order

obj-m	+= gpio_init_example.o
