obj-m += rplidar.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

install:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	mkdir -p /lib/modules/$(shell uname -r)/kernel/drivers/rplidar
	cp rplidar.ko /lib/modules/$(shell uname -r)/kernel/drivers/rplidar
	echo "kernel/drivers/rplidar/rplidar.ko:" >> /lib/modules/$(shell uname -r)/modules.dep
	echo "kernel/drivers/rplidar/rplidar.ko" >> /lib/modules/$(shell uname -r)/modules.order
	insmod rplidar.ko
	make clean

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
