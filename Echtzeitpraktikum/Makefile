################################################################################
# MODULE_NAME muss ungleich SOURCES sein!
MODULE_NAME = control_module

OBJS		= Echtzeitpraktikum.o
SOURCES		= Echtzeitpraktikum.c
################################################################################

KERNEL_DIR				:= /usr/src/linux/
INCLUDES 				:= -I/usr/src/linux/include -I/usr/src/linux/arch/x86/include -I/usr/realtime/include -I/usr/include -I/usr/include/x86_64-linux-gnu 
LIBS					:= 
LFLAGS					:= 
SYMBOLS 				:= /usr/realtime/Module.symvers /usr/share/modbus-com/Module.symvers 
EXTRA					:= -O2 -Wall

KBUILD_EXTRA_SYMBOLS	:= $(SYMBOLS)
EXTRA_CFLAGS			+= $(INCLUDES) $(EXTRA) $(LIBS)
obj-m					+= $(MODULE_NAME).o
$(MODULE_NAME)-objs		:= $(OBJS)

all:
	$(MAKE) KBUILD_VERBOSE=3 -C $(KERNEL_DIR) SUBDIRS=$(PWD) modules

clean:
	rm -rf .tmp_versions *.symvers *.o *.ko *.mod.c .*.cmd .*flags *.order
