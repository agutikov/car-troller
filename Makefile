
CROSS_PREFIX=arm-none-eabi-

CC=$(CROSS_PREFIX)gcc
OBJCOPY=$(CROSS_PREFIX)objcopy
OBJDUMP=$(CROSS_PREFIX)objdump
LD=$(CROSS_PREFIX)ld

CFLAGS=-std=gnu99 -ffreestanding -nostdlib
# -nostdinc
CLFAGS+=-ggdb -O0
# CFLAGS+=-O2
CFLAGS+= -mcpu=cortex-m3 -mlittle-endian -mthumb
# High-density devices are STM32F101xx and STM32F103xx microcontrollers where
#   the Flash memory density ranges between 256 and 512 Kbytes.
# STM32F10X_HD enables the maximum system clock (72MHz) in system_stm32f10x.c
CFLAGS+= -DSTM32F10X_HD
CFLAGS+= -Wall

SPL_LIB_PATH=../../spl/STM32F10x_StdPeriph_Lib_V3.5.0

SPL_SYSTEM_DIR=$(SPL_LIB_PATH)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x
SPL_CORE_DIR=$(SPL_LIB_PATH)/Libraries/CMSIS/CM3/CoreSupport
SPL_DRIVER_DIR=$(SPL_LIB_PATH)/Libraries/STM32F10x_StdPeriph_Driver

CFLAGS+= -I.
CFLAGS+= -I$(SPL_SYSTEM_DIR)
CFLAGS+= -I$(SPL_CORE_DIR)
CFLAGS+= -I$(SPL_DRIVER_DIR)/inc

LDSCRIPT=stm32f103vet6.ld

LDFLAGS= -nostdlib -M -Map firmware.map

SRCS=main.c startup.c
LOCAL_OBJS=$(SRCS:.c=.o)
SPL_SYSTEM_OBJS=system_stm32f10x.o
SPL_DRIVER_OBJS=stm32f10x_gpio.o stm32f10x_rcc.o stm32f10x_usart.o
ALL_OBJS=$(LOCAL_OBJS) $(SPL_SYSTEM_OBJS) $(SPL_DRIVER_OBJS)

.PHONY: all
all: firmware.bin firmware.lst

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@

# ld script should be first prerequisite because of -T $^
%.elf: $(LDSCRIPT) $(ALL_OBJS)
	$(LD) $(LDFLAGS) -T $^ -o $@

$(SPL_SYSTEM_OBJS): %.o: $(SPL_SYSTEM_DIR)/%.c
	$(CC) $(CFLAGS) -c -o $@ $^

$(SPL_DRIVER_OBJS): %.o: $(SPL_DRIVER_DIR)/src/%.c
	$(CC) $(CFLAGS) -c -o $@ $^

$(LOCAL_OBJS): %.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $^

%.lst:	%.elf
	$(OBJDUMP) -htS $< > $@

.PHONY: clean
clean:
	rm -rf *.o *.elf *.bin *.lst *.map


######### Flash and Debug

OPENOCD_CFG=board/stm32vldiscovery.cfg

FLASH_ADDR=0x8000000

.PHONY: flash
flash: firmware.bin
	st-flash write v1 $^ $(FLASH_ADDR)
#	openocd -f $(OPENOCD_CFG) -c "program firmware.bin verify reset"

.PHONY: gdb-server
gdb-server:
	st-util -1
#	openocd -f $(OPENOCD_CFG)


