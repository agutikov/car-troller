
# TODO: copy sources into build directory, change include paths, build inside separate directory

### ENVIRONMENT

CROSS_PREFIX=arm-none-eabi-

CC=$(CROSS_PREFIX)gcc
OBJCOPY=$(CROSS_PREFIX)objcopy
OBJDUMP=$(CROSS_PREFIX)objdump
LD=$(CROSS_PREFIX)ld


### FLAGS

ARCH_FLAGS= -mcpu=cortex-m4 -mlittle-endian -mthumb

CFLAGS=-std=gnu99 -ffreestanding
# -nostdlib -nostdinc
CFLAGS+=-ggdb
CFLAGS+=-O0
CFLAGS+= $(ARCH_FLAGS)
CFLAGS+= -Wall

CFLAGS+= -DSTM32F429XX -DSTM32F429xx

### CUBE SOURCES

CUBE_URL="http://www.st.com/st-web-ui/static/active/en/st_prod_software_internet/resource/technical/software/firmware/stm32cubef4.zip"
CUBE_VERSION=1.4.0
CUBE_ZIP_FILENAME=stm32cubef4.zip
CUBE_DIR_NAME=STM32Cube_FW_F4
CUBE_DIR=./$(CUBE_DIR_NAME)_V$(CUBE_VERSION)

CMSIS_INC_DIR=$(CUBE_DIR)/Drivers/CMSIS/Include
CMSIS_F4XX_INC_DIR=$(CUBE_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Include

HAL_INC_DIR=$(CUBE_DIR)/Drivers/STM32F4xx_HAL_Driver/Inc
HAL_SRC_DIR=$(CUBE_DIR)/Drivers/STM32F4xx_HAL_Driver/Src

DISCO_BSP_DIR=$(CUBE_DIR)/Drivers/BSP/STM32F429I-Discovery

COMPONENTS_DIR=$(CUBE_DIR)/Drivers/BSP/Components

### INCLUDES

CFLAGS+= -I.
CFLAGS+= -I$(CMSIS_INC_DIR)
CFLAGS+= -I$(CMSIS_F4XX_INC_DIR)
CFLAGS+= -I$(HAL_INC_DIR)
CFLAGS+= -I$(DISCO_BSP_DIR)
CFLAGS+= -I$(COMPONENTS_DIR)/Common/
CFLAGS+= -I$(COMPONENTS_DIR)/ili9341/
CFLAGS+= -I$(COMPONENTS_DIR)/stmpe811/
CFLAGS+= -I$(COMPONENTS_DIR)/l3gd20/

### LINKAGE

LDSCRIPT=stm32f429i-discovery.ld

GCCLDFLAGS= $(ARCH_FLAGS) -nostartfiles -Wl,-M -Wl,-Map,firmware.map

### SOURCES

SRCS=main.c system_stm32f4xx.c
LOCAL_OBJS=$(SRCS:.c=.o)

ALL_C_OBJS=$(LOCAL_OBJS)
ALL_S_OBJS=startup_stm32f429xx.o


### TARGETS

.PHONY: all
all: firmware.bin firmware.lst firmware.elf firmware.hex

%.hex: %.elf
	$(OBJCOPY) $^ -O ihex $@

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@

# ld script should be first prerequisite because of -T $^
%.elf: $(LDSCRIPT) $(ALL_C_OBJS) $(ALL_S_OBJS)
	$(CC) $(GCCLDFLAGS) -Wl,-T,$^ -o $@

$(ALL_C_OBJS): %.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $^

$(ALL_S_OBJS): %.o: %.s
	$(CC) $(CFLAGS) -c -o $@ $^

%.lst:	%.elf
	$(OBJDUMP) -htS $< > $@


.PHONY: get_cube
get_cube: $(CUBE_DIR)

$(CUBE_DIR): $(CUBE_ZIP_FILENAME)
	unzip $^

$(CUBE_ZIP_FILENAME):
	wget $(CUBE_URL)


.PHONY: clean
clean:
	rm -rvf *.o *.elf *.bin *.lst *.map *.hex
	find -name "*.o" -exec rm -v {} \;

######### Flash and Debug

OPENOCD_CFG=board/stm32f429discovery.cfg

FLASH_ADDR=0x08000000

.PHONY: flash
flash: firmware.hex
	openocd -f $(OPENOCD_CFG) -c "program firmware.hex verify reset"

.PHONY: gdb-server
gdb-server:
	openocd -f $(OPENOCD_CFG)

.PHONY: gdb
gdb: firmware.elf
	$(CROSS_PREFIX)gdb --eval-command="target remote localhost:3333" firmware.elf


