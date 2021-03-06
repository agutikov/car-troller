
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


### DOCUMENTATION

DATASHEET_URL=http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00071990.pdf
DATASHEET_FILENAME=datasheet.pdf

USER_MANUAL_URL=http://www.st.com/st-web-ui/static/active/en/resource/technical/document/user_manual/DM00091013.pdf
USER_MANUAL_FILENAME=user_manual.pdf

REFERENCE_MANUAL_URL=http://www.st.com/st-web-ui/static/active/en/resource/technical/document/reference_manual/DM00031020.pdf
REFERENCE_MANUAL_FILENAME=reference_manual.pdf

PROGRAMMING_MANUAL_URL=http://www.st.com/st-web-ui/static/active/en/resource/technical/document/programming_manual/DM00046982.pdf
PROGRAMMING_MANUAL_FILENAME=programming_manual.pdf

DOCS_FILENAMES=$(DATASHEET_FILENAME) $(USER_MANUAL_FILENAME) \
$(REFERENCE_MANUAL_FILENAME) $(PROGRAMMING_MANUAL_FILENAME)
DOCS=$(DOCS_FILENAMES:%=Documentation/%)


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

### DATA FILES

BMPS=menu_0.bmp selected_120x80.bmp
IMAGES=$(BMPS:%=%.c)

MENU_O_BMP_FILEID=0B4GiJQNyi2tUOXZpcXJfUU9JdW8
SELECTED_120x80_BMP_FILEID=0B4GiJQNyi2tUOTZhc2hjM3BhSmM

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

SRCS=main.c system_stm32f4xx.c ts_calibration.c stm32f4xx_it.c
LOCAL_OBJS=$(SRCS:.c=.o)

HAL_SRC= stm32f4xx_hal.c \
	stm32f4xx_hal_adc.c \
	stm32f4xx_hal_adc_ex.c \
	stm32f4xx_hal_can.c \
	stm32f4xx_hal_cortex.c  \
	stm32f4xx_hal_crc.c  \
	stm32f4xx_hal_cryp.c  \
	stm32f4xx_hal_cryp_ex.c  \
	stm32f4xx_hal_dac.c  \
	stm32f4xx_hal_dac_ex.c  \
	stm32f4xx_hal_dcmi.c  \
	stm32f4xx_hal_dma.c  \
	stm32f4xx_hal_dma2d.c  \
	stm32f4xx_hal_dma_ex.c  \
	stm32f4xx_hal_eth.c  \
	stm32f4xx_hal_flash.c  \
	stm32f4xx_hal_flash_ex.c  \
	stm32f4xx_hal_flash_ramfunc.c  \
	stm32f4xx_hal_gpio.c  \
	stm32f4xx_hal_hash.c  \
	stm32f4xx_hal_hash_ex.c  \
	stm32f4xx_hal_hcd.c  \
	stm32f4xx_hal_i2c.c  \
	stm32f4xx_hal_i2c_ex.c  \
	stm32f4xx_hal_i2s.c  \
	stm32f4xx_hal_i2s_ex.c  \
	stm32f4xx_hal_irda.c  \
	stm32f4xx_hal_iwdg.c  \
	stm32f4xx_hal_ltdc.c  \
	stm32f4xx_hal_nand.c  \
	stm32f4xx_hal_nor.c  \
	stm32f4xx_hal_pccard.c  \
	stm32f4xx_hal_pcd.c  \
	stm32f4xx_hal_pcd_ex.c  \
	stm32f4xx_hal_pwr.c  \
	stm32f4xx_hal_pwr_ex.c  \
	stm32f4xx_hal_rcc.c  \
	stm32f4xx_hal_rcc_ex.c  \
	stm32f4xx_hal_rng.c  \
	stm32f4xx_hal_rtc.c  \
	stm32f4xx_hal_rtc_ex.c  \
	stm32f4xx_hal_sai.c  \
	stm32f4xx_hal_sd.c  \
	stm32f4xx_hal_sdram.c  \
	stm32f4xx_hal_smartcard.c  \
	stm32f4xx_hal_spi.c  \
	stm32f4xx_hal_sram.c  \
	stm32f4xx_hal_tim.c  \
	stm32f4xx_hal_tim_ex.c  \
	stm32f4xx_hal_uart.c  \
	stm32f4xx_hal_usart.c  \
	stm32f4xx_hal_wwdg.c  \
	stm32f4xx_ll_fmc.c  \
	stm32f4xx_ll_sdmmc.c  \
	stm32f4xx_ll_usb.c
HAL_SRCS=$(HAL_SRC:%=$(HAL_SRC_DIR)/%)
HAL_OBJS=$(HAL_SRCS:.c=.o)

DISCO_BSP_SRC=stm32f429i_discovery.c \
	stm32f429i_discovery_eeprom.c \
	stm32f429i_discovery_gyroscope.c \
	stm32f429i_discovery_io.c \
	stm32f429i_discovery_lcd.c \
	stm32f429i_discovery_sdram.c \
	stm32f429i_discovery_ts.c
DISCO_BSP_SRCS=$(DISCO_BSP_SRC:%=$(DISCO_BSP_DIR)/%)
DISCO_BSP_OBJS=$(DISCO_BSP_SRCS:.c=.o)

COMPONENTS_SRCS=$(COMPONENTS_DIR)/ili9341/ili9341.c \
	$(COMPONENTS_DIR)/stmpe811/stmpe811.c \
	$(COMPONENTS_DIR)/l3gd20/l3gd20.c
COMPONENTS_OBJS=$(COMPONENTS_SRCS:.c=.o)


ALL_C_OBJS=$(LOCAL_OBJS) $(CMSIS_SYSTEM_OBJS) $(HAL_OBJS) \
$(DISCO_BSP_OBJS) $(COMPONENTS_OBJS)
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

main.c: images.c


.PHONY: clean
clean:
	rm -rvf *.o *.elf *.bin *.lst *.map *.hex images.c *.bmp *.bmp.c
	find -name "*.o" -exec rm -v {} \;
#	rm -rf $(CUBE_DIR)


.PHONY: get_cube
get_cube: $(CUBE_DIR)

$(CUBE_DIR): $(CUBE_ZIP_FILENAME)
	unzip $^

$(CUBE_ZIP_FILENAME):
	wget $(CUBE_URL)


.PHONY: get_docs
get_docs: $(DOCS)

Documentation/$(DATASHEET_FILENAME): Documentation
	wget -v $(DATASHEET_URL) -O $@

Documentation/$(USER_MANUAL_FILENAME): Documentation
	wget -v $(USER_MANUAL_URL) -O $@

Documentation/$(REFERENCE_MANUAL_FILENAME): Documentation
	wget -v $(REFERENCE_MANUAL_URL) -O $@

Documentation/$(PROGRAMMING_MANUAL_FILENAME): Documentation
	wget -v $(PROGRAMMING_MANUAL_URL) -O $@

Documentation:
	mkdir Documentation


.PHONY: get_data
get_data:  images.c

images.c: $(IMAGES)

$(IMAGES): %.bmp.c : %.bmp
	echo "const" > $@
	xxd -i $< >> $@
	echo "#include \"$@\";" >> images.c

menu_0.bmp:
	wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=$(MENU_O_BMP_FILEID)' -O $@

selected_120x80.bmp:
	wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=$(SELECTED_120x80_BMP_FILEID)' -O $@


######### Flash and Debug

OPENOCD_CFG=board/stm32f429discovery.cfg

FLASH_ADDR=0x8000000

.PHONY: flash
flash: firmware.hex
	openocd -f $(OPENOCD_CFG) -c "program firmware.hex verify reset"

.PHONY: gdb-server
gdb-server:
	openocd -f $(OPENOCD_CFG)

.PHONY: gdb
gdb: firmware.elf
	$(CROSS_PREFIX)gdb --eval-command="target remote localhost:3333" firmware.elf


