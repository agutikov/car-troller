

CROSS_PREFIX=arm-none-eabi-

CC=$(CROSS_PREFIX)gcc
OBJCOPY=$(CROSS_PREFIX)objcopy
OBJDUMP=$(CROSS_PREFIX)objdump
LD=$(CROSS_PREFIX)ld

ARCH_FLAGS= -mcpu=cortex-m4 -mlittle-endian -mthumb

CFLAGS=-std=gnu99 -ffreestanding
# -nostdlib -nostdinc
CFLAGS+=-ggdb
CFLAGS+=-O0
CFLAGS+= $(ARCH_FLAGS)
CFLAGS+= -Wall

CFLAGS+= -DSTM32F429XX

DATASHEET_URL=http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00071990.pdf
DATASHEET_FILENAME=datasheet.pdf

USER_MANUAL_URL=http://www.st.com/st-web-ui/static/active/en/resource/technical/document/user_manual/DM00091013.pdf
USER_MANUAL_FILENAME=user_manual.pdf

REFERENCE_MANUAL_URL=http://www.st.com/st-web-ui/static/active/en/resource/technical/document/reference_manual/DM00031020.pdf
REFERENCE_MANUAL_FILENAME=reference_manual.pdf

PROGRAMMING_MANUAL_URL=http://www.st.com/st-web-ui/static/active/en/resource/technical/document/programming_manual/DM00046982.pdf
PROGRAMMING_MANUAL_FILENAME=programming_manual.pdf

DOCS_FILENAMES=$(DATASHEET_FILENAME) $(USER_MANUAL_FILENAME) $(REFERENCE_MANUAL_FILENAME) $(PROGRAMMING_MANUAL_FILENAME)
DOCS=$(DOCS_FILENAMES:%=Documentation/%)

CUBE_URL="http://www.st.com/st-web-ui/static/active/en/st_prod_software_internet/resource/technical/software/firmware/stm32cubef4.zip"
CUBE_VERSION=1.4.0
CUBE_ZIP_FILENAME=stm32cubef4.zip
CUBE_DIR_NAME=STM32Cube_FW_F4
CUBE_DIR=$(CUBE_DIR_NAME)_V$(CUBE_VERSION)




CFLAGS+= -I.



LDSCRIPT=stm32f429zit6.ld

GCCLDFLAGS= $(ARCH_FLAGS) -nostartfiles -Wl,-M -Wl,-Map,firmware.map




SRCS=main.c startup.c
LOCAL_OBJS=$(SRCS:.c=.o)

ALL_OBJS=$(LOCAL_OBJS)


.PHONY: all
all: firmware.bin firmware.lst firmware.elf firmware.hex

%.hex: %.elf
	$(OBJCOPY) $^ -O ihex $@

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@

# ld script should be first prerequisite because of -T $^
%.elf: $(LDSCRIPT) $(ALL_OBJS)
	$(CC) $(GCCLDFLAGS) -Wl,-T,$^ -o $@


$(LOCAL_OBJS): %.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $^


%.lst:	%.elf
	$(OBJDUMP) -htS $< > $@

.PHONY: clean
clean:
	rm -rf *.o *.elf *.bin *.lst *.map *.hex

.PHONY: get_cube
get_cube: $(CUBE_DIR)

$(CUBE_DIR): $(CUBE_ZIP_FILENAME)
	unzip $^

$(CUBE_ZIP_FILENAME):
	wget $(CUBE_URL)


.PHONY: download_docs
download_docs: $(DOCS)

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


