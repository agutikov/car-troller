car-troller
===========

Firmware for STM32F4-DISCO board:

http://www.st.com/web/catalog/tools/FM116/SC959/SS1532/PF259090

with STM32F429ZIT6 mcu:

http://www.st.com/web/en/catalog/mmc/FM141/SC1169/SS1577/LN1806


TODO
----

1. Store calibration, call calibration from menu

2. alfa blending

3. time, rtc?, timer delay


BUILD, FLASH and DEBUG
----------------------

```bash

# download STM32Cube
make get_cube

# compile
make all

# flash
make flash

# for debug - check Makefile last section "Flash and Debug"

# download datasheet and other pdfs from ST site
make get_docs

# clean revious build
make clean

# download and convert images
make get_data

```

Used examples
-------------

./STM32Cube_FW_F4_V1.4.0/Projects/STM32F429I-Discovery/Examples/BSP
./STM32Cube_FW_F4_V1.4.0/Projects/STM32F429I-Discovery/Examples/LTDC/LTDC_Display_2Layers
./STM32Cube_FW_F4_V1.4.0/Projects/STM32F429I-Discovery/Applications/Display/LTDC_Paint











