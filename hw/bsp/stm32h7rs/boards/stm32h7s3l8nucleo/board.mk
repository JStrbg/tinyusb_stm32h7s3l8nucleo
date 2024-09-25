CFLAGS += -DSTM32H7S3xx -DHSE_VALUE=24000000

# Default is HighSpeed port
PORT ?= 1
LOG  = 2
DEBUG = 1
SPEED = high	# high or full
LOGGER = swo

# GCC
SRC_S_GCC += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32h7s3xx.s
LD_FILE_GCC =  $(FAMILY_PATH)/linker/stm32h7s3xx_flash.ld

# IAR
SRC_S_IAR += $(ST_CMSIS)/Source/Templates/iar/startup_stm32h7s3xx.s
LD_FILE_IAR = $(ST_CMSIS)/Source/Templates/iar/linker/stm32h7s3xx_flash.icf

# For flash-jlink target
JLINK_DEVICE = stm32h7s3l8

# flash target using on-board stlink
flash: flash-stlink
