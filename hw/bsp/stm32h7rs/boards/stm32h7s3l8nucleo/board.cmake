set(MCU_VARIANT stm32h7s3xx)
set(JLINK_DEVICE stm32h7s3l8h)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/../../linker/${MCU_VARIANT}_flash.ld)

# Device port default to PORT1 Highspeed
#if (NOT DEFINED PORT)
set(PORT 1)
set(LOGGER swo)
set(DEBUG 1)
set(SPEED high)
#endif()

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32H7S3xx
    HSE_VALUE=24000000
    # default to PORT 1
    LOG=2
    BOARD_TUD_RHPORT=1
    BOARD_TUD_MAX_SPEED=OPT_MODE_HIGH_SPEED
    )
endfunction()
