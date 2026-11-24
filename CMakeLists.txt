cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

project(AT_START_F457_PCAN_FD C ASM)

set(COMMON_MCU_FLAGS
  -mcpu=cortex-m4
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
  -mthumb
  -ffunction-sections
  -fdata-sections
)

set(COMMON_DEFINES
  AT32F457ZET7
  USE_STDPERIPH_DRIVER
  AT_START_F457_V1
  SUPPORT_CAN_FD
  PCAN_PRO_FD=1
  INCLUDE_LIN_INTERFACE=1
)

set(COMMON_INCLUDES
  ${CMAKE_SOURCE_DIR}/app
  ${CMAKE_SOURCE_DIR}/app/pcan
  ${CMAKE_SOURCE_DIR}/board
  ${CMAKE_SOURCE_DIR}/libraries/cmsis/cm4/core_support
  ${CMAKE_SOURCE_DIR}/libraries/cmsis/cm4/device_support
  ${CMAKE_SOURCE_DIR}/libraries/drivers/inc
  ${CMAKE_SOURCE_DIR}/middlewares/usb_drivers/inc
)

add_library(at32_sdk OBJECT
  libraries/cmsis/cm4/device_support/system_at32f45x.c
  libraries/cmsis/cm4/device_support/startup/gcc/startup_at32f45x.s
  board/at32f45x_board.c
  libraries/drivers/src/at32f45x_can.c
  libraries/drivers/src/at32f45x_crm.c
  libraries/drivers/src/at32f45x_exint.c
  libraries/drivers/src/at32f45x_flash.c
  libraries/drivers/src/at32f45x_gpio.c
  libraries/drivers/src/at32f45x_misc.c
  libraries/drivers/src/at32f45x_pwc.c
  libraries/drivers/src/at32f45x_scfg.c
  libraries/drivers/src/at32f45x_tmr.c
  libraries/drivers/src/at32f45x_usart.c
  libraries/drivers/src/at32f45x_usb.c
  middlewares/usb_drivers/src/usb_core.c
  middlewares/usb_drivers/src/usbd_core.c
  middlewares/usb_drivers/src/usbd_int.c
  middlewares/usb_drivers/src/usbd_sdr.c
)

target_compile_definitions(at32_sdk PRIVATE ${COMMON_DEFINES})
target_include_directories(at32_sdk PRIVATE ${COMMON_INCLUDES})
target_compile_options(at32_sdk PRIVATE
  $<$<COMPILE_LANGUAGE:C>:${COMMON_MCU_FLAGS} -Wall -Wno-unused-parameter -Wno-strict-prototypes>
  $<$<COMPILE_LANGUAGE:ASM>:${COMMON_MCU_FLAGS} -x assembler-with-cpp>
  $<$<CONFIG:Debug>:-O0 -g3>
  $<$<CONFIG:Release>:-O2 -g0>
)

add_executable(${PROJECT_NAME}
  app/main.c
  app/at32f45x_clock.c
  app/at32f45x_int.c
  app/pcan/pcan_usb_device.c
  app/pcan/pcan_usbd_desc.c
  app/pcan/pcanpro_can.c
  app/pcan/pcanpro_fd_protocol.c
  app/pcan/pcanpro_led.c
  app/pcan/pcanpro_timestamp.c
  app/pcan/pcanpro_usbd.c
  $<TARGET_OBJECTS:at32_sdk>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE ${COMMON_DEFINES})
target_include_directories(${PROJECT_NAME} PRIVATE ${COMMON_INCLUDES})
target_compile_options(${PROJECT_NAME} PRIVATE
  ${COMMON_MCU_FLAGS}
  -Wall
  -Wextra
  -Werror
  $<$<CONFIG:Debug>:-O0 -g3>
  $<$<CONFIG:Release>:-O2 -g0>
)

set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/libraries/cmsis/cm4/device_support/startup/gcc/linker/AT32F457xE_FLASH.ld")
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_DEPENDS "${LINKER_SCRIPT}")

target_link_options(${PROJECT_NAME} PRIVATE
  ${COMMON_MCU_FLAGS}
  -T${LINKER_SCRIPT}
  --specs=nano.specs
  --specs=nosys.specs
  -Wl,--gc-sections
  -Wl,--print-memory-usage
  -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
  COMMENT "Generating Intel HEX, raw binary, and size summary"
)
