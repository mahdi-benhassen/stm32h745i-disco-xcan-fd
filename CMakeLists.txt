cmake_minimum_required(VERSION 3.13)

set(RV_ARCH "rv32imac_zicsr_zifencei")
set(RV_ABI "ilp32")
set(HPM_SDK_LD_NO_NANO_SPECS 1)

set(CONFIG_CHERRYUSB 1)
set(CONFIG_USB_DEVICE 1)
set(CONFIG_CHERRYUSB_DEVICE 1)

set(APP_NAME "HPM5300EVK_PCAN_FD")

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})
project(hpm5300evk_pcan_fd)

sdk_ld_options("--specs=nano.specs")

sdk_compile_definitions(
  -DPCAN_PRO_FD=1
  -DINCLUDE_LIN_INTERFACE=1
)

# Expose app/ so CherryUSB finds usb_config.h
sdk_inc(${CMAKE_CURRENT_SOURCE_DIR}/app)
sdk_app_inc(${CMAKE_CURRENT_SOURCE_DIR}/app/pcan)

set(PCAN_APP_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/app/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/app/pcan/pcan_usb_device.c
  ${CMAKE_CURRENT_SOURCE_DIR}/app/pcan/pcanpro_can.c
  ${CMAKE_CURRENT_SOURCE_DIR}/app/pcan/pcanpro_fd_protocol.c
  ${CMAKE_CURRENT_SOURCE_DIR}/app/pcan/pcanpro_led.c
  ${CMAKE_CURRENT_SOURCE_DIR}/app/pcan/pcanpro_timestamp.c
)

set_source_files_properties(${PCAN_APP_SRCS} PROPERTIES
  COMPILE_OPTIONS "-Wall;-Wextra;-Werror"
)

sdk_app_src(${PCAN_APP_SRCS})

add_custom_command(TARGET ${APP_ELF_NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex -S
          ${EXECUTABLE_OUTPUT_PATH}/${APP_ELF_NAME}
          ${EXECUTABLE_OUTPUT_PATH}/${APP_NAME}.hex
  COMMENT "Generating ${APP_NAME}.hex"
)

generate_ide_projects()
