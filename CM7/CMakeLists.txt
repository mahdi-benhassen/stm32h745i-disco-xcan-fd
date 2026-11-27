cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Compiler options
set(STM32_MCU_FLAGS  "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard " )

# Linker options
set(STM32_LINKER_SCRIPT stm32h745xx_flash_CM7.ld)
set(STM32_LINKER_OPTION  )

# Set the project name
set(CMAKE_PROJECT_NAME PCAN_STM32H745I_CM7)
project(${CMAKE_PROJECT_NAME})

# ---------------------------------------------------------------------------
# Topology & can-node ISO-TP / UDS configuration options
# ---------------------------------------------------------------------------
option(CAN_NODE_SINGLE_BOARD
    "Option B: keep PCAN-USB Pro FD on FDCAN1 and run can-node UDS/ISO-TP ECU on FDCAN2" OFF)
set(CAN_NODE_UDS_BLOCK_DATA_MAX "4096"
    CACHE STRING "UDS TransferData max block payload in bytes (4096 -> UDS_REQUEST_MAX=4098 triggers 32-bit FF_DL; or 1024)")
set(CAN_NODE_ISOTP_RX_BUF_SIZE "8192"
    CACHE STRING "ISO-TP static RX reassembly buffer size in bytes")
set(CAN_NODE_CAN_FD "1"
    CACHE STRING "1: CAN-FD + BRS enabled, 0: Classical CAN")
set(CAN_NODE_CAN_FD_BRS "1"
    CACHE STRING "1: Enable CAN-FD Bit Rate Switch, 0: BRS disabled")
set(CAN_NODE_TX_DL "64"
    CACHE STRING "ISO 15765-2 TX_DL: 8 (classical) or {8,12,16,20,24,32,48,64}")

if(CAN_NODE_SINGLE_BOARD)
    set(CAN_NODE_SINGLE_BOARD_DEF 1)
    message(STATUS "Topology: Option B (Single-Board PCAN on FDCAN1 + can-node UDS ECU on FDCAN2)")
else()
    set(CAN_NODE_SINGLE_BOARD_DEF 0)
    message(STATUS "Topology: Option A (Standalone can-node UDS ECU on FDCAN1)")
endif()

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Include mx-generated
include("mx-generated.cmake")

# In order to use CMake for cross-compiling
set(CMAKE_CXX_COMPILER_FORCED "true")

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    CAN_NODE_SINGLE_BOARD=${CAN_NODE_SINGLE_BOARD_DEF}
    UDS_BLOCK_DATA_MAX=${CAN_NODE_UDS_BLOCK_DATA_MAX}U
    CAN_NODE_ISOTP_RX_BUF_SIZE=${CAN_NODE_ISOTP_RX_BUF_SIZE}U
    CAN_NODE_CAN_FD=${CAN_NODE_CAN_FD}U
    CAN_NODE_CAN_FD_BRS=${CAN_NODE_CAN_FD_BRS}U
    CAN_NODE_TX_DL=${CAN_NODE_TX_DL}U
)

if(CAN_NODE_SINGLE_BOARD)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        PCAN_PRO_FD=1
        INCLUDE_LIN_INTERFACE=1
        USB_MODULE_ID=DEVICE_FS
    )
endif()

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/App/can_node
    ${CMAKE_CURRENT_SOURCE_DIR}/App/can_node/third_party
)

if(CAN_NODE_SINGLE_BOARD)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/App/pcan
    )
endif()

set(CAN_NODE_THIRD_PARTY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/App/can_node/third_party/can/can_frame.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/can_node/third_party/isotp/isotp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/can_node/third_party/dtc/dtc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/can_node/third_party/crc/crc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/can_node/third_party/uds/uds.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/can_node/third_party/uds/uds_program.c
)

set(CAN_NODE_PORT_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/App/can_node/can_node_time.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/can_node/can_node_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/can_node/can_node_fdcan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/can_node/can_node_diag.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/can_node/can_node_backend.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/can_node/can_node_app.c
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CAN_NODE_THIRD_PARTY_SOURCES}
    ${CAN_NODE_PORT_SOURCES}
)

if(CAN_NODE_SINGLE_BOARD)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/App/pcan/pcan_usb_device.c
        ${CMAKE_CURRENT_SOURCE_DIR}/App/pcan/pcan_usbd_desc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/App/pcan/pcanpro_usbd.c
        ${CMAKE_CURRENT_SOURCE_DIR}/App/pcan/pcanpro_can.c
        ${CMAKE_CURRENT_SOURCE_DIR}/App/pcan/pcanpro_fd_protocol.c
        ${CMAKE_CURRENT_SOURCE_DIR}/App/pcan/pcanpro_timestamp.c
        ${CMAKE_CURRENT_SOURCE_DIR}/App/pcan/pcanpro_led.c
    )
endif()

# Enforce strict MISRA-grade warning flags: third_party pure C11 gets full -Wconversion;
# HAL-facing port sources omit -Wconversion because STM32H7 CMSIS cachel1_armv7.h triggers it.
set_source_files_properties(${CAN_NODE_THIRD_PARTY_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-Wall;-Wextra;-Wconversion;-Wshadow;-Wstrict-prototypes;-Wdouble-promotion;-Werror"
)
set_source_files_properties(${CAN_NODE_PORT_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-Wall;-Wextra;-Wshadow;-Wstrict-prototypes;-Wdouble-promotion;-Werror"
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined libraries
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()

# Generate .hex, .bin, and print firmware size after linking
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex and ${CMAKE_PROJECT_NAME}.bin"
)
