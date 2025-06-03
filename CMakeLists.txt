# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")
set(PICO_CYW43_ARCH_THREADSAFE_BACKGROUND ON)         # <-- NOVO
set(PICO_CYW43_ARCH_POLL ON)                           # <-- NOVO
set(PICO_ENABLE_MBEDTLS ON)                            # <-- NOVO

cmake_minimum_required(VERSION 3.12)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pico_emb)

# initialize the Pico SDK
pico_sdk_init()

# rest of your project

add_subdirectory(freertos)
add_subdirectory(main)

