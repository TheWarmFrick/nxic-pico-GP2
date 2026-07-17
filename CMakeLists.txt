cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)

# Pico-PIO-USB (USB-A host port, D+ = GP12 / D- = GP13 on RP2350-USB-A).
# Priority: -DPICO_PIO_USB_PATH=... > lib/Pico-PIO-USB > FetchContent from GitHub.
if (NOT DEFINED PICO_PIO_USB_PATH)
    if (EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib/Pico-PIO-USB/src/pio_usb.c)
        set(PICO_PIO_USB_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/Pico-PIO-USB)
    else()
        include(FetchContent)
        FetchContent_Declare(
            pico_pio_usb_fetch
            GIT_REPOSITORY https://github.com/sekigon-gonnoc/Pico-PIO-USB
            GIT_TAG main
        )
        FetchContent_GetProperties(pico_pio_usb_fetch)
        if (NOT pico_pio_usb_fetch_POPULATED)
            FetchContent_Populate(pico_pio_usb_fetch)
        endif()
        set(PICO_PIO_USB_PATH ${pico_pio_usb_fetch_SOURCE_DIR})
    endif()
endif()
message("Pico-PIO-USB: ${PICO_PIO_USB_PATH}")

project(nxic_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(nxic_pico
    src/main.c
    src/usb_descriptors.c
    src/procon.c
    src/hid_input.c
    src/mapping.c
    src/settings.c
    src/config_mode.c
)

target_include_directories(nxic_pico PRIVATE src)

pico_generate_pio_header(nxic_pico ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio)

target_link_libraries(nxic_pico
    pico_stdlib
    pico_multicore
    pico_unique_id
    hardware_pio
    hardware_flash
    hardware_watchdog
    pico_lwip_nosys
    tinyusb_device
    tinyusb_host
    tinyusb_pico_pio_usb
)

pico_add_extra_outputs(nxic_pico)
