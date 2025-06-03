add_executable(pico_emb
        main.c
        lwipopts.h
)

# Salva o .uf2 no diretório de build
set_target_properties(pico_emb PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

target_link_libraries(pico_emb
        pico_stdlib
        pico_cyw43_arch_lwip_threadsafe_background  # Wi-Fi + LWIP
        hardware_adc 
        hardware_pwm 
        hardware_clocks
        freertos
        hardware_i2c                               # I2C para o LCD
        pico_lwip_mqtt                             # MQTT
)

# Adiciona o diretório atual como include path
target_include_directories(pico_emb PRIVATE ${CMAKE_CURRENT_LIST_DIR})

pico_add_extra_outputs(pico_emb)
pico_enable_stdio_usb(pico_emb 1)
pico_enable_stdio_uart(pico_emb 1)
