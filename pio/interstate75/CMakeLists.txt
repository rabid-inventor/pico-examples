add_executable(interstate75)

pico_generate_pio_header(interstate75 ${CMAKE_CURRENT_LIST_DIR}/hub75.pio)

target_sources(interstate75 PRIVATE hub75.c)

pico_enable_stdio_usb(interstate75 1)
pico_enable_stdio_uart(interstate75 0)


target_link_libraries(interstate75 PRIVATE pico_multicore pico_stdlib hardware_pio hardware_adc)
pico_add_extra_outputs(interstate75)

# add url via pico_set_program_url
example_auto_set_url(interstate75)
