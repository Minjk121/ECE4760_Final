add_executable(final)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(final PRIVATE pico_stdlib pico_multicore pico_bootsel_via_double_reset hardware_sync hardware_spi hardware_pio hardware_dma hardware_adc hardware_irq)

# must match with pio filename and executable name from above
pico_generate_pio_header(final ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(final ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(final ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

# must match with executable name and source file names
target_sources(final PRIVATE final.c vga_graphics.c)

# create map/bin/hex file etc.
pico_add_extra_outputs(final)