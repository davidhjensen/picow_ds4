add_executable(picow_ds4
	main.c
	bt_hid.c
)

pico_enable_stdio_usb(picow_ds4 1)
pico_enable_stdio_uart(picow_ds4 1)
pico_enable_stdio_semihosting(picow_ds4 0)

add_subdirectory("dht20-pico")

target_include_directories(picow_ds4 PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	dht20 PUBLIC "dht20-pico"
)

target_link_directories(picow_ds4 PUBLIC "dht20-pico")

target_link_libraries(picow_ds4
	hardware_pwm
	hardware_gpio
	pico_stdlib
	pico_cyw43_arch_none
        pico_btstack_classic
        pico_btstack_ble
        pico_btstack_cyw43
	pico_multicore
	hardware_i2c
	dht20
	pico_binary_info
)

pico_add_extra_outputs(picow_ds4)
