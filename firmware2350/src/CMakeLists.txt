add_executable(${NAME}
        main.cpp
        xArmServoController.cpp
        xArmServosRad.cpp
        xArmAgent.cpp
        
        Agent.cpp
        BlinkAgent.cpp
		pico_usb_transport.c
		uRosBridge.cpp
		freertos_allocators.c
		uRosEntities.cpp
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
    pico_stdlib 
    hardware_pwm
	FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
    freertos_config
    micro_ros
    )

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 1)
target_compile_definitions(${NAME} PRIVATE
    PICO_DEFAULT_UART_RX_PIN=17
    PICO_DEFAULT_UART_TX_PIN=16
)
