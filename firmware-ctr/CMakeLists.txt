cmake_minimum_required(VERSION 3.12)

# Change your executable name to something creative!
set(NAME xarmCtr) # <-- Name your project/executable here!

include(pico_sdk_import.cmake)


project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 26)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# Initialize the SDK
pico_sdk_init()

SET(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/../firmware/lib/FreeRTOS-Kernel" CACHE STRING "Course Common Lib")
SET(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel" CACHE STRING "Local Config")
include(${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel/freertos_config.cmake)
include_directories("${FREERTOS_CONFIG_FILE_DIRECTORY}") 
include(FreeRTOS_Kernel_import.cmake)

SET(MICRO_ROS_PATH "${CMAKE_CURRENT_LIST_DIR}/../firmware/lib/micro_ros_raspberrypi_pico_sdk/" CACHE STRING "Course Common Lib")
include(micro_ros.cmake)



add_subdirectory(src)

#Set up files for the release packages
install(CODE "execute_process(COMMAND $ENV{HOME}/bin/picoDeploy.sh ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.elf)")

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.uf2
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)
