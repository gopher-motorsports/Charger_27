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

# Set the project name
set(CMAKE_PROJECT_NAME Charger_27)


# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Gather all project source files
file(GLOB MAIN_SOURCES
    "${CMAKE_SOURCE_DIR}/Core/Src/*.c"
)
list(REMOVE_ITEM MAIN_SOURCES "${CMAKE_SOURCE_DIR}/Core/Src/system_stm32f4xx.c")

# Gather all gophercan source files
file(GLOB GOPHERCAN_SOURCES
    "${CMAKE_SOURCE_DIR}/../gophercan-lib/*.c"
)

# Gather all gopher sense source files
file(GLOB GOPHERSENSE_SOURCES
    "${CMAKE_SOURCE_DIR}/../Gopher_Sense/lib/*.c"
    "${CMAKE_SOURCE_DIR}/../Gopher_Sense/Build/*.c"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${MAIN_SOURCES}
    ${GOPHERCAN_SOURCES}
    ${GOPHERSENSE_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    "${CMAKE_SOURCE_DIR}/../gophercan-lib"
    "${CMAKE_SOURCE_DIR}/../Gopher_Sense/lib"
    "${CMAKE_SOURCE_DIR}/../Gopher_Sense/Build"
)

# Bring-up test firmware for the board test plan. Off by default, so the normal build is unchanged.
# Enable with -DBRINGUP_TEST=ON; the result is <project>_bringup.elf. The HAL I2C driver is not part
# of the generated project, so it is added here instead of in the CubeMX configuration.
option(BRINGUP_TEST "Build the bring-up test firmware instead of the charger application" OFF)
if(BRINGUP_TEST)
    file(GLOB BRINGUP_SOURCES "${CMAKE_SOURCE_DIR}/bringup/*.c")
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${BRINGUP_SOURCES}
        "${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c"
    )
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/bringup")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE BRINGUP_TEST HAL_I2C_MODULE_ENABLED)
    set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME ${CMAKE_PROJECT_NAME}_bringup)
endif()

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Enable floating point support for printf
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u _printf_float")
# set(CMAKE_EXE_LINKER_FLAGS
#     "${CMAKE_EXE_LINKER_FLAGS} -u _printf_float -Wl,--specs=nano.specs -Wl,--specs=nosys.specs"
# )
# Convert output to hex and binary
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> $<TARGET_FILE_BASE_NAME:${PROJECT_NAME}>.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> $<TARGET_FILE_BASE_NAME:${PROJECT_NAME}>.bin
)
