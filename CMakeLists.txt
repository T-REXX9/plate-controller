cmake_minimum_required(VERSION 3.18)
project(plate_reader_cpp LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(PLATE_ENABLE_CAMERA "Build real-time camera support" ON)
option(PLATE_RASPBERRY_PI "Tune the build for Raspberry Pi 4" OFF)
option(PLATE_ENABLE_GPIO "Build Raspberry Pi gate GPIO support" OFF)

set(PLATE_OPENCV_COMPONENTS core imgproc imgcodecs dnn)
if(PLATE_ENABLE_CAMERA)
    list(APPEND PLATE_OPENCV_COMPONENTS videoio highgui)
endif()
find_package(OpenCV REQUIRED COMPONENTS ${PLATE_OPENCV_COMPONENTS})
if(OpenCV_VERSION VERSION_LESS "4.10")
    message(FATAL_ERROR "OpenCV 4.10 or newer is required (found ${OpenCV_VERSION})")
endif()

add_executable(plate_reader src/main.cpp)
target_link_libraries(plate_reader PRIVATE ${OpenCV_LIBS})
target_include_directories(plate_reader PRIVATE ${OpenCV_INCLUDE_DIRS})
if(PLATE_ENABLE_CAMERA)
    find_package(CURL REQUIRED)
    target_compile_definitions(plate_reader PRIVATE PLATE_ENABLE_CAMERA=1)
    target_link_libraries(plate_reader PRIVATE CURL::libcurl)
endif()

add_library(gate_control src/gate_controller.cpp)
target_include_directories(gate_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(plate_reader PRIVATE gate_control)

if(PLATE_ENABLE_GPIO)
    find_package(PkgConfig REQUIRED)
    find_package(Threads REQUIRED)
    pkg_check_modules(GPIOD REQUIRED IMPORTED_TARGET libgpiod)
    target_sources(
        plate_reader
        PRIVATE
            src/status_leds.cpp
            src/gate_gpio.cpp
            src/serial_rfid_reader.cpp
    )
    target_compile_definitions(plate_reader PRIVATE PLATE_ENABLE_GPIO=1)
    if(GPIOD_VERSION VERSION_LESS "2.0")
        target_compile_definitions(plate_reader PRIVATE PLATE_GPIOD_V1=1)
    else()
        target_compile_definitions(plate_reader PRIVATE PLATE_GPIOD_V2=1)
    endif()
    target_link_libraries(
        plate_reader PRIVATE PkgConfig::GPIOD Threads::Threads
    )

    add_executable(
        gate_hardware_diagnostic
        src/gate_hardware_diagnostic.cpp
        src/gate_gpio.cpp
    )
    target_include_directories(
        gate_hardware_diagnostic PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    if(GPIOD_VERSION VERSION_LESS "2.0")
        target_compile_definitions(
            gate_hardware_diagnostic PRIVATE PLATE_GPIOD_V1=1
        )
    else()
        target_compile_definitions(
            gate_hardware_diagnostic PRIVATE PLATE_GPIOD_V2=1
        )
    endif()
    target_link_libraries(gate_hardware_diagnostic PRIVATE PkgConfig::GPIOD)
endif()

add_executable(gate_simulator src/gate_simulator.cpp)
target_link_libraries(gate_simulator PRIVATE gate_control)

if(BUILD_TESTING)
    add_executable(gate_controller_tests tests/gate_controller_tests.cpp)
    target_link_libraries(gate_controller_tests PRIVATE gate_control)
    add_test(NAME gate_controller_safety COMMAND gate_controller_tests)
    add_executable(
        serial_rfid_reader_tests
        tests/serial_rfid_reader_tests.cpp
        src/serial_rfid_reader.cpp
    )
    target_include_directories(
        serial_rfid_reader_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    add_test(NAME serial_rfid_reader_parser COMMAND serial_rfid_reader_tests)
endif()

if(PLATE_RASPBERRY_PI AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(plate_reader PRIVATE -mcpu=cortex-a72)
endif()
