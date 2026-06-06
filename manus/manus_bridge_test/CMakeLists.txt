cmake_minimum_required(VERSION 3.20)

project(manus_bridge_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT WIN32)
    message(FATAL_ERROR "manus_bridge_test first stage supports Windows x64 + MANUS Core only.")
endif()

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "Please configure a 64-bit build, for example: cmake -S . -B build -A x64")
endif()

set(MANUS_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ManusSDK" CACHE PATH "Path to MANUS SDK root containing include/ and lib/")
set(MANUS_SDK_INCLUDE_DIR "${MANUS_SDK_ROOT}/include" CACHE PATH "Path to MANUS SDK include directory")
set(MANUS_SDK_LIB_DIR "${MANUS_SDK_ROOT}/lib" CACHE PATH "Path to MANUS SDK lib directory")
set(MANUS_SDK_LIBRARY "${MANUS_SDK_LIB_DIR}/ManusSDK.lib" CACHE FILEPATH "Path to ManusSDK.lib")
set(MANUS_SDK_DLL "${MANUS_SDK_LIB_DIR}/ManusSDK.dll" CACHE FILEPATH "Path to ManusSDK.dll")

if(NOT EXISTS "${MANUS_SDK_INCLUDE_DIR}/ManusSDK.h")
    message(FATAL_ERROR
        "Could not find ManusSDK.h.\n"
        "Set -DMANUS_SDK_ROOT=C:/path/to/ManusSDK or copy the SDK into third_party/ManusSDK.\n"
        "Expected: ${MANUS_SDK_INCLUDE_DIR}/ManusSDK.h")
endif()

if(NOT EXISTS "${MANUS_SDK_LIBRARY}")
    message(FATAL_ERROR
        "Could not find ManusSDK.lib.\n"
        "Set -DMANUS_SDK_LIBRARY=C:/path/to/ManusSDK.lib or check MANUS_SDK_ROOT.\n"
        "Expected: ${MANUS_SDK_LIBRARY}")
endif()

add_executable(manus_bridge_test
    src/main.cpp
    src/ManusClient.cpp
    src/DataPrinter.cpp
    src/CsvRecorder.cpp
    src/TcpSender.cpp
)

target_include_directories(manus_bridge_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${MANUS_SDK_INCLUDE_DIR}"
)

target_link_libraries(manus_bridge_test PRIVATE "${MANUS_SDK_LIBRARY}" ws2_32)

if(EXISTS "${MANUS_SDK_DLL}")
    add_custom_command(TARGET manus_bridge_test POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${MANUS_SDK_DLL}"
            "$<TARGET_FILE_DIR:manus_bridge_test>/ManusSDK.dll"
        COMMENT "Copying ManusSDK.dll next to manus_bridge_test.exe"
    )
else()
    message(WARNING "ManusSDK.dll was not found at ${MANUS_SDK_DLL}. Put it next to the exe or add it to PATH before running.")
endif()
