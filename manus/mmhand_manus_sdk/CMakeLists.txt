cmake_minimum_required(VERSION 3.20)
project(mmhand_manus_sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MANUS_SDK_ROOT
    "${CMAKE_CURRENT_LIST_DIR}/../third_party/ManusSDK"
    CACHE PATH "Path to the MANUS SDK root containing include/ and lib/")

add_library(mmhand_manus_sdk SHARED
    src/mmhand_manus_sdk.cpp
    ../manus_bridge_test/src/ManusClient.cpp
)

target_compile_definitions(mmhand_manus_sdk PRIVATE MMHAND_MANUS_SDK_EXPORTS)

target_include_directories(mmhand_manus_sdk PRIVATE
    include
    ../manus_bridge_test/src
    "${MANUS_SDK_ROOT}/include"
)

if (WIN32)
    target_link_directories(mmhand_manus_sdk PRIVATE "${MANUS_SDK_ROOT}/lib")
    target_link_libraries(mmhand_manus_sdk PRIVATE ManusSDK)
    add_custom_command(TARGET mmhand_manus_sdk POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${MANUS_SDK_ROOT}/lib/ManusSDK.dll"
            "$<TARGET_FILE_DIR:mmhand_manus_sdk>/ManusSDK.dll"
        VERBATIM)
endif()
