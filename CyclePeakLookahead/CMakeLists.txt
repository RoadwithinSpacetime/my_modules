
cmake_minimum_required(VERSION 3.20)
project(CyclePeakLookahead_SEM LANGUAGES CXX)

if(NOT DEFINED SE_SDK_DIR)
    set(SE_SDK_DIR "C:/SynthEdit_SDK3" CACHE PATH "Path to SynthEdit SDK v3")
endif()

set(TARGET_NAME CyclePeakLookahead)

add_library(${TARGET_NAME} SHARED
    src/CyclePeakLookahead.cpp
    src/factory.cpp
)

target_include_directories(${TARGET_NAME} PRIVATE
    "${SE_SDK_DIR}"
    "${SE_SDK_DIR}/modules/shared"
)

if(MSVC)
    target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)
    target_compile_options(${TARGET_NAME} PRIVATE /permissive- /W3 /EHsc)
    target_compile_definitions(${TARGET_NAME} PRIVATE _WINDOWS UNICODE _UNICODE)
    set_target_properties(${TARGET_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        OUTPUT_NAME "${TARGET_NAME}"
        SUFFIX ".sem"
    )
endif()

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_SOURCE_DIR}/src/module.xml"
            "$<TARGET_FILE_DIR:${TARGET_NAME}>/module.xml"
)

