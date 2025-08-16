cmake_minimum_required(VERSION 3.19)

project(RwS3Power)

build_gmpi_plugin(
    PROJECT_NAME ${PROJECT_NAME}
    HAS_DSP
    SOURCE_FILES 
        RwS3Power.cpp
)