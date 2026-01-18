# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

idf_build_set_property(MINIMAL_BUILD OFF)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(climate_logger)
