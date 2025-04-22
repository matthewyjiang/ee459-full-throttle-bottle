# This is the root CMakeLists.txt file

# The minimum required version of CMake. If you have an older version, ESP-IDF might raise an error.
# 3.16 is generally safe for recent ESP-IDF versions.
cmake_minimum_required(VERSION 3.16)

# Include the ESP-IDF project CMake file which provides the necessary functions.
# Make sure your IDF_PATH environment variable is set correctly
# (usually done by running "export.sh" or "get.sh" from the ESP-IDF directory).
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Declare the project name. This can be anything you like.
project(esp32_master)