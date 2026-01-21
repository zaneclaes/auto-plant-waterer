# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Tell esp-matter where connectedhomeip root is (this is what zzz_generated lives under)
set(MATTER_SDK_PATH "$ENV{CHIP_ROOT}" CACHE PATH "connectedhomeip root" FORCE)

# (optional, but often helpful if other logic looks for it)
set(ESP_MATTER_PATH "$ENV{ESP_MATTER_PATH}" CACHE PATH "esp-matter root" FORCE)

# Point IDF at the esp-matter component set (NOT the repo root)
set(EXTRA_COMPONENT_DIRS)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(auto-plant-waterer)
