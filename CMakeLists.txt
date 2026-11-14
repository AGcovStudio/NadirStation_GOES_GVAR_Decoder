cmake_minimum_required(VERSION 3.21)
project(NadirStation_GOES_GVAR_Decoder VERSION 1.0.0 LANGUAGES C CXX)

# MinGW Makefiles is a single-configuration generator: default to Release so the per-sample DSP is optimised.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)
qt_standard_project_setup()

if(MSVC)
    set(GVAR_WARN /W4 /permissive-)
else()
    set(GVAR_WARN -Wall -Wextra)
endif()

# ---- receive chain: demodulator, block sync, image assembly (Qt Core/Gui only, no widgets) --------------------------
add_library(gvar_core STATIC
    src/dsp.cpp src/dsp.h
    src/gvarengine.cpp src/gvarengine.h
    src/gvardecoder.h src/gvardoc.h src/gvarimage.h src/softpixel.h src/gvar_fc_lut.inc
    src/iqreader.cpp src/iqreader.h
    src/runtime_logger.cpp src/runtime_logger.h)
target_include_directories(gvar_core PUBLIC src)
target_compile_options(gvar_core PRIVATE ${GVAR_WARN})
find_package(Threads REQUIRED)
target_link_libraries(gvar_core PUBLIC Threads::Threads Qt6::Core Qt6::Gui)

# ---- the application -----------------------------------------------------------------------------------------------
set(APP_SOURCES
    src/main.cpp src/mainwindow.cpp src/mainwindow.h
    src/gvarworker.cpp src/gvarworker.h
    src/rtlsdrreceiver.cpp src/rtlsdrreceiver.h src/spectrumtap.h
    src/constellationwidget.cpp src/constellationwidget.h
    src/spectrumview.cpp src/spectrumview.h
    src/appinfo.h src/qtpath.h
    resources/app.qrc)
if(WIN32)
    enable_language(RC)
    list(APPEND APP_SOURCES resources/app.rc)
endif()
qt_add_executable(GvarDecoder WIN32 ${APP_SOURCES})
set_target_properties(GvarDecoder PROPERTIES OUTPUT_NAME NadirStation_GOES_GVAR_Decoder)
target_compile_options(GvarDecoder PRIVATE ${GVAR_WARN})
target_link_libraries(GvarDecoder PRIVATE gvar_core Qt6::Core Qt6::Gui Qt6::Widgets)

# Command-line check of the receive chain (no window): gvar_cli <recording> [sample rate in Hz] [output folder]
add_executable(GvarCli tests/gvar_cli.cpp)
target_link_libraries(GvarCli PRIVATE gvar_core)
set_target_properties(GvarCli PROPERTIES OUTPUT_NAME gvar_cli)

install(TARGETS GvarDecoder RUNTIME DESTINATION .)
