cmake_minimum_required(VERSION 3.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)
#Autofocus_v9 used Release

project(hvi-gtk)

###
find_package(OpenCV REQUIRED )

find_package(PkgConfig REQUIRED)

pkg_check_modules(GTKMM gtkmm-3.0)
pkg_check_modules(TOON TooN)
pkg_check_modules(SDL sdl2)
pkg_check_modules(FMT fmt)
pkg_check_modules(SPDLOG spdlog)
pkg_check_modules(SDL2TTF SDL2_ttf)
#pkg_check_modules(SDLGFX SDL2_gfx)

### Commented these out
#pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)
#include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})

# Add our project include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

include_directories(${GTKMM_INCLUDE_DIRS})
link_directories (${GTKMM_LIBRARY_DIRS})
add_definitions (${GTKMM_CFLAGS_OTHER})

include_directories(${TOON_INCLUDE_DIRS})
link_directories (${TOON_LIBRARY_DIRS})
add_definitions (${TOON_CFLAGS_OTHER})

include_directories(${SDL_INCLUDE_DIRS})
link_directories (${SDL_LIBRARY_DIRS})
add_definitions (${SDL_CFLAGS_OTHER})

include_directories(${SDL2TTF_INCLUDE_DIRS})
link_directories(${SDL2TTF_LIBRARY_DIRS})
add_definitions(${SDL2TTF_CFLAGS_OTHER})

#include_directories(${SDLGFX_INCLUDE_DIRS})
#link_directories (${SDLGFX_LIBRARY_DIRS})
#add_definitions (${SDLGFX_CFLAGS_OTHER})

# Update Vimba path
include_directories(${CMAKE_SOURCE_DIR}/lib)

include_directories(${FMT_INCLUDE_DIRS})
link_directories(${FMT_LIBRARY_DIRS})

include_directories(${SPDLOG_INCLUDE_DIRS})
link_directories(${SPDLOG_LIBRARY_DIRS})


#added -pg to profile using gprof
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
# SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
# SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

#added -g to use valgrind
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS}")


# Add Xeryon library with updated file paths
add_library(xeryon_lib
    ${CMAKE_SOURCE_DIR}/src/Xeryon.cpp
    ${CMAKE_SOURCE_DIR}/src/Distance.cpp
    ${CMAKE_SOURCE_DIR}/src/Axis.cpp
    ${CMAKE_SOURCE_DIR}/src/Stage.cpp
)

# Set C++11 standard for Xeryon library (required for std::future, etc.)
target_compile_features(xeryon_lib PUBLIC cxx_std_11)

# Copy settings file from config directory
configure_file(${CMAKE_SOURCE_DIR}/config/settings_default.txt ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)

# Define source files
set(SOURCES
    ${CMAKE_SOURCE_DIR}/src/main.cpp 
    ${CMAKE_SOURCE_DIR}/src/main_XERYON.cpp
    ${CMAKE_SOURCE_DIR}/src/mainwindow.cpp 
    ${CMAKE_SOURCE_DIR}/src/sdlwindow.cpp 
    ${CMAKE_SOURCE_DIR}/src/sdlwinchild.cpp 
    ${CMAKE_SOURCE_DIR}/src/cond.cpp 
    ${CMAKE_SOURCE_DIR}/src/system.cpp 
    ${CMAKE_SOURCE_DIR}/src/recorder.cpp 
    ${CMAKE_SOURCE_DIR}/src/thread.cpp 
    ${CMAKE_SOURCE_DIR}/src/stabiliser.cpp 
    ${CMAKE_SOURCE_DIR}/src/vesseledgel.cpp 
    ${CMAKE_SOURCE_DIR}/src/autofocus.cpp 
    ${CMAKE_SOURCE_DIR}/src/lens.cpp 
    ${CMAKE_SOURCE_DIR}/src/tiltedcam.cpp 
    ${CMAKE_SOURCE_DIR}/src/pid.cpp
    ${CMAKE_SOURCE_DIR}/src/Distance.cpp
    ${CMAKE_SOURCE_DIR}/src/Xeryon.cpp
    ${CMAKE_SOURCE_DIR}/src/Axis.cpp
    ${CMAKE_SOURCE_DIR}/src/Stage.cpp
    ${CMAKE_SOURCE_DIR}/src/phasecorr_stabiliser.cpp
)

# Add the executable with source files
add_executable(hvi-gtk ${SOURCES})

# Update file path for Vimba config
configure_file(${CMAKE_SOURCE_DIR}/lib/VimbaGigETL.cti ${CMAKE_BINARY_DIR}/VimbaGigETL.cti COPYONLY)

#target_compile_options(hvi-gtk PRIVATE -Wall -Wextra)
target_link_libraries(hvi-gtk ${GTKMM_LIBRARIES} ${TOON_LIBRARIES} ${OpenCV_LIBS} ${SDL_LIBRARIES} ${SDL2TTF_LIBRARIES} ${FMT_LIBRARIES} ${SPDLOG_LIBRARIES} cvd VimbaCPP rt pthread xeryon_lib ${CMAKE_SOURCE_DIR}/lib/libASICamera2.a -lusb-1.0)

# Copy settings file to build directory
configure_file(${CMAKE_SOURCE_DIR}/config/settings_default.txt ${CMAKE_BINARY_DIR}/settings_default.txt COPYONLY)

# Add Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
FetchContent_MakeAvailable(googletest)

# Enable testing
enable_testing()

# Create logs directory for tests
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/logs)

# Create a library target for testing tiltedcam
add_library(tiltedcam_lib STATIC
    ${CMAKE_SOURCE_DIR}/src/tiltedcam.cpp
    ${CMAKE_SOURCE_DIR}/test/mock_logfile.cpp  # Use the mock implementation
)

target_link_libraries(tiltedcam_lib 
    ${OpenCV_LIBS} 
    ${SDL_LIBRARIES} 
    ${FMT_LIBRARIES} 
    ${SPDLOG_LIBRARIES}
    ${GTKMM_LIBRARIES}
    cvd
    VimbaCPP
    rt 
    pthread 
    ${CMAKE_SOURCE_DIR}/lib/libASICamera2.a 
    -lusb-1.0
)

# Add test executable
add_executable(tiltedcam_test test/tiltedcam_test.cpp)
target_link_libraries(tiltedcam_test PRIVATE 
    gtest_main
    tiltedcam_lib
)

# Register test
include(GoogleTest)
gtest_discover_tests(tiltedcam_test)