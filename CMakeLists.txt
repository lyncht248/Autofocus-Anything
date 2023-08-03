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
#pkg_check_modules(SDLGFX SDL2_gfx)

### Commented these out
#pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)
#include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})

include_directories(${GTKMM_INCLUDE_DIRS})
link_directories (${GTKMM_LIBRARY_DIRS})
add_definitions (${GTKMM_CFLAGS_OTHER})

include_directories(${TOON_INCLUDE_DIRS})
link_directories (${TOON_LIBRARY_DIRS})
add_definitions (${TOON_CFLAGS_OTHER})

include_directories(${SDL_INCLUDE_DIRS})
link_directories (${SDL_LIBRARY_DIRS})
add_definitions (${SDL_CFLAGS_OTHER})

#include_directories(${SDLGFX_INCLUDE_DIRS})
#link_directories (${SDLGFX_LIBRARY_DIRS})
#add_definitions (${SDLGFX_CFLAGS_OTHER})

include_directories(Vimba_6_0)

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

add_executable(hvi-gtk main.cpp mainwindow.cpp sdlwindow.cpp sdlwinchild.cpp cond.cpp system.cpp recorder.cpp thread.cpp stabiliser.cpp vesseledgel.cpp autofocus.cpp lens.cpp tiltedcam.cpp pid.cpp)

configure_file(${CMAKE_SOURCE_DIR}/Vimba_6_0/VimbaGigETL.cti ${CMAKE_BINARY_DIR}/VimbaGigETL.cti COPYONLY)

#target_compile_options(hvi-gtk PRIVATE -Wall -Wextra)
target_link_libraries(hvi-gtk ${GTKMM_LIBRARIES} ${TOON_LIBRARIES} ${OpenCV_LIBS} ${SDL_LIBRARIES} ${FMT_LIBRARIES} ${SPDLOG_LIBRARIES} cvd VimbaCPP rt pthread ${CMAKE_SOURCE_DIR}/libASICamera2.a -lusb-1.0)
