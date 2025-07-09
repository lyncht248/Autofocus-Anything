cmake_minimum_required(VERSION 3.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

project(hvi-gtk)

find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)

set(OpenGL_GL_PREFERENCE LEGACY) #Using the legacy LibGL.so. Chage to GLVND for LibOpenGL.so or LibGLX.so
find_package(OpenGL REQUIRED)
pkg_check_modules(GTKMM gtkmm-3.0)
pkg_check_modules(TOON TooN)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${GTKMM_INCLUDE_DIRS})
link_directories (${GTKMM_LIBRARY_DIRS})
add_definitions (${GTKMM_CFLAGS_OTHER})

include_directories(${TOON_INCLUDE_DIRS})
link_directories (${TOON_LIBRARY_DIRS})
add_definitions (${TOON_CFLAGS_OTHER})

include_directories(Vimba_6_0)

add_executable(hvi-gtk main.cpp mainwindow.cpp cond.cpp system.cpp recorder.cpp thread.cpp stabiliser.cpp vesseledge.cpp)

configure_file(${CMAKE_SOURCE_DIR}/Vimba_6_0/VimbaGigETL.cti ${CMAKE_BINARY_DIR}/VimbaGigETL.cti COPYONLY)

#target_compile_options(hvi-gtk PRIVATE -Wall -Wextra)
target_link_libraries (hvi-gtk ${GTKMM_LIBRARIES} ${TOON_LIBRARIES} cvd VimbaCPP GL ${OpenCV_LIBS})

