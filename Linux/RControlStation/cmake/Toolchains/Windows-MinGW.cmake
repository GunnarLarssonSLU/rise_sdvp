# Toolchain file for Windows MinGW builds
# Usage: cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/Toolchains/Windows-MinGW.cmake

set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# Specify the compiler (adjust paths as needed)
set(CMAKE_C_COMPILER x86_64-w64-mingw32-gcc)
set(CMAKE_CXX_COMPILER x86_64-w64-mingw32-g++)

# Set build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Set find root path for cross-compilation
set(CMAKE_FIND_ROOT_PATH /usr/x86_64-w64-mingw32)

# Adjust default behavior of FIND_XXX commands
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Link OpenGL for Windows
set(OpenGL_GL_PREFERENCE GLVND)
set(OPENGL_INCLUDE_DIR /usr/x86_64-w64-mingw32/include)
set(OPENGL_gl_LIBRARY /usr/x86_64-w64-mingw32/lib/libopengl32.a)

# Set Qt installation path for MinGW
# set(Qt6_DIR /path/to/qt6/mingw/lib/cmake/Qt6)

message(STATUS "Configuring for Windows MinGW cross-compilation")
message(STATUS "  C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "  C++ Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Find Root Path: ${CMAKE_FIND_ROOT_PATH}")
