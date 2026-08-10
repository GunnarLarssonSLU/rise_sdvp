# Toolchain file for Android ARM builds
# Usage: cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/Toolchains/Android-ARM.cmake

set(CMAKE_SYSTEM_NAME Android)
set(CMAKE_SYSTEM_PROCESSOR armv7-a)

# Specify the Android NDK path and compiler
set(ANDROID_NDK /opt/android-ndk)
set(ANDROID_ABI armeabi-v7a)
set(ANDROID_PLATFORM android-21)

# Set compiler
set(CMAKE_C_COMPILER ${ANDROID_NDK}/toolchains/arm-linux-androideabi-4.9/prebuilt/linux-x86_64/bin/arm-linux-androideabi-gcc)
set(CMAKE_CXX_COMPILER ${ANDROID_NDK}/toolchains/arm-linux-androideabi-4.9/prebuilt/linux-x86_64/bin/arm-linux-androideabi-g++)

# Set build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Set find root path
set(CMAKE_FIND_ROOT_PATH ${ANDROID_NDK}/sysroot)

# Adjust default behavior of FIND_XXX commands
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Set Android-specific flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdata-sections -ffunction-sections")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Set linker flags
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

# Set Qt installation path for Android
# set(Qt6_DIR /path/to/qt6/android/lib/cmake/Qt6)

message(STATUS "Configuring for Android ARM build")
message(STATUS "  C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "  C++ Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Android ABI: ${ANDROID_ABI}")
message(STATUS "  Android Platform: ${ANDROID_PLATFORM}")
