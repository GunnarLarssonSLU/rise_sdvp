# CMake Build System for RControlStation

This document describes the CMake build system for RControlStation, which replaces the original qmake-based build system.

## Overview

The CMake build system provides:
- Cross-platform support (Linux, Windows, Android)
- Modern build configuration with CMakePresets
- Modular toolchain files for different platforms
- Support for all the original qmake features

## Prerequisites

### Linux
- CMake >= 3.16
- GCC or Clang
- Qt6 development packages
- All external libraries required by the original .pro file

### Windows (Cross-compilation from Linux)
- MinGW-w64 toolchain
- Qt6 for MinGW
- All external libraries for Windows

### Android
- Android NDK
- Qt6 for Android
- All external libraries for Android

## Build Options

The CMake build system supports the following options:
- `HAS_OPENGL`: Enable OpenGL support (default: OFF)
- `HAS_ASSIMP`: Enable Assimp support (default: OFF)
- `HAS_SIM_SCEN`: Enable simulation scenarios (default: OFF)

## Building

### Using CMakePresets (Recommended)

```bash
# Configure and build for Linux debug
cmake --preset=linux-debug
cmake --build --preset=linux-debug

# Configure and build for Linux release
cmake --preset=linux-release
cmake --build --preset=linux-release

# Configure and build for Linux with OpenGL
cmake --preset=linux-opengl
cmake --build --preset=linux-opengl
```

### Using Build Scripts

```bash
# Debug build
./build_cmake_debug.sh

# Release build
./build_cmake_release.sh

# Clean build
./build_cmake_debug.sh clean
./build_cmake_release.sh clean
```

### Manual CMake Configuration

```bash
# Create build directory
mkdir -p build/cmake_custom
cd build/cmake_custom

# Configure
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchains/Linux-GCC.cmake \
    -DHAS_OPENGL=OFF \
    -DHAS_ASSIMP=OFF \
    -DHAS_SIM_SCEN=OFF \
    ..

# Build
cmake --build . --config Release --parallel $(nproc)
```

### Cross-compilation for Windows

```bash
# Configure for Windows
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE=cmake/Toolchains/Windows-MinGW.cmake \
    -DHAS_OPENGL=ON \
    ..

# Build
cmake --build . --config Release
```

### Cross-compilation for Android

```bash
# Configure for Android
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE=cmake/Toolchains/Android-ARM.cmake \
    -DHAS_OPENGL=OFF \
    ..

# Build
cmake --build . --config Release
```

## Directory Structure

```
RControlStation/
├── CMakeLists.txt              # Main CMake configuration
├── CMakePresets.json           # CMake presets for different configurations
├── cmake/
│   └── Toolchains/
│       ├── Linux-GCC.cmake     # Linux GCC toolchain
│       ├── Windows-MinGW.cmake # Windows MinGW toolchain
│       └── Android-ARM.cmake   # Android ARM toolchain
├── build_cmake_linux.sh       # Legacy build script
├── build_cmake_debug.sh       # Debug build script
├── build_cmake_release.sh     # Release build script
└── CMAKE_BUILD.md              # This file
```

## Migration Notes

### From qmake to CMake

The CMake build system replicates all the functionality from the original `RControlStation.pro` file:

- **Qt Modules**: All Qt modules (Core5Compat, Gui, Widgets, PrintSupport, SerialPort, Network, Quick, Sql, Charts) are included
- **C++ Standard**: C++17 is enforced
- **Definitions**: HAS_JOYSTICK_CHECK and HAS_JOYSTICK are defined by default
- **External Libraries**: All external libraries from the .pro file are linked
- **Include Paths**: All include paths are preserved
- **Output Directories**: Build outputs are placed in appropriate directories

### Static Linking

For Linux release builds, static linking of libstdc++ and libgcc is enabled by default, matching the original qmake configuration.

### Platform-specific Configurations

The original qmake file had platform-specific configurations:
- `release_win`: Windows release configuration
- `release_lin`: Linux release configuration with static linking
- `release_android`: Android release configuration

These are replicated in the CMake toolchain files and build presets.

## Troubleshooting

### Missing Qt Packages

If CMake cannot find Qt packages, ensure Qt6 is installed and the Qt6_DIR variable points to the correct location:

```bash
cmake -DQt6_DIR=/path/to/qt6/installation/lib/cmake/Qt6 ..
```

### Missing External Libraries

The build system expects all external libraries to be available in standard system paths. If libraries are in custom locations, update the CMakeLists.txt file to include the correct paths.

### SDL2 Not Found

For gamepad support, SDL2 is required. Install it with:

```bash
# Ubuntu/Debian
sudo apt-get install libsdl2-dev

# Fedora
sudo dnf install SDL2-devel
```

## Customization

### Adding New Source Files

Add new source files to the `SOURCES` list in CMakeLists.txt:

```cmake
set(SOURCES
    ${SOURCES}
    newfile.cpp
)
```

### Adding New Libraries

Add new libraries to the `EXTERNAL_LIBS` list in CMakeLists.txt:

```cmake
set(EXTERNAL_LIBS
    ${EXTERNAL_LIBS}
    newlibrary
)
```

### Enabling Optional Features

Enable optional features using CMake options:

```bash
cmake -DHAS_OPENGL=ON -DHAS_ASSIMP=ON -DHAS_SIM_SCEN=ON ..
```

## Clean Build

To perform a clean build, remove the build directory:

```bash
rm -rf build/cmake_*
```

Or use the clean option with build scripts:

```bash
./build_cmake_debug.sh clean
./build_cmake_release.sh clean
```

## Installation

To install the built executable:

```bash
cmake --install . --prefix /path/to/install
```

Or manually copy the executable from the build directory.
