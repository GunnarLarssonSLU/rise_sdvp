# RControlStation

Remote Control Station Application for vehicle control and monitoring.

## Overview

RControlStation is a Qt-based application for controlling and monitoring remote vehicles. It provides interfaces for GPS, sensor data, network communication, and vehicle management.

## Features

- Real-time vehicle monitoring
- GPS and sensor data visualization
- Network communication (TCP, UDP, serial)
- RTCM3 correction data handling
- Map and route management
- Database integration for vehicle configurations

## Requirements

### Common Dependencies
- C++17 compatible compiler
- CMake >= 3.16
- Qt 6 (or Qt 5 with `USE_QT5=ON`)

### Required Qt Components
- Qt Core
- Qt Gui
- Qt Widgets
- Qt PrintSupport
- Qt Network
- Qt Sql
- Qt SerialPort
- Qt Charts (for Qt6)

### Optional Qt Components
- Qt OpenGLWidgets
- Qt Gamepad

### External Dependencies
- SDL2 (for gamepad support)
- GDAL (for geospatial data)
- Eigen3 (for matrix operations)
- Various system libraries (zlib, png, jpeg, curl, etc.)

## Installation

### Linux (Debian/Ubuntu)

#### Install Dependencies

```bash
# For Qt6 (recommended)
sudo apt update
sudo apt install -y build-essential cmake \
    qt6-base-dev qt6-base-private-dev \
    qt6-serialport-dev qt6-charts-dev \
    libsdl2-dev libgdal-dev \
    libeigen3-dev \
    libsqlite3-dev libssl-dev libcurl4-openssl-dev \
    zlib1g-dev libpng-dev libjpeg-dev

# For Qt5 (if needed)
# sudo apt install -y qt5-default qt5-qmake qtbase5-dev qtbase5-private-dev \
#     qtserialport5-dev libqt5charts5-dev
```

#### Build and Install

```bash
# Clone the repository (if not already done)
git clone <repository-url>
cd RControlStation

# Create build directory
mkdir build && cd build

# Configure with CMake (Qt6)
cmake .. -DCMAKE_BUILD_TYPE=Release

# Or for Qt5
# cmake .. -DUSE_QT5=ON -DCMAKE_BUILD_TYPE=Release

# Build
make -j$(nproc)

# Install system-wide (optional)
sudo make install

# Or create a .deb package
cpack -G DEB

# Or create an .rpm package (on Fedora/RHEL)
cpack -G RPM
```

### Windows

#### Using MSYS2/MinGW

1. Install [MSYS2](https://www.msys2.org/)
2. Update packages:
   ```bash
   pacman -Syu
   pacman -S --needed base-devel mingw-w64-x86_64-toolchain mingw-w64-x86_64-cmake
   ```
3. Install dependencies:
   ```bash
   pacman -S mingw-w64-x86_64-qt6 mingw-w64-x86_64-qt6-serialport \
           mingw-w64-x86_64-SDL2 mingw-w64-x86_64-gdal
   ```
4. Build:
   ```bash
   mkdir build && cd build
   cmake -G "MinGW Makefiles" ..
   mingw32-make -j$(nproc)
   ```
5. Create installer:
   ```bash
   cpack -G NSIS
   ```

#### Using Visual Studio

1. Install Visual Studio with C++ support
2. Install Qt for Windows
3. Install vcpkg and required packages:
   ```bash
   vcpkg install qt6 sdl2 gdal eigen3
   ```
4. Configure with CMake:
   ```bash
   cmake -B build -G "Visual Studio 17 2022" -DCMAKE_TOOLCHAIN_FILE=[vcpkg-root]/scripts/buildsystems/vcpkg.cmake
   cmake --build build --config Release
   ```

### macOS

#### Using Homebrew

```bash
# Install dependencies
brew install cmake qt@6 sdl2 gdal eigen

# Build
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=$(brew --prefix qt@6)
make -j$(sysctl -n hw.ncpu)

# Create DMG package
cpack -G DragNDrop
```

## Build Options

| Option | Description | Default |
|--------|-------------|---------|
| `USE_QT5` | Use Qt5 instead of Qt6 | OFF |
| `HAS_OPENGL` | Enable OpenGL support | OFF |
| `HAS_ASSIMP` | Enable Assimp support | OFF |
| `HAS_SIM_SCEN` | Enable simulation scenarios | OFF |
| `CMAKE_BUILD_TYPE` | Build type (Debug/Release) | (not set) |

Example:
```bash
cmake .. -DUSE_QT5=ON -DHAS_OPENGL=ON -DCMAKE_BUILD_TYPE=Debug
```

## Running

After building, the executable will be in the `build/bin/` directory:

```bash
# From build directory
./bin/RControlStation

# Or if installed system-wide
RControlStation
```

## Package Generation

After building, you can generate installable packages:

```bash
# Debian package
cpack -G DEB

# RPM package
cpack -G RPM

# Windows installer
cpack -G NSIS

# macOS DMG
cpack -G DragNDrop
```

Packages will be created in the build directory.

## Troubleshooting

### Common Issues

1. **Qt not found**: Ensure Qt is installed and `CMAKE_PREFIX_PATH` is set correctly.
   ```bash
   cmake .. -DCMAKE_PREFIX_PATH=/path/to/qt/installation
   ```

2. **Missing dependencies**: Install the required system packages for your distribution.

3. **GDAL linking issues**: The project requires GDAL compiled with OpenJPEG 2.5.0. You may need to build GDAL from source or adjust the CMake configuration.

4. **SDL2 not found**: Install SDL2 development packages.

### Getting Help

For support, please contact: support@ri.se

## License

This project is proprietary software owned by RISE.

## Contributing

Contributions are welcome. Please contact the project maintainers for access.
