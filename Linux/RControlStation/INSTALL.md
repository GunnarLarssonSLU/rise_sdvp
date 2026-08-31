# Building RControlStation on Linux

## Prerequisites (Ubuntu 22.04 LTS)

RControlStation requires the following dependencies to build successfully:

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake \
    qt5-default libqt5charts5-dev libqt5serialport5-dev \
    libsdl2-dev libgdal-dev
```

### Notes on Dependencies
- **Qt5**: The project is configured to use Qt5 by default. Ensure `qmake` and Qt5 development libraries are installed.
- **SDL2**: Required for gamepad support.
- **GDAL**: Required for geospatial data handling. The build system uses `find_package(GDAL REQUIRED)` to locate it.

## Build Steps

### 1. Clone the Repository
```bash
git clone <repository-url>
cd RControlStation
```

### 2. Run the Build Script
The project provides a build script (`build_cmake_linux.sh`) to simplify the build process:

```bash
./build_cmake_linux.sh release
```

This will:
- Create a build directory at `build/cmake_linux`
- Configure the project with CMake
- Build the project in Release mode

### 3. (Optional) Build in Debug Mode
```bash
./build_cmake_linux.sh debug
```

### 4. (Optional) Clean the Build
```bash
./build_cmake_linux.sh clean
```

## Manual Build (Without Script)

If you prefer to build manually:

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --parallel $(nproc)
```

## Troubleshooting

### GDAL/OpenJPEG Conflicts
If you see linker errors related to OpenJPEG (e.g., version mismatches), ensure your GDAL installation is compatible with your system's OpenJPEG version. On Ubuntu 22.04, the system packages are compatible:

```bash
sudo apt install libgdal-dev
```

### Missing Qt Components
If CMake reports missing Qt components (e.g., `Charts`), install the corresponding development packages:

```bash
sudo apt install libqt5charts5-dev
```

### SDL2 Not Found
If SDL2 is missing, install it with:

```bash
sudo apt install libsdl2-dev
```

### Build Logs
Build logs are saved to:
```
build/cmake_linux/CMakeFiles/CMakeOutput.log
```

## Output
The built executable will be located at:
```
build/cmake_linux/bin/RControlStation
```
