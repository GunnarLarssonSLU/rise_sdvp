#!/bin/bash

# CMake build script for Linux
# Usage: ./build_cmake_linux.sh [debug|release] [clean]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build/cmake_linux"
BUILD_TYPE="Release"
CLEAN=false
PARALLEL_JOBS=$(nproc)

# Parse arguments
for arg in "$@"; do
    case "$arg" in
        debug|Debug)
            BUILD_TYPE="Debug"
            ;;
        release|Release)
            BUILD_TYPE="Release"
            ;;
        clean)
            CLEAN=true
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [debug|release] [clean]"
            exit 1
            ;;
    esac
done

echo "Building RControlStation for Linux"
echo "  Build Type: $BUILD_TYPE"
echo "  Build Directory: $BUILD_DIR"

# Check dependencies
check_dependencies() {
    local missing=()
    command -v cmake >/dev/null 2>&1 || missing+=("CMake")
    command -v qmake >/dev/null 2>&1 || missing+=("Qt development tools (qmake)")
    pkg-config --exists Qt5Core >/dev/null 2>&1 || missing+=("Qt5 development libraries")
    pkg-config --exists sdl2 >/dev/null 2>&1 || missing+=("SDL2 (libsdl2-dev)")
    pkg-config --exists gdal >/dev/null 2>&1 || missing+=("GDAL (libgdal-dev)")

    if [ ${#missing[@]} -gt 0 ]; then
        echo "ERROR: Missing dependencies:"
        for dep in "${missing[@]}"; do
            echo "  - $dep"
        done
        echo ""
        echo "Install them on Ubuntu/Debian with:"
        echo "  sudo apt install cmake qt5-default libqt5charts5-dev libsdl2-dev libgdal-dev"
        exit 1
    fi
}
check_dependencies

# Clean if requested
if [ "$CLEAN" = true ]; then
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
    exit 0
fi

# Create build directory
mkdir -p "$BUILD_DIR"

# Configure with CMake
cd "$BUILD_DIR"
echo "Configuring with CMake..."
cmake \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_TOOLCHAIN_FILE="${SCRIPT_DIR}/cmake/Toolchains/Linux-GCC.cmake" \
    -DHAS_OPENGL=OFF \
    -DHAS_ASSIMP=OFF \
    -DHAS_SIM_SCEN=OFF \
    "$SCRIPT_DIR"

# Build
echo "Building..."
cmake --build . --config "$BUILD_TYPE" --parallel $PARALLEL_JOBS

# Build summary
echo ""
echo "=== Build Summary ==="
echo "  Build Type: $BUILD_TYPE"
echo "  Executable: ${BUILD_DIR}/bin/RControlStation"
echo "  Build Log:  ${BUILD_DIR}/CMakeFiles/CMakeOutput.log"
