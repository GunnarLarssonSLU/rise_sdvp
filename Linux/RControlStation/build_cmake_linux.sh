#!/bin/bash

# CMake build script for Linux
# Usage: ./build_cmake_linux.sh [debug|release] [clean]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build/cmake_linux"
BUILD_TYPE="Release"
CLEAN=false

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
cmake --build . --config "$BUILD_TYPE" --parallel $(nproc)

# Copy resources if needed
echo "Build completed successfully!"
echo "Executable: ${BUILD_DIR}/bin/RControlStation"
