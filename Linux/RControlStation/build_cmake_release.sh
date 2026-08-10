#!/bin/bash

# CMake release build script for Linux with static linking
# Usage: ./build_cmake_release.sh [clean]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build/cmake_release"

# Parse arguments
CLEAN=false
for arg in "$@"; do
    case "$arg" in
        clean)
            CLEAN=true
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [clean]"
            exit 1
            ;;
    esac
done

echo "Building RControlStation for Linux (Release with static linking)"
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
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE="${SCRIPT_DIR}/cmake/Toolchains/Linux-GCC.cmake" \
    -DHAS_OPENGL=OFF \
    -DHAS_ASSIMP=OFF \
    -DHAS_SIM_SCEN=OFF \
    "$SCRIPT_DIR"

# Build
echo "Building..."
cmake --build . --config Release --parallel $(nproc)

echo "Build completed successfully!"
echo "Executable: ${BUILD_DIR}/bin/RControlStation"
