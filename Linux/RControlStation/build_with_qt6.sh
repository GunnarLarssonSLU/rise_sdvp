#!/bin/bash

# Build RControlStation with Qt 6
# This script ensures the correct Qt version is used

set -e

echo "🔧 Setting up Qt 6 build environment..."

# Path to Qt 6 installation
QT6_PATH="/home/gunnar/Qt/6.9.1/gcc_64"

# Set up environment to use Qt 6
export PATH="$QT6_PATH/bin:$PATH"
export LD_LIBRARY_PATH="$QT6_PATH/lib:$LD_LIBRARY_PATH"

echo "✅ Qt 6 path: $QT6_PATH"
echo "✅ Using qmake: $(which qmake)"
echo "✅ Qt version: $(qmake -version)"

# Clean previous build
echo "🧹 Cleaning previous build..."
rm -rf build/Desktop_Qt_6_9_1-release
mkdir -p build/Desktop_Qt_6_9_1-release

# Build with Qt 6
echo "🛠️ Building with Qt 6..."
cd build/Desktop_Qt_6_9_1-release
$QT6_PATH/bin/qmake ../../RControlStation.pro -spec linux-g++ CONFIG+=release
make -j$(nproc)

# Copy back to main directory
echo "📋 Copying build artifacts..."
cp RControlStation ../../
cp -r lib ../../ 2>/dev/null || echo "No lib directory to copy"

cd ../..

echo "✅ Build completed successfully!"
echo "📦 Executable: $(ls -lh RControlStation | awk '{print $9, $5}')"
echo "🎯 Ready to create AppImage"
