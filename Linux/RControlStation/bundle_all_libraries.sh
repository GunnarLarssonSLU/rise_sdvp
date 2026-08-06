#!/bin/bash

# Bundle ALL dependencies for RControlStation
# This ensures the AppImage is completely self-contained

set -e

echo "🔧 Bundling ALL dependencies..."

BUILD_DIR="build/Desktop_Qt_6_9_1-release"
EXECUTABLE="$BUILD_DIR/RControlStation"
LIB_DIR="$BUILD_DIR/lib"

# Create lib directory if it doesn't exist
mkdir -p "$LIB_DIR"

# Get all dependencies
echo "📚 Finding all dependencies..."
DEPS=$(ldd "$EXECUTABLE" | grep -o '/[^ ]*' | grep -v '^/' | sort | uniq)

echo "📋 Copying dependencies..."
COPIED=0
SKIPPED=0

for dep in $DEPS; do
    # Skip system directories and special libraries
    if [[ $dep == /lib* ]] || [[ $dep == /usr/lib* ]] || [[ $dep == linux-vdso.so* ]]; then
        SKIPPED=$((SKIPPED + 1))
        continue
    fi
    
    # Copy the library
    if [ -f "$dep" ]; then
        cp "$dep" "$LIB_DIR/"
        COPIED=$((COPIED + 1))
        echo "   ✅ $(basename "$dep")"
    fi
done

echo "📊 Summary:"
echo "   Copied: $COPIED libraries"
echo "   Skipped: $SKIPPED system libraries"
echo ""
echo "✅ All dependencies bundled in: $LIB_DIR"
