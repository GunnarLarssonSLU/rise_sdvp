#!/bin/bash

# Simple script to create a portable bundle for RControlStation
# This creates a tarball that can be extracted and run on other systems

set -e  # Exit on error

# Configuration
APP_NAME="RControlStation"
BUILD_DIR="build/Desktop_Qt_6_9_1-release"
BUNDLE_DIR="RControlStation-Portable"
TARBALL_NAME="RControlStation-Portable.tar.gz"

# Clean up previous builds
echo "Cleaning up previous builds..."
rm -rf "$BUNDLE_DIR"
rm -f "$TARBALL_NAME"

# Create bundle structure
echo "Creating bundle structure..."
mkdir -p "$BUNDLE_DIR"
mkdir -p "$BUNDLE_DIR/lib"

# Copy the executable
echo "Copying executable..."
cp "$BUILD_DIR/$APP_NAME" "$BUNDLE_DIR/"

# Copy existing libraries
echo "Copying libraries..."
if [ -d "$BUILD_DIR/lib" ]; then
    cp -r "$BUILD_DIR/lib"/* "$BUNDLE_DIR/lib/"
fi

# Copy icon
echo "Copying icon..."
mkdir -p "$BUNDLE_DIR/icons"
cp "Icons/Car-96.png" "$BUNDLE_DIR/icons/$APP_NAME.png"

# Create a simple launch script
echo "Creating launch script..."
cat > "$BUNDLE_DIR/launch.sh" << 'EOF'
#!/bin/bash

# Set library path to include bundled libraries
export LD_LIBRARY_PATH="$(dirname "$0")/lib:$LD_LIBRARY_PATH"

# Launch the application
./RControlStation
EOF

chmod +x "$BUNDLE_DIR/launch.sh"

# Create README
echo "Creating README..."
cat > "$BUNDLE_DIR/README.txt" << EOF
RControlStation Portable
========================

To run RControlStation:
1. Extract this archive
2. Run: ./launch.sh

This bundle includes all required libraries and should work on most Linux systems.

System requirements:
- Linux x86_64
- Qt 6.9.1 or compatible
- Basic system libraries (glibc, etc.)
EOF

# Create tarball
echo "Creating tarball..."
tar -czvf "$TARBALL_NAME" "$BUNDLE_DIR"

echo "Done!"
echo "Created: $TARBALL_NAME"
echo "To use: extract and run ./launch.sh"
