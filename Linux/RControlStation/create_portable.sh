#!/bin/bash

# Simple script to create a portable bundle for RControlStation

set -e

APP_NAME="RControlStation"
BUILD_DIR="build/Desktop_Qt_6_9_1-release"
BUNDLE_DIR="RControlStation-Portable"
TARBALL_NAME="RControlStation-Portable.tar.gz"

echo "Creating portable bundle for RControlStation..."

# Clean up
rm -rf "$BUNDLE_DIR"
rm -f "$TARBALL_NAME"

# Create structure
mkdir -p "$BUNDLE_DIR/lib"

# Copy executable
echo "Copying executable..."
cp "$BUILD_DIR/$APP_NAME" "$BUNDLE_DIR/"

# Copy libraries
echo "Copying libraries..."
cp -r "$BUILD_DIR/lib/"* "$BUNDLE_DIR/lib/" 2>/dev/null || true

# Create launch script
cat > "$BUNDLE_DIR/launch.sh" << 'EOF'
#!/bin/bash
export LD_LIBRARY_PATH="$(dirname "$0")/lib:$LD_LIBRARY_PATH"
./RControlStation
EOF

chmod +x "$BUNDLE_DIR/launch.sh"

# Create tarball
echo "Creating archive..."
tar -czf "$TARBALL_NAME" "$BUNDLE_DIR"

echo "Done! Created $TARBALL_NAME"
