#!/bin/bash

# Create a self-contained distributable package for RControlStation
# This creates a single directory that can be zipped and distributed
# Users just need to extract and run the launch script

set -e

echo "🚀 Creating distributable package for RControlStation..."

APP_NAME="RControlStation"
BUILD_DIR="build/Desktop_Qt_6_9_1-release"
DIST_DIR="RControlStation-Distributable"
ZIP_NAME="RControlStation-Linux-x86_64.zip"

# Clean up
echo "🧹 Cleaning up..."
rm -rf "$DIST_DIR"
rm -f "$ZIP_NAME"

# Create distribution structure
echo "📁 Creating distribution structure..."
mkdir -p "$DIST_DIR"
mkdir -p "$DIST_DIR/lib"

# Copy the main executable
echo "📋 Copying executable..."
cp "$BUILD_DIR/$APP_NAME" "$DIST_DIR/"

# Copy all the libraries that are already bundled by the build system
echo "📚 Copying pre-bundled libraries..."
if [ -d "$BUILD_DIR/lib" ]; then
    echo "   Found $(ls $BUILD_DIR/lib | wc -l) libraries..."
    cp -r "$BUILD_DIR/lib/"* "$DIST_DIR/lib/"
fi

# Copy Qt libraries that are still missing
echo "🎯 Copying Qt libraries..."
Qt_LIBS=$(ldd "$BUILD_DIR/$APP_NAME" | grep "Qt" | grep "/home/gunnar/Qt" | awk '{print $3}')
Qt_COUNT=0
for lib in $Qt_LIBS; do
    if [ -f "$lib" ]; then
        cp "$lib" "$DIST_DIR/lib/"
        Qt_COUNT=$((Qt_COUNT + 1))
    fi
done
echo "   Copied $Qt_COUNT Qt libraries"

# Create launch script
echo "🔧 Creating launch script..."
cat > "$DIST_DIR/launch.sh" << 'EOF'
#!/bin/bash

# RControlStation Launch Script
# Sets up the environment and launches the application

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Set library path to include bundled libraries
if [ -d "$SCRIPT_DIR/lib" ]; then
    export LD_LIBRARY_PATH="$SCRIPT_DIR/lib:$LD_LIBRARY_PATH"
fi

# Launch the application
"$SCRIPT_DIR/RControlStation" "$@"
EOF

chmod +x "$DIST_DIR/launch.sh"

# Create README
echo "📄 Creating README..."
cat > "$DIST_DIR/README.txt" << EOF
RControlStation - Linux Distribution
=====================================

RControlStation is a control station for autonomous vehicles.

INSTALLATION INSTRUCTIONS:
---------------------------
1. Extract this ZIP archive to any location
2. Make the launch script executable:
   chmod +x launch.sh
3. Run the application:
   ./launch.sh

SYSTEM REQUIREMENTS:
--------------------
- Linux x86_64 (64-bit)
- glibc 2.31 or newer
- Basic X11/Wayland environment

TROUBLESHOOTING:
-----------------
If you get library errors, try:
- Installing missing system libraries
- Running: ldconfig
- Checking that your system is 64-bit

COMMAND LINE OPTIONS:
----------------------
Run ./launch.sh --help for available options

LICENSE:
--------
[Include your license information here]
EOF

# Copy icon and create desktop file for user convenience
echo "🎨 Adding desktop integration files..."
mkdir -p "$DIST_DIR/desktop"
cp "Icons/Car-96.png" "$DIST_DIR/desktop/rcontrolstation.png"

cat > "$DIST_DIR/desktop/rcontrolstation.desktop" << EOF
[Desktop Entry]
Name=RControlStation
Exec=$PWD/$DIST_DIR/launch.sh
Icon=$PWD/$DIST_DIR/desktop/rcontrolstation.png
Type=Application
Categories=Utility;Development;
Terminal=false
Comment=Control station for autonomous vehicles
EOF

# Create ZIP archive
echo "📦 Creating ZIP archive..."
zip -r "$ZIP_NAME" "$DIST_DIR"

echo "✅ Success!"
echo ""
echo "📦 Created: $ZIP_NAME"
echo "📁 Size: $(du -h "$ZIP_NAME" | cut -f1)"
echo ""
echo "💡 DISTRIBUTION INSTRUCTIONS:"
echo "   Share the ZIP file with users"
echo "   Users extract and run: ./launch.sh"
echo ""
echo "🎯 USER EXPERIENCE:"
echo "   1. User downloads $ZIP_NAME"
echo "   2. User extracts: unzip $ZIP_NAME"
echo "   3. User runs: cd $DIST_DIR && ./launch.sh"
echo "   4. Application starts with all dependencies included"
