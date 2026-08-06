#!/bin/bash

# Create a proper AppImage for RControlStation
# This script creates a single AppImage file that users can run directly

set -e

echo "🚀 Creating AppImage for RControlStation..."

APP_NAME="RControlStation"
BUILD_DIR="build/Desktop_Qt_6_9_1-release"
APP_DIR="AppDir"
FINAL_APPIMAGE="RControlStation-x86_64.AppImage"

# Clean up
echo "🧹 Cleaning up..."
rm -rf "$APP_DIR"
rm -f "$FINAL_APPIMAGE"
rm -f appimagetool

# Create AppDir structure
echo "📁 Creating AppDir structure..."
mkdir -p "$APP_DIR"
mkdir -p "$APP_DIR/usr/bin"
mkdir -p "$APP_DIR/usr/lib"

# Copy executable
echo "📋 Copying executable..."
cp "$BUILD_DIR/$APP_NAME" "$APP_DIR/usr/bin/"

# Copy all existing libraries
echo "📚 Copying pre-bundled libraries..."
if [ -d "$BUILD_DIR/lib" ]; then
    cp -r "$BUILD_DIR/lib/"* "$APP_DIR/usr/lib/"
fi

# Copy Qt libraries manually
echo "🎯 Copying Qt libraries..."
Qt_LIBS=$(ldd "$BUILD_DIR/$APP_NAME" | grep "Qt" | grep "/home/gunnar/Qt" | awk '{print $3}')
Qt_COUNT=0
for lib in $Qt_LIBS; do
    if [ -f "$lib" ]; then
        cp "$lib" "$APP_DIR/usr/lib/"
        Qt_COUNT=$((Qt_COUNT + 1))
    fi
done
echo "   Copied $Qt_COUNT Qt libraries"

# Download appimagetool
echo "🔧 Downloading appimagetool..."
wget -q "https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage" -O appimagetool
chmod +x appimagetool

# Create desktop file in the right location
echo "📄 Creating desktop file..."
cat > "$APP_DIR/$APP_NAME.desktop" << EOF
[Desktop Entry]
Name=RControlStation
Exec=$APP_NAME
Icon=$APP_NAME
Type=Application
Categories=Utility;Development;
Terminal=false
Comment=Control station for autonomous vehicles
EOF

# Copy icon to AppDir root
echo "🎨 Setting up icon..."
cp "Icons/Car-96.png" "$APP_DIR/$APP_NAME.png"

# Set permissions
echo "🔐 Setting permissions..."
chmod +x "$APP_DIR/usr/bin/$APP_NAME"

# Create AppImage
echo "🛠️ Creating AppImage..."
./appimagetool "$APP_DIR" "$FINAL_APPIMAGE"

# Clean up
echo "🧹 Cleaning up..."
rm -f appimagetool
rm -rf "$APP_DIR"

echo "✅ Success! AppImage created: $FINAL_APPIMAGE"
echo ""
echo "📋 USER INSTRUCTIONS:"
echo "   1. Download: $FINAL_APPIMAGE"
echo "   2. Make executable: chmod +x $FINAL_APPIMAGE"
echo "   3. Run: ./$FINAL_APPIMAGE"
echo ""
echo "💡 The AppImage contains everything needed to run RControlStation"
echo "   on any compatible Linux system (x86_64)."
