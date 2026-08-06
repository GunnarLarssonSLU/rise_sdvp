#!/bin/bash

# Proper AppImage creation script for RControlStation
# This script creates a single AppImage file that users can run directly

set -e  # Exit on error

# Configuration
APP_NAME="RControlStation"
BUILD_DIR="build/Desktop_Qt_6_9_1-release"
APP_DIR="AppDir"
APP_IMAGE_NAME="RControlStation-x86_64.AppImage"

# Clean up previous builds
echo "🧹 Cleaning up previous builds..."
rm -rf "$APP_DIR"
rm -f "$APP_IMAGE_NAME"
rm -f appimagetool

# Create AppDir structure
echo "📁 Creating AppDir structure..."
mkdir -p "$APP_DIR/usr/bin"
mkdir -p "$APP_DIR/usr/lib"
mkdir -p "$APP_DIR/usr/share/applications"
mkdir -p "$APP_DIR/usr/share/icons/hicolor/256x256/apps"

# Copy the executable
echo "📋 Copying executable..."
cp "$BUILD_DIR/$APP_NAME" "$APP_DIR/usr/bin/"

# Copy existing libraries
echo "📚 Copying existing libraries..."
if [ -d "$BUILD_DIR/lib" ]; then
    cp -r "$BUILD_DIR/lib/"* "$APP_DIR/usr/lib/"
fi

# Download appimagetool if not available
echo "🔧 Downloading appimagetool..."
wget -c "https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage" -O appimagetool
chmod +x appimagetool

# Use linuxdeployqt to bundle Qt libraries
echo "🎯 Bundling Qt libraries with linuxdeployqt..."
linuxdeployqt "$APP_DIR/usr/bin/$APP_NAME" -appimage -no-translations -always-overwrite -verbose=0

# Create desktop file
echo "📄 Creating desktop file..."
cat > "$APP_DIR/usr/share/applications/$APP_NAME.desktop" << EOF
[Desktop Entry]
Name=RControlStation
Exec=$APP_NAME
Icon=$APP_NAME
Type=Application
Categories=Utility;Development;
Terminal=false
Comment=RControlStation - Control station for autonomous vehicles
EOF

# Copy icon
echo "🎨 Setting up icon..."
cp "Icons/Car-96.png" "$APP_DIR/$APP_NAME.png"
cp "Icons/Car-96.png" "$APP_DIR/usr/share/icons/hicolor/256x256/apps/$APP_NAME.png"

# Set proper permissions
echo "🔐 Setting permissions..."
chmod +x "$APP_DIR/usr/bin/$APP_NAME"

# Create AppImage
echo "🛠️ Creating AppImage..."
./appimagetool "$APP_DIR" "$APP_IMAGE_NAME"

# Clean up
echo "🧹 Cleaning up..."
rm appimagetool
rm -rf "$APP_DIR"

echo "✅ Done!"
echo "📦 Created AppImage: $APP_IMAGE_NAME"
echo "💡 Users can run it with: chmod +x $APP_IMAGE_NAME && ./$APP_IMAGE_NAME"
