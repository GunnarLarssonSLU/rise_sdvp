#!/bin/bash

# Script to create AppImage for RControlStation
# This script should be run from the RControlStation directory

set -e  # Exit on error

# Configuration
APP_NAME="RControlStation"
BUILD_DIR="build/Desktop_Qt_6_9_1-release"
APP_DIR="AppDir"
APP_IMAGE_NAME="RControlStation.AppImage"

# Clean up previous builds
echo "Cleaning up previous builds..."
rm -rf "$APP_DIR"
rm -f "$APP_IMAGE_NAME"

# Create AppDir structure
echo "Creating AppDir structure..."
mkdir -p "$APP_DIR/usr/bin"
mkdir -p "$APP_DIR/usr/lib"
mkdir -p "$APP_DIR/usr/share/applications"
mkdir -p "$APP_DIR/usr/share/icons/hicolor/256x256/apps"

# Copy the executable
echo "Copying executable..."
cp "$BUILD_DIR/$APP_NAME" "$APP_DIR/usr/bin/"

# Copy existing libraries
echo "Copying existing libraries..."
if [ -d "$BUILD_DIR/lib" ]; then
    cp -r "$BUILD_DIR/lib"/* "$APP_DIR/usr/lib/"
fi

# Run linuxdeployqt to bundle Qt libraries
echo "Bundling Qt libraries with linuxdeployqt..."
linuxdeployqt "$APP_DIR/usr/bin/$APP_NAME" -appimage -no-translations -always-overwrite

# Create desktop file
echo "Creating desktop file..."
cat > "$APP_DIR/usr/share/applications/$APP_NAME.desktop" << EOF
[Desktop Entry]
Name=RControlStation
Exec=$APP_NAME
Icon=$APP_NAME
Type=Application
Categories=Utility;
Terminal=false
EOF

# Copy icon (use Car-96.png as application icon)
if [ -f "Icons/Car-96.png" ]; then
    echo "Copying icon..."
    cp "Icons/Car-96.png" "$APP_DIR/usr/share/icons/hicolor/256x256/apps/$APP_NAME.png"
    # Also copy to 96x96 for completeness
    mkdir -p "$APP_DIR/usr/share/icons/hicolor/96x96/apps"
    cp "Icons/Car-96.png" "$APP_DIR/usr/share/icons/hicolor/96x96/apps/$APP_NAME.png"
fi

# Set proper permissions
echo "Setting permissions..."
chmod +x "$APP_DIR/usr/bin/$APP_NAME"

# Create AppImage
echo "Creating AppImage..."
# Check if appimagetool is available
if command -v appimagetool &> /dev/null; then
    # Use appimagetool if available
    wget -c "https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage" -O appimagetool
    chmod +x appimagetool
    ./appimagetool "$APP_DIR" "$APP_IMAGE_NAME"
    rm appimagetool
else
    # Fallback: create a simple tarball
    echo "appimagetool not found, creating tarball instead..."
    tar -czvf "${APP_NAME}.tar.gz" "$APP_DIR"
    echo "Created ${APP_NAME}.tar.gz instead of AppImage"
fi

echo "Done!"
echo "The AppImage should be available as: $APP_IMAGE_NAME"
