#!/bin/bash

# Use linuxdeployqt to properly bundle ALL dependencies
# This is the recommended approach for Qt applications

set -e

echo "🚀 Using linuxdeployqt for complete dependency bundling..."

APP_NAME="RControlStation"
BUILD_DIR="build/Desktop_Qt_6_9_1-release"
APP_DIR="AppDir"
FINAL_APPIMAGE="RControlStation-x86_64.AppImage"

# Clean up
echo "🧹 Cleaning up..."
rm -rf "$APP_DIR"
rm -f "$FINAL_APPIMAGE"

# Create basic AppDir structure
echo "📁 Creating AppDir..."
mkdir -p "$APP_DIR/usr/bin"

# Copy executable
echo "📋 Copying executable..."
cp "$BUILD_DIR/$APP_NAME" "$APP_DIR/usr/bin/"

# Use linuxdeployqt to bundle ALL dependencies
echo "🔧 Running linuxdeployqt (this may take several minutes)..."
linuxdeployqt "$APP_DIR/usr/bin/$APP_NAME" \
    -appimage \
    -bundle-non-qt-libs \
    -extra-plugins=charts,quick \
    -no-translations \
    -always-overwrite \
    -verbose=1 \
    -qmake=/home/gunnar/Qt/6.9.1/gcc_64/bin/qmake

# Check if AppImage was created
echo "🔍 Checking results..."
if [ -f "$APP_NAME.AppImage" ]; then
    echo "✅ linuxdeployqt created: $APP_NAME.AppImage"
    mv "$APP_NAME.AppImage" "$FINAL_APPIMAGE"
else
    echo "❌ linuxdeployqt failed to create AppImage"
    echo "   Trying manual appimagetool approach..."
    
    # Download appimagetool
    wget -q "https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage" -O appimagetool
    chmod +x appimagetool
    
    # Create desktop file
    mkdir -p "$APP_DIR/usr/share/applications"
    
    # Try to find existing desktop file first
    if [ -f "rcontrolstation.desktop" ]; then
        cp "rcontrolstation.desktop" "$APP_DIR/usr/share/applications/"
        sed -i "s|^Exec=.*|Exec=$APP_NAME|" "$APP_DIR/usr/share/applications/rcontrolstation.desktop"
        sed -i "s|^Icon=.*|Icon=$APP_NAME|" "$APP_DIR/usr/share/applications/rcontrolstation.desktop"
    elif [ -f "build/Desktop_Qt_6_9_1-release/rcontrolstation.desktop" ]; then
        cp "build/Desktop_Qt_6_9_1-release/rcontrolstation.desktop" "$APP_DIR/usr/share/applications/"
        sed -i "s|^Exec=.*|Exec=$APP_NAME|" "$APP_DIR/usr/share/applications/rcontrolstation.desktop"
        sed -i "s|^Icon=.*|Icon=$APP_NAME|" "$APP_DIR/usr/share/applications/rcontrolstation.desktop"
    else
        cat > "$APP_DIR/usr/share/applications/$APP_NAME.desktop" << EOF
[Desktop Entry]
Name=RControlStation
Exec=$APP_NAME
Icon=$APP_NAME
Type=Application
Categories=Utility;
EOF
    fi
    
    # Copy icon
    if [ -f "Icons/Car-96.png" ]; then
        cp "Icons/Car-96.png" "$APP_DIR/$APP_NAME.png"
    fi
    
    # Create AppImage
    ./appimagetool "$APP_DIR" "$FINAL_APPIMAGE"
    rm appimagetool
fi

# Clean up
echo "🧹 Cleaning up..."
rm -rf "$APP_DIR"

echo "✅ Success! Created: $FINAL_APPIMAGE"
echo ""
echo "📋 This AppImage should include ALL dependencies including:"
echo "   • libtiff, libgeotiff"
echo "   • libnetcdf and other scientific libraries"
echo "   • All Qt libraries"
echo "   • System libraries that were missing"
