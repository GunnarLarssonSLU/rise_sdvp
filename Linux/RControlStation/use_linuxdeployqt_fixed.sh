#!/bin/bash

# Fixed version of linuxdeployqt script with better error handling and timeout management
# This script creates a complete AppImage with all dependencies for RControlStation

echo "🚀 Starting AppImage creation with improved error handling..."

APP_NAME="RControlStation"
BUILD_DIR="build/Desktop_Qt_6_9_1-release"
APP_DIR="AppDir"
FINAL_APPIMAGE="RControlStation-x86_64.AppImage"
TIMEOUT_SECONDS=300  # 5 minute timeout

# Function to clean up on exit
cleanup() {
    echo "🧹 Cleaning up..."
    if [ -d "$APP_DIR" ]; then
        rm -rf "$APP_DIR"
    fi
    if [ -f "appimagetool" ]; then
        rm -f "appimagetool"
    fi
}

# Set trap to clean up on script exit
trap cleanup EXIT

# Check if executable exists
if [ ! -f "$BUILD_DIR/$APP_NAME" ]; then
    echo "❌ Executable not found: $BUILD_DIR/$APP_NAME"
    echo "   Please build the project first with qmake && make"
    exit 1
fi

echo "✅ Found executable: $BUILD_DIR/$APP_NAME"

# Try linuxdeployqt with timeout
echo "🔧 Attempting linuxdeployqt (with ${TIMEOUT_SECONDS}s timeout)..."

# Create AppDir structure
mkdir -p "$APP_DIR/usr/bin"
cp "$BUILD_DIR/$APP_NAME" "$APP_DIR/usr/bin/"

# Run linuxdeployqt with timeout
if timeout ${TIMEOUT_SECONDS} linuxdeployqt "$APP_DIR/usr/bin/$APP_NAME" \
    -appimage \
    -bundle-non-qt-libs \
    -extra-plugins=charts,quick \
    -no-translations \
    -always-overwrite \
    -verbose=1 \
    -qmake=/home/gunnar/Qt/6.9.1/gcc_64/bin/qmake 2>&1 | tee linuxdeployqt.log; then

    echo "✅ linuxdeployqt completed successfully!"
    
    # Check if AppImage was created
    if [ -f "$APP_NAME.AppImage" ]; then
        mv "$APP_NAME.AppImage" "$FINAL_APPIMAGE"
        echo "🎉 Success! Created: $FINAL_APPIMAGE"
        
        # Show AppImage info
        echo ""
        echo "📊 AppImage Information:"
        file "$FINAL_APPIMAGE"
        ls -lh "$FINAL_APPIMAGE"
        
        # Make it executable
        chmod +x "$FINAL_APPIMAGE"
        
        exit 0
    else
        echo "❌ linuxdeployqt failed to create AppImage"
        echo "   Check linuxdeployqt.log for details"
    fi
else
    echo "⏰ linuxdeployqt timed out after ${TIMEOUT_SECONDS} seconds"
    echo "   Trying manual AppImage creation..."
fi

# Manual AppImage creation fallback
echo "🛠️  Starting manual AppImage creation..."

# Download appimagetool if needed
if [ ! -f "appimagetool" ]; then
    echo "📥 Downloading appimagetool..."
    wget -q "https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage" -O appimagetool
    chmod +x appimagetool
fi

# Find and copy desktop file
DESKTOP_FILE=""
if [ -f "rcontrolstation.desktop" ]; then
    DESKTOP_FILE="rcontrolstation.desktop"
elif [ -f "build/Desktop_Qt_6_9_1-release/rcontrolstation.desktop" ]; then
    DESKTOP_FILE="build/Desktop_Qt_6_9_1-release/rcontrolstation.desktop"
fi

if [ -n "$DESKTOP_FILE" ]; then
    echo "📄 Found desktop file: $DESKTOP_FILE"
    mkdir -p "$APP_DIR/usr/share/applications"
    cp "$DESKTOP_FILE" "$APP_DIR/usr/share/applications/"
    
    # Fix desktop file paths
    sed -i "s|^Exec=.*|Exec=$APP_NAME|" "$APP_DIR/usr/share/applications/$(basename "$DESKTOP_FILE")"
    sed -i "s|^Icon=.*|Icon=$APP_NAME|" "$APP_DIR/usr/share/applications/$(basename "$DESKTOP_FILE")"
else
    echo "⚠️  No desktop file found, creating a basic one..."
    mkdir -p "$APP_DIR/usr/share/applications"
    cat > "$APP_DIR/usr/share/applications/$APP_NAME.desktop" << EOF
[Desktop Entry]
Name=RControlStation
Exec=$APP_NAME
Icon=$APP_NAME
Type=Application
Categories=Utility;Science;Engineering;
Terminal=false
EOF
fi

# Copy icon if available
if [ -f "Icons/Car-96.png" ]; then
    cp "Icons/Car-96.png" "$APP_DIR/$APP_NAME.png"
    echo "🖼️  Copied icon"
else
    echo "⚠️  No icon found at Icons/Car-96.png"
fi

# Try to create AppImage
echo "📦 Creating AppImage with appimagetool..."
if ./appimagetool "$APP_DIR" "$FINAL_APPIMAGE" 2>&1 | tee appimagetool.log; then
    echo "🎉 Manual AppImage creation successful!"
    chmod +x "$FINAL_APPIMAGE"
    
    # Show final info
    echo ""
    echo "📊 Final AppImage Information:"
    file "$FINAL_APPIMAGE"
    ls -lh "$FINAL_APPIMAGE"
    
    # Clean up
    rm -f appimagetool
    rm -rf "$APP_DIR"
    
    exit 0
else
    echo "❌ Manual AppImage creation also failed"
    echo "   Check appimagetool.log for details"
    exit 1
fi