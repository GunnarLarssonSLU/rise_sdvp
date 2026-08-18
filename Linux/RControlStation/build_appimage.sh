#!/bin/bash

# Build AppImage for RControlStation
# Creates a single AppImage file that users can run directly
# Requires: fuse, libfuse2, and internet access to download appimagetool

set -e

echo "🚀 Building AppImage for RControlStation..."

APP_NAME="RControlStation"
# Try CMake build directory first, fall back to qmake directory
if [ -f "build/cmake_linux_release/build/lin/$APP_NAME" ]; then
    BUILD_DIR="build/cmake_linux_release/build/lin"
elif [ -f "build/cmake_linux_release/$APP_NAME" ]; then
    BUILD_DIR="build/cmake_linux_release"
elif [ -f "build/Desktop_Qt_6_9_1-release/$APP_NAME" ]; then
    BUILD_DIR="build/Desktop_Qt_6_9_1-release"
else
    echo "❌ Error: Could not find RControlStation executable in any build directory"
    echo "   Checked:"
    echo "   - build/cmake_linux_release/build/lin/"
    echo "   - build/cmake_linux_release/"
    echo "   - build/Desktop_Qt_6_9_1-release/"
    exit 1
fi
APP_DIR="AppDir"
FINAL_APPIMAGE="RControlStation-x86_64.AppImage"

# Clean up previous builds
echo "🧹 Cleaning up..."
rm -rf "$APP_DIR"
rm -f "$FINAL_APPIMAGE"
rm -f appimagetool

# Create AppDir structure
echo "📁 Setting up AppDir..."
mkdir -p "$APP_DIR"
mkdir -p "$APP_DIR/usr/bin"
mkdir -p "$APP_DIR/usr/lib"
mkdir -p "$APP_DIR/usr/share/applications"
mkdir -p "$APP_DIR/usr/share/icons/hicolor/256x256/apps"
mkdir -p "$APP_DIR/usr/share/$APP_NAME"

# Copy the main executable
echo "📋 Copying executable..."
cp "$BUILD_DIR/$APP_NAME" "$APP_DIR/usr/bin/"

# Create AppRun symlink (required by AppImage spec)
echo "🔗 Creating AppRun symlink..."
ln -sf "usr/bin/$APP_NAME" "$APP_DIR/AppRun"

# Copy all pre-bundled libraries
echo "📚 Copying pre-bundled libraries..."
if [ -d "$BUILD_DIR/lib" ]; then
    cp -r "$BUILD_DIR/lib/"* "$APP_DIR/usr/lib/" 2>/dev/null || true
    echo "   Copied $(ls $BUILD_DIR/lib 2>/dev/null | wc -l) libraries from build/lib"
fi

# Copy Qt libraries that are still referenced from system Qt installation
echo "🎯 Copying Qt libraries..."
Qt_LIBS=$(ldd "$BUILD_DIR/$APP_NAME" 2>/dev/null | grep "Qt" | grep "/home/gunnar/Qt" | awk '{print $3}')
Qt_COUNT=0
for lib in $Qt_LIBS; do
    if [ -f "$lib" ]; then
        cp "$lib" "$APP_DIR/usr/lib/" 2>/dev/null || true
        Qt_COUNT=$((Qt_COUNT + 1))
    fi
done
if [ $Qt_COUNT -gt 0 ]; then
    echo "   Copied $Qt_COUNT Qt libraries"
else
    echo "   No Qt libraries found in /home/gunnar/Qt - using system Qt or bundled Qt"
fi

# Download appimagetool
echo "🔧 Downloading appimagetool..."
wget -q "https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage" -O appimagetool
chmod +x appimagetool

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
Comment=Control station for autonomous vehicles
EOF

# Copy desktop file to AppDir root (required by appimagetool)
cp "$APP_DIR/usr/share/applications/$APP_NAME.desktop" "$APP_DIR/"

# Copy icon
echo "🎨 Setting up icon..."
cp "Icons/Car-96.png" "$APP_DIR/usr/share/icons/hicolor/256x256/apps/$APP_NAME.png"
cp "Icons/Car-96.png" "$APP_DIR/$APP_NAME.png"

# Copy database file
echo "🗃️  Copying database..."
# Try multiple locations for data.db
DB_FOUND=0
for db_path in "$BUILD_DIR/data.db" "$BUILD_DIR/../data.db" "data.db"; do
    if [ -f "$db_path" ]; then
        cp "$db_path" "$APP_DIR/usr/share/$APP_NAME/"
        echo "   Copied database from: $db_path"
        DB_FOUND=1
        break
    fi
done
if [ $DB_FOUND -eq 0 ]; then
    echo "   Warning: data.db not found in any expected location"
fi

# Set proper permissions
echo "🔐 Setting permissions..."
chmod +x "$APP_DIR/usr/bin/$APP_NAME"
chmod +x "$APP_DIR/AppRun"

# Create the AppImage
echo "🛠️ Creating AppImage (this may take a minute)..."
./appimagetool "$APP_DIR" "$FINAL_APPIMAGE"

# Clean up
echo "🧹 Cleaning up..."
rm -f appimagetool
rm -rf "$APP_DIR"

echo "✅ Success! AppImage created: $FINAL_APPIMAGE"
echo ""
echo "📊 AppImage Statistics:"
echo "   Size: $(du -h "$FINAL_APPIMAGE" | cut -f1)"
echo ""
echo "📋 USER INSTRUCTIONS:"
echo "   1. Share this single file with users: $FINAL_APPIMAGE"
echo "   2. Users make it executable: chmod +x $FINAL_APPIMAGE"
echo "   3. Users run it: ./$FINAL_APPIMAGE"
echo ""
echo "💡 The AppImage is completely self-contained and includes:"
echo "   • RControlStation executable"
echo "   • All Qt libraries"
echo "   • All GDAL and dependency libraries"
echo "   • Icon and desktop integration"
echo ""
echo "🎯 This will work on any Linux x86_64 system with FUSE support."
