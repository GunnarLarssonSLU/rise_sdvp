#!/bin/bash

# Create Final AppImage for RControlStation with Database Support
# Uses existing Qt 6 build and adds proper database handling

set -e

echo "🚀 Creating Final AppImage with Database Support..."

APP_NAME="RControlStation"
BUILD_DIR="build/Desktop_Qt_6_9_1-release"
APP_DIR="AppDir"
FINAL_APPIMAGE="RControlStation-x86_64.AppImage"

# Verify we have a Qt 6 build
echo "🔍 Checking build..."
if [ ! -f "$BUILD_DIR/$APP_NAME" ]; then
    echo "❌ Error: Qt 6 build not found at $BUILD_DIR/$APP_NAME"
    echo "Please build with Qt 6 first using: /home/gunnar/Qt/6.9.1/gcc_64/bin/qmake && make"
    exit 1
fi

# Check if it's actually using Qt 6
Qt6_CHECK=$(ldd "$BUILD_DIR/$APP_NAME" | grep -c "libQt6")
if [ "$Qt6_CHECK" -eq 0 ]; then
    echo "❌ Error: Build is not using Qt 6 libraries"
    exit 1
fi

echo "✅ Found valid Qt 6 build"

# Clean up
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

# Create AppRun symlink
echo "🔗 Creating AppRun symlink..."
ln -sf "usr/bin/$APP_NAME" "$APP_DIR/AppRun"

# Copy all pre-bundled libraries
echo "📚 Copying pre-bundled libraries..."
if [ -d "$BUILD_DIR/lib" ]; then
    cp -r "$BUILD_DIR/lib/"* "$APP_DIR/usr/lib/"
    echo "   Copied $(ls $BUILD_DIR/lib | wc -l) libraries"
fi

# Copy Qt libraries that are still referenced from system Qt installation
echo "🎯 Copying additional Qt libraries..."
Qt_LIBS=$(ldd "$BUILD_DIR/$APP_NAME" | grep "Qt" | grep "/home/gunnar/Qt" | awk '{print $3}')
Qt_COUNT=0
for lib in $Qt_LIBS; do
    if [ -f "$lib" ]; then
        cp "$lib" "$APP_DIR/usr/lib/"
        Qt_COUNT=$((Qt_COUNT + 1))
    fi
done
echo "   Copied $Qt_COUNT additional Qt libraries"

# Copy database file
echo "🗃️  Copying database..."
if [ -f "$BUILD_DIR/data.db" ]; then
    cp "$BUILD_DIR/data.db" "$APP_DIR/usr/share/$APP_NAME/"
    echo "   ✅ Copied database to AppDir"
elif [ -f "build/data.db" ]; then
    cp "build/data.db" "$APP_DIR/usr/share/$APP_NAME/"
    echo "   ✅ Copied database from build directory"
else
    echo "   ⚠️  Warning: data.db not found - AppImage will create new database on first run"
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
if [ -f "Icons/Car-96.png" ]; then
    cp "Icons/Car-96.png" "$APP_DIR/usr/share/icons/hicolor/256x256/apps/$APP_NAME.png"
    cp "Icons/Car-96.png" "$APP_DIR/$APP_NAME.png"
else
    echo "   ⚠️  Warning: Icon not found"
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
echo "💡 DATABASE HANDLING:"
echo "   • AppImage includes bundled database (if found)"
echo "   • Modified code looks in multiple locations for database"
echo "   • Creates new database if none found in user's data directory"
echo ""
echo "🎯 This will work on any Linux x86_64 system with FUSE support."
