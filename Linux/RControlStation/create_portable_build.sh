#!/bin/bash

# Script to create a portable build of RControlStation with bundled libraries

set -e

echo "Creating portable build of RControlStation..."

# Clean up any previous build
rm -rf build/portable
mkdir -p build/portable
mkdir -p build/portable/libs

# Build the application with release_lin configuration
echo "Building application..."
qmake RControlStation.pro -spec linux-g++ CONFIG+=release_lin
make -j$(nproc) clean
make -j$(nproc)

# Copy the executable
echo "Copying executable..."
cp build/lin/RControlStation build/portable/

# Copy the local libraries
echo "Copying libraries..."
cp local_libs/*.so* build/portable/libs/

# Copy Qt libraries (this would need to be customized based on your Qt installation)
echo "Copying Qt libraries..."
QT_LIB_DIR="$(qmake -query QT_INSTALL_LIBS)"
if [ -d "$QT_LIB_DIR" ]; then
    cp $QT_LIB_DIR/libQt6*.so.* build/portable/libs/
fi

# Copy other dependencies
echo "Copying system dependencies..."
DEPS=$(ldd build/lin/RControlStation | grep -o '/[^ ]*\.so[^ ]*' | grep -v '^/' | grep -v 'linux-vdso' | grep -v '^(')
for dep in $DEPS; do
    if [ -f "$dep" ]; then
        cp "$dep" build/portable/libs/
    fi
done

# Create a launch script
echo "Creating launch script..."
cat > build/portable/launch.sh << 'EOF'
#!/bin/bash

# Set up library path
SCRIPT_DIR=$(dirname "$0")
export LD_LIBRARY_PATH="$SCRIPT_DIR/libs:$LD_LIBRARY_PATH"

# Launch the application
"$SCRIPT_DIR/RControlStation" "$@"
EOF

chmod +x build/portable/launch.sh

# Copy resources
echo "Copying resources..."
cp -r Icons build/portable/
cp -r resources build/portable/

# Create a tarball for distribution
echo "Creating distribution tarball..."
cd build
 tar -czf portable_RControlStation.tar.gz portable/
cd ..

echo "Portable build created successfully!"
echo "You can find it in: build/portable_RControlStation.tar.gz"
echo "To run: extract the tarball and execute ./launch.sh"