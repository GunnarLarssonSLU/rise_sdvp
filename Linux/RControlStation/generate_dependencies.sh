#!/bin/bash

# generate_dependencies.sh
# Automatically generates CPack dependencies for DEB and RPM packages
# by analyzing the built executable with ldd

set -e

echo "🔍 Generating package dependencies..."

# Find the executable
if [ -z "$1" ]; then
    echo "Usage: $0 <path-to-executable>"
    echo "Example: $0 build/cmake_linux_release/RControlStation"
    exit 1
fi

EXECUTABLE="$1"

if [ ! -f "$EXECUTABLE" ]; then
    echo "❌ Error: Executable not found at $EXECUTABLE"
    exit 1
fi

echo "📋 Analyzing: $EXECUTABLE"

# Get all dependencies using ldd
DEPS=$(ldd "$EXECUTABLE" | awk '/=>/ {print $1}' | sort | uniq)

echo "📦 Found dependencies:"
echo "$DEPS"
echo ""

# Generate Debian dependencies
DEB_DEPS=""
for lib in $DEPS; do
    # Skip standard libraries
    if [[ "$lib" =~ ^(libc|libstdc\++|libm|libpthread|libdl|librt|libgcc_s) ]]; then
        continue
    fi
    
    # Convert to Debian package name
    pkg=$(echo "$lib" | sed -e 's/^lib//' -e 's/\.so\.//' -e 's/\.so$//' | tr '[:upper:]' '[:lower:]')
    
    # Handle special cases
    case "$pkg" in
        qt6core|qt6gui|qt6widgets|qt6serialport|qt6network|qt6sql|qt6printsupport|qt6charts)
            pkg="lib${pkg}"
            ;;
        gdal)
            pkg="libgdal30"
            ;;
        geotiff)
            pkg="libgeotiff5"
            ;;
        proj)
            pkg="libproj22"
            ;;
        json-c)
            pkg="libjson-c5"
            ;;
        xerces-c)
            pkg="libxerces-c3.2"
            ;;
        kmlbase|kmldom|kmlengine)
            pkg="lib${pkg}1"
            ;;
        hdf5)
            pkg="libhdf5-103"
            ;;
        netcdf)
            pkg="libnetcdf19"
            ;;
        pq)
            pkg="libpq5"
            ;;
        blosc)
            pkg="libblosc1"
            ;;
        zstd)
            pkg="libzstd1"
            ;;
        lz4)
            pkg="liblz4-1"
            ;;
        webp)
            pkg="libwebp7"
            ;;
        tiff)
            pkg="libtiff6"
            ;;
        geos_c)
            pkg="libgeos-c1v5"
            ;;
        sqlite3)
            pkg="libsqlite3-0"
            ;;
        sdl2)
            pkg="libsdl2-2.0-0"
            ;;
    esac
    
    if [ -n "$pkg" ]; then
        DEB_DEPS="$DEB_DEPS, $pkg"
    fi
done

# Remove leading comma and space
DEB_DEPS=${DEB_DEPS#, }

echo "📋 Debian dependencies:"
echo "$DEB_DEPS"
echo ""

# Generate RPM dependencies
RPM_DEPS=""
for lib in $DEPS; do
    # Skip standard libraries
    if [[ "$lib" =~ ^(libc|libstdc\++|libm|libpthread|libdl|librt|libgcc_s) ]]; then
        continue
    fi
    
    # Convert to RPM package name
    pkg=$(echo "$lib" | sed -e 's/^lib//' -e 's/\.so\.//' -e 's/\.so$//' | tr '[:upper:]' '[:lower:]')
    
    # Handle special cases for RPM
    case "$pkg" in
        qt6*)
            pkg=$(echo "$pkg" | sed -e 's/qt6/qt6-/')
            ;;
        gdal)
            pkg="gdal"
            ;;
        sdl2)
            pkg="SDL2"
            ;;
    esac
    
    if [ -n "$pkg" ]; then
        RPM_DEPS="$RPM_DEPS, $pkg"
    fi
done

# Remove leading comma and space
RPM_DEPS=${RPM_DEPS#, }

echo "📋 RPM dependencies:"
echo "$RPM_DEPS"
echo ""

# Update CMakeLists.txt
CMAKE_FILE="CMakeLists.txt"

if [ ! -f "$CMAKE_FILE" ]; then
    echo "❌ Error: CMakeLists.txt not found"
    exit 1
fi

echo "🔧 Updating CMakeLists.txt..."

# Backup original file
cp "$CMAKE_FILE" "${CMAKE_FILE}.bak"

# Update DEB dependencies
sed -i "/CPACK_DEBIAN_PACKAGE_DEPENDS/c\\    set(CPACK_DEBIAN_PACKAGE_DEPENDS \"${DEB_DEPS}\")" "$CMAKE_FILE"

# Update RPM dependencies  
sed -i "/CPACK_RPM_PACKAGE_REQUIRES/c\\    set(CPACK_RPM_PACKAGE_REQUIRES \"${RPM_DEPS}\")" "$CMAKE_FILE"

echo "✅ CMakeLists.txt updated with new dependencies"
echo ""
echo "💡 Next steps:"
echo "   1. Review the changes in CMakeLists.txt"
echo "   2. Rebuild your project"
echo "   3. Generate new .deb and .rpm packages with cpack"
