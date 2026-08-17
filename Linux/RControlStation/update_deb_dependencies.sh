#!/bin/bash

# update_deb_dependencies.sh
# Automatically updates CMakeLists.txt with correct Debian package names
# for all dependencies found by ldd

set -e

echo "🔍 Updating Debian dependencies for RControlStation..."

# Check if we have an executable
if [ ! -f "$1" ]; then
    echo "Usage: $0 <path-to-executable>"
    echo "Example: $0 build/cmake_linux_release/RControlStation"
    exit 1
fi

EXECUTABLE="$1"
TMP_DIR=$(mktemp -d)

# Get all shared library dependencies
echo "📋 Analyzing dependencies of: $EXECUTABLE"
LDD_OUTPUT=$(ldd "$EXECUTABLE" 2>/dev/null | awk '/=>/ {print $1}' | sort | uniq)

if [ -z "$LDD_OUTPUT" ]; then
    echo "❌ Error: Could not get dependencies from $EXECUTABLE"
    exit 1
fi

echo "Found libraries:"
echo "$LDD_OUTPUT"
echo ""

# Create a list of package names
DEB_DEPENDS=()

for LIB in $LDD_OUTPUT; do
    # Skip standard C/C++ libraries
    if [[ "$LIB" =~ ^(libc|libstdc\++|libm|libpthread|libdl|librt|libgcc_s|libc\.so|ld-linux) ]]; then
        continue
    fi
    
    # Try to find the package that provides this library
    PKG=$(apt-file search "$LIB" 2>/dev/null | grep -v "^$" | head -1 | awk '{print $1}')
    
    if [ -z "$PKG" ]; then
        # Try without the .so version
        BASE_LIB=$(echo "$LIB" | sed 's/\.so[^ ]*//')
        PKG=$(apt-file search "$BASE_LIB" 2>/dev/null | grep -v "^$" | head -1 | awk '{print $1}')
    fi
    
    if [ -n "$PKG" ]; then
        DEB_DEPENDS+=("$PKG")
        echo "✓ $LIB → $PKG"
    else
        echo "⚠ $LIB → [package not found, will need manual addition]"
        # Add a placeholder
        DEB_DEPENDS+=("# MISSING: $LIB")
    fi
done

echo ""
echo "📦 Generated Debian dependencies:"
printf '    "%s",\n' "${DEB_DEPENDS[@]}"

# Format as a CMake list
DEB_DEPENDS_STR=$(IFS=,; echo "${DEB_DEPENDS[*]}")
DEB_DEPENDS_STR=${DEB_DEPENDS_STR// /}

# Update CMakeLists.txt
CMAKE_FILE="CMakeLists.txt"

if [ ! -f "$CMAKE_FILE" ]; then
    echo "❌ Error: CMakeLists.txt not found"
    exit 1
fi

echo ""
echo "🔧 Updating CMakeLists.txt..."

# Backup
cp "$CMAKE_FILE" "${CMAKE_FILE}.bak"

# Update the DEB dependencies line
sed -i "/CPACK_DEBIAN_PACKAGE_DEPENDS/c\\    set(CPACK_DEBIAN_PACKAGE_DEPENDS ${DEB_DEPENDS_STR})" "$CMAKE_FILE"

echo "✅ CMakeLists.txt updated"
echo ""
echo "💡 Next steps:"
echo "   1. Review the changes in CMakeLists.txt"
echo "   2. Fix any '# MISSING: ...' entries manually"
echo "   3. Rebuild your .deb package with cpack -G DEB"

# Cleanup
rm -rf "$TMP_DIR"
