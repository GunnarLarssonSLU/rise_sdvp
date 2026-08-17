# Deployment Guide for RControlStation

This document describes how to create robust installation packages for distribution to other users.

## Overview

RControlStation can be deployed in several ways, each with different trade-offs between convenience and system integration.

## Recommended Deployment Methods

### 1. AppImage (Most Portable - Recommended for General Distribution)

**Best for:** Users who want a single file that works on most Linux systems without installation.

**Pros:**
- Single self-contained file
- Works on most Linux distributions
- No installation required
- Bundles all dependencies

**Cons:**
- Larger file size
- Slightly slower startup

**How to build:**
```bash
chmod +x build_appimage.sh
./build_appimage.sh
```

**Output:** `RControlStation-x86_64.AppImage`

**User instructions:**
```bash
chmod +x RControlStation-x86_64.AppImage
./RControlStation-x86_64.AppImage
```

---

### 2. .deb Package (For Debian/Ubuntu Systems)

**Best for:** Users on Debian, Ubuntu, or related distributions who want proper system integration.

**Pros:**
- Proper system integration (menu entries, icons)
- Automatic dependency resolution via apt
- Standard Linux packaging

**Cons:**
- Only works on Debian-based systems
- Requires dependencies to be installed

**How to build:**
```bash
# Clean build
rm -rf build/cmake_linux_release
mkdir -p build/cmake_linux_release
cd build/cmake_linux_release

# Configure and build
cmake --preset=linux-release
cmake --build . --config Release

# Generate .deb package
cpack -G DEB
```

**Output:** `RControlStation-1.0.0-Linux.deb`

**User instructions:**
```bash
sudo apt install ./RControlStation-1.0.0-Linux.deb
# If there are missing dependencies:
sudo apt-get install -f
```

---

### 3. .rpm Package (For Fedora/RHEL Systems)

**Best for:** Users on Fedora, RHEL, CentOS, or related distributions.

**How to build:**
```bash
# First install rpmbuild if needed
sudo apt install rpm

# Then build as above but with:
cpack -G RPM
```

**Output:** `RControlStation-1.0.0-Linux.rpm`

**User instructions:**
```bash
sudo rpm -i RControlStation-1.0.0-Linux.rpm
# If there are missing dependencies:
sudo dnf install <missing-package>
```

---

### 4. Portable ZIP Archive

**Best for:** Users who want to extract and run without installation.

**How to build:**
```bash
chmod +x create_distributable.sh
./create_distributable.sh
```

**Output:** `RControlStation-Linux-x86_64.zip`

**User instructions:**
```bash
unzip RControlStation-Linux-x86_64.zip
cd RControlStation-Distributable
chmod +x launch.sh
./launch.sh
```

---

## Dependency Management

The project uses a **comprehensive list of dependencies** to ensure it works across different systems. The dependencies are automatically included in the package metadata.

### Manual Dependency Update

If you need to update dependencies manually:

1. Build your project
2. Run the dependency analysis script:
   ```bash
   chmod +x generate_dependencies.sh
   ./generate_dependencies.sh build/cmake_linux_release/RControlStation
   ```
3. Review the changes to CMakeLists.txt
4. Rebuild your packages

### Common Missing Dependencies

If users report missing libraries, the most common ones are:

```bash
# For Qt6
sudo apt install libqt6core6 libqt6gui6 libqt6widgets6 libqt6charts6

# For GDAL and related
sudo apt install libgdal30 libgeotiff5 libproj22

# For other dependencies
sudo apt install libsdl2-2.0-0 libjson-c5 libxerces-c3.2
```

---

## Troubleshooting

### "error while loading shared libraries"

This means a required library is missing. To diagnose:

```bash
ldd /usr/bin/RControlStation | grep "not found"
```

Then install the missing package. The package name is usually the library name with some modifications:
- `libQt6Core.so.6` → `libqt6core6`
- `libgdal.so.30` → `libgdal30`
- `libproj.so.22` → `libproj22`

### "Download is performed unsandboxed as root"

This happens when trying to install a .deb file from a user directory. Solutions:

```bash
# Use absolute path
sudo apt install /full/path/to/package.deb

# Or copy to /tmp
cp package.deb /tmp/
sudo apt install /tmp/package.deb

# Or use dpkg
sudo dpkg -i package.deb
sudo apt-get install -f
```

### Desktop icon not appearing

After installing the .deb package:
```bash
# Update desktop database
update-desktop-database

# Update icon cache
gtk-update-icon-cache
```

---

## Best Practices for Distribution

1. **Test on a clean system**: Always test your packages on a system that doesn't have the development environment
2. **Use virtual machines**: Create a clean Ubuntu VM to test .deb packages
3. **Document dependencies**: Include a README with installation instructions
4. **Provide multiple formats**: Offer both AppImage and .deb/.rpm for maximum compatibility
5. **Version your packages**: Include version numbers in filenames

---

## File Locations After Installation

| Method | Executable | Desktop File | Icon |
|--------|------------|--------------|------|
| .deb/.rpm | `/usr/bin/RControlStation` | `/usr/share/applications/RControlStation.desktop` | `/usr/share/icons/hicolor/256x256/apps/rcontrolstation.png` |
| AppImage | Inside AppImage | Inside AppImage | Inside AppImage |
| ZIP | In extraction directory | In `desktop/` subdirectory | In `desktop/` subdirectory |

---

## Continuous Integration

For automated builds, consider setting up:

1. **GitHub Actions**: For automated testing and package building
2. **Docker containers**: For consistent build environments
3. **Multiple distributions**: Build on Ubuntu, Fedora, etc. to ensure compatibility

Example GitHub Actions workflow would:
- Build on Ubuntu 22.04
- Generate .deb, AppImage, and ZIP packages
- Upload as release artifacts
