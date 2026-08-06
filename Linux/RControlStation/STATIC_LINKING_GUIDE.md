# Static Linking Guide for RControlStation

## Why Static Linking?

Static linking solves dependency issues by embedding libraries directly into your executable:
- ✅ No missing library errors
- ✅ No version conflicts
- ✅ Smaller AppImage size
- ✅ Better compatibility across Linux distributions

## How to Implement Static Linking

### 1. Install Static Versions of Problematic Libraries

```bash
# Install static versions of common problematic libraries
sudo apt install libtiff-dev libgeotiff-dev libnetcdf-dev zlib1g-dev
sudo apt install libssl-dev libcurl4-openssl-dev libsqlite3-dev
```

### 2. Modify the .pro File

Add these lines to your `RControlStation.pro` file:

```qmake
# Enable static linking for problematic libraries
CONFIG += static

# Link statically with specific libraries
LIBS += -ltiff -lgeotiff -lnetcdf -lz -lssl -lcrypto -lsqlite3

# For Qt libraries (if needed)
# QMAKE_LFLAGS += -static-libstdc++ -static-libgcc
```

### 3. Rebuild with Static Linking

```bash
cd /home/gunnar/code/rise_sdvp/Linux/RControlStation

# Clean previous build
rm -rf build/Desktop_Qt_6_9_1-release
mkdir -p build/Desktop_Qt_6_9_1-release
cd build/Desktop_Qt_6_9_1-release

# Build with static linking
/home/gunnar/Qt/6.9.1/gcc_64/bin/qmake ../../RControlStation.pro -config static
make -j$(nproc)

cd ../..
```

### 4. Verify Static Linking

```bash
# Check if libraries are statically linked
ldd build/Desktop_Qt_6_9_1-release/RControlStation | grep -E "(tiff|geotiff|netcdf)" || echo "✅ Libraries are statically linked!"
```

## Alternative: Partial Static Linking

If full static linking causes issues, try linking only problematic libraries statically:

```qmake
# In RControlStation.pro

# Dynamic linking by default
CONFIG -= static
CONFIG += shared

# But link these libraries statically
LIBS += -Wl,-Bstatic -ltiff -lgeotiff -lnetcdf -Wl,-Bdynamic
```

## Handling libnetcdf Version Issues

### Option A: Bundle the Exact Version

```bash
# Copy the exact libnetcdf version you built with
cp /usr/lib/x86_64-linux-gnu/libnetcdf.so.19* build/Desktop_Qt_6_9_1-release/lib/
```

### Option B: Use a Compatibility Layer

```bash
# Install multiple versions
sudo apt install libnetcdf19 libnetcdf-dev

# Or build from source to match your version
wget https://github.com/Unidata/netcdf-c/releases/download/v4.9.2/netcdf-c-4.9.2.tar.gz
tar -xzf netcdf-c-4.9.2.tar.gz
cd netcdf-c-4.9.2
./configure --prefix=/usr/local
make
sudo make install
```

### Option C: Use Conditional Loading

Modify your code to handle version differences:

```cpp
// In your code where you use netcdf
#ifdef NETCDF_VERSION_22
    // Use version 22 API
#else
    // Use version 19 API
#endif
```

## Recommended Approach

1. **First try**: Use `linuxdeployqt -bundle-non-qt-libs` (Option 2 above)
2. **If that fails**: Bundle missing libraries manually (Option 1)
3. **For critical libraries**: Use static linking (Option 3)
4. **Last resort**: Build a compatibility layer

## Checking Your Current Dependencies

```bash
# See what's currently being linked
ldd build/Desktop_Qt_6_9_1-release/RControlStation

# Check for version conflicts
ldd build/Desktop_Qt_6_9_1-release/RControlStation | grep -E "not found|=>"
```

## Final Notes

- Static linking increases binary size but improves compatibility
- Some licenses may restrict static linking (check library licenses)
- Test thoroughly on different Linux distributions
- Consider using Docker for consistent build environments
