# RControlStation - Qt 6 Setup and AppImage Build Guide

## Fixing Qt Version Priority

### Problem
Your system has both Qt 5 and Qt 6 installed, but `qmake` defaults to Qt 5.
The RControlStation project requires Qt 6.9.1.

### Solution Options

#### Option 1: System-wide Fix (Recommended if you primarily use Qt 6)
```bash
# Add Qt 6 to qtchooser
sudo mkdir -p /etc/xdg/qtchooser
echo "/home/gunnar/Qt/6.9.1/gcc_64/bin" | sudo tee /etc/xdg/qtchooser/qt6.conf

# Set Qt 6 as default
sudo mkdir -p /etc/xdg/qtchooser
echo "/home/gunnar/Qt/6.9.1/gcc_64/bin" | sudo tee /etc/xdg/qtchooser/default.conf

# Update alternatives
sudo update-alternatives --install /usr/bin/qmake qmake /home/gunnar/Qt/6.9.1/gcc_64/bin/qmake 60
sudo update-alternatives --set qmake /home/gunnar/Qt/6.9.1/gcc_64/bin/qmake

# Verify
qmake -version  # Should show Qt 6.9.1
```

#### Option 2: Update PATH (Good for individual user)
Add to your `~/.bashrc` or `~/.profile`:
```bash
echo 'export PATH="/home/gunnar/Qt/6.9.1/gcc_64/bin:$PATH"' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH="/home/gunnar/Qt/6.9.1/gcc_64/lib:$LD_LIBRARY_PATH"' >> ~/.bashrc
source ~/.bashrc
```

#### Option 3: Use Project Wrapper (No system changes needed)
A wrapper script `qmake` has been created in your project directory that always uses Qt 6.

---

## Building RControlStation

### 1. Build with Qt 6
```bash
cd /home/gunnar/code/rise_sdvp/Linux/RControlStation
./build_with_qt6.sh
```

This script:
- Uses Qt 6.9.1 explicitly
- Cleans previous builds
- Creates a release build
- Outputs to `build/Desktop_Qt_6_9_1-release/`

### 2. Create AppImage
```bash
./create_final_appimage.sh
```

This script:
- Uses the existing Qt 6 build
- Bundles all dependencies
- Includes the database file
- Creates `RControlStation-x86_64.AppImage`

---

## Quick Build (One Command)
```bash
cd /home/gunnar/code/rise_sdvp/Linux/RControlStation
./build_with_qt6.sh && ./create_final_appimage.sh
```

---

## Testing the AppImage

### Basic Test
```bash
chmod +x RControlStation-x86_64.AppImage
./RControlStation-x86_64.AppImage --help
```

### With Database
Copy your database to the same directory as the AppImage:
```bash
cp build/Desktop_Qt_6_9_1-release/data.db .
./RControlStation-x86_64.AppImage
```

---

## Database Configuration

The application now looks for the database in these locations:

1. **Current directory** (`data.db`) - for development
2. **AppImage data dir** (`/usr/share/RControlStation/data.db`) - bundled
3. **User data directory** (`~/.local/share/RControlStation/data.db`) - user-specific
4. **Creates new database** if none found

---

## Files Modified

### Code Changes
- `database.cpp` - Enhanced database location detection
- `database.h` - No changes needed

### Build Scripts
- `build_with_qt6.sh` - Builds with Qt 6.9.1
- `create_final_appimage.sh` - Creates AppImage with database
- `qmake` - Wrapper script for Qt 6

### Configuration
- `RControlStation.pro` - Already configured for Qt 6

---

## Troubleshooting

### Qt Version Issues
```bash
# Check which qmake is being used
which qmake
qmake -version

# Force Qt 6
/home/gunnar/Qt/6.9.1/gcc_64/bin/qmake -version
```

### Database Not Found
```bash
# Copy database to AppImage directory
cp build/Desktop_Qt_6_9_1-release/data.db /path/to/AppImage/

# Or run from directory containing data.db
cd /directory/with/data.db
./RControlStation-x86_64.AppImage
```

### Missing Dependencies
```bash
# Check what libraries are needed
ldd build/Desktop_Qt_6_9_1-release/RControlStation

# Install missing system libraries if needed
sudo apt install libssl3 libcurl4 etc...
```

---

## User Instructions

Share the single AppImage file with users:

**Users simply:**
1. Download `RControlStation-x86_64.AppImage`
2. Make executable: `chmod +x RControlStation-x86_64.AppImage`
3. Run: `./RControlStation-x86_64.AppImage`

No installation required! The AppImage contains everything needed.

---

## Notes

- The build process may take several minutes
- AppImage creation may take 1-2 minutes
- Final AppImage size: ~186MB
- Requires FUSE on the target system (standard on most Linux distributions)

---

## Support

For issues with:
- **Qt version**: Use the full path to Qt 6.9.1
- **Database**: Check database locations in the modified code
- **AppImage**: Ensure FUSE is installed on target systems
