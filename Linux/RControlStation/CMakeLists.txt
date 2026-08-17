cmake_minimum_required(VERSION 3.16)

# Project name and version
project(RControlStation VERSION 1.0.0 LANGUAGES CXX C)

# Set C++ standard to C++17 (as used in the .pro file)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable position independent code for static builds
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Option to use Qt5 instead of Qt6 (for compatibility with existing code)
option(USE_QT5 "Use Qt5 instead of Qt6" OFF)

if(USE_QT5)
    set(QT_VERSION 5)
    message(STATUS "Using Qt5")
else()
    set(QT_VERSION 6)
    message(STATUS "Using Qt6")
endif()

# Find Qt components - handle missing components gracefully
if(QT_VERSION EQUAL 5)
    set(QT_REQUIRED_COMPONENTS Core Gui Widgets PrintSupport Network Sql SerialPort)
    set(QT_OPTIONAL_COMPONENTS Charts OpenglWidgets Gamepad)
else()
    set(QT_REQUIRED_COMPONENTS Core Gui Widgets PrintSupport Network Sql SerialPort Charts)
    set(QT_OPTIONAL_COMPONENTS Core5Compat Quick OpenglWidgets Gamepad)
endif()

# Find required components first
find_package(Qt${QT_VERSION} REQUIRED COMPONENTS ${QT_REQUIRED_COMPONENTS})

# Find optional components
foreach(COMPONENT IN LISTS QT_OPTIONAL_COMPONENTS)
    find_package(Qt${QT_VERSION} COMPONENTS ${COMPONENT} QUIET)
    if(Qt${QT_VERSION}${COMPONENT}_FOUND)
        list(APPEND QT_FOUND_COMPONENTS ${COMPONENT})
        message(STATUS "Found Qt${QT_VERSION} component: ${COMPONENT}")
    else()
        message(WARNING "Qt${QT_VERSION} component not found: ${COMPONENT}")
    endif()
endforeach()

# Combine all found components
set(QT_ALL_COMPONENTS ${QT_REQUIRED_COMPONENTS} ${QT_FOUND_COMPONENTS})

# Add build directory to include path for generated UI headers
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Load Qt macros for automatic code generation
if(QT_VERSION EQUAL 5)
    include(${Qt${QT_VERSION}Core_DIR}/Qt5CoreMacros.cmake)
else()
    include(${Qt${QT_VERSION}Core_DIR}/Qt6CoreMacros.cmake)
endif()

# Add SDL2 for gamepad support (Qt6)
find_package(SDL2 QUIET)
if(SDL2_FOUND)
    message(STATUS "SDL2 found for gamepad support")
else()
    message(WARNING "SDL2 not found - gamepad support may be limited")
endif()

# Find optional dependencies with find_package
find_package(Eigen3 QUIET)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 found at ${Eigen3_INCLUDE_DIRS}")
else()
    message(WARNING "Eigen3 not found")
endif()

# GDAL linking is handled explicitly below to ensure version compatibility
# find_package(GDAL QUIET) - disabled to use explicit paths
# if(GDAL_FOUND)
#     message(STATUS "GDAL found at ${GDAL_INCLUDE_DIRS}")
# else()
#     message(WARNING "GDAL not found")
# endif()

# Add include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add found package include directories if available
if(Eigen3_FOUND)
    include_directories(${Eigen3_INCLUDE_DIRS})
endif()
# GDAL include directories - using explicit paths for version compatibility
include_directories(/usr/local/include /usr/include/gdal)
if(SDL2_FOUND)
    include_directories(${SDL2_INCLUDE_DIRS})
endif()

# Add definitions from the .pro file
add_definitions(
    -DHAS_JOYSTICK_CHECK
    -DHAS_JOYSTICK
)

# Option for OpenGL support
option(HAS_OPENGL "Enable OpenGL support" OFF)
if(HAS_OPENGL)
    add_definitions(-DHAS_OPENGL)
    # Add OpenGL libraries for Windows
    if(WIN32)
        find_package(OpenGL REQUIRED)
        list(APPEND EXTRA_LIBS opengl32)
    endif()
endif()

# Option for Assimp support
option(HAS_ASSIMP "Enable Assimp support" OFF)
if(HAS_ASSIMP)
    add_definitions(-DHAS_ASSIMP)
    find_package(Assimp QUIET)
    if(Assimp_FOUND)
        list(APPEND EXTRA_LIBS ${Assimp_LIBRARIES})
        include_directories(${Assimp_INCLUDE_DIRS})
    endif()
endif()

# Option for simulation scenarios
option(HAS_SIM_SCEN "Enable simulation scenarios" OFF)
if(HAS_SIM_SCEN)
    add_definitions(-DHAS_SIM_SCEN)
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/esmini/EnvironmentSimulator/Libraries/esminiLib
        ${CMAKE_CURRENT_SOURCE_DIR}/esmini/EnvironmentSimulator/Modules/RoadManager
        ${CMAKE_CURRENT_SOURCE_DIR}/esmini/externals/pugixml
        ${CMAKE_CURRENT_SOURCE_DIR}/esmini/EnvironmentSimulator/Modules/CommonMini
    )
    # Note: esminiLib would need to be built separately
endif()

# Collect all source files
set(SOURCES
    main.cpp
    arduinoreader.cpp
    checkboxdelegate.cpp
    database.cpp
    vehicletypedelegate.cpp
    mainwindow.cpp
    maproute.cpp
    maproutecollection.cpp
    qcustomplot.cpp
    packetinterface.cpp
    actionmanager.cpp
    colourutils.cpp
    rangeslider.cpp
    shapefile.cpp
    utility.cpp
    mapwidget.cpp
    carinfo.cpp
    locpoint.cpp
    perspectivepixmap.cpp
    carinterface.cpp
    nmeaserver.cpp
    rtcm3_simple.c
    rtcmclient.cpp
    tcpbroadcast.cpp
    rtcmwidget.cpp
    basestation.cpp
    ping.cpp
    networklogger.cpp
    osmclient.cpp
    osmtile.cpp
    tcpserversimple.cpp
    packet.cpp
    networkinterface.cpp
    moteconfig.cpp
    magcal.cpp
    imuplot.cpp
    nmeawidget.cpp
    confcommonwidget.cpp
    ublox.cpp
    ncom.cpp
    correctionanalysis.cpp
    historylineedit.cpp
    imagewidget.cpp
    tcpclientmulti.cpp
    routemagic.cpp
    routegenerator.cpp
    task_basestation.cpp
    controlmanager.cpp
    actuatormanager.cpp
    sensormanager.cpp
    wireguard.cpp
    versionchecker.cpp
)

# Collect all header files
set(HEADERS
    mainwindow.h
    arduinoreader.h
    checkboxdelegate.h
    database.h
    vehicletypedelegate.h
    actionmanager.h
    colourutils.h
    maproute.h
    maproutecollection.h
    qcustomplot.h
    datatypes.h
    packetinterface.h
    rangeslider.h
    shapefile.h
    utility.h
    mapwidget.h
    carinfo.h
    locpoint.h
    perspectivepixmap.h
    carinterface.h
    nmeaserver.h
    rtcm3_simple.h
    rtcmclient.h
    tcpbroadcast.h
    rtcmwidget.h
    basestation.h
    ping.h
    networklogger.h
    osmclient.h
    osmtile.h
    tcpserversimple.h
    packet.h
    networkinterface.h
    moteconfig.h
    magcal.h
    imuplot.h
    nmeawidget.h
    confcommonwidget.h
    ublox.h
    correctionanalysis.h
    historylineedit.h
    imagewidget.h
    tcpclientmulti.h
    routemagic.h
    routegenerator.h
    attributes_masks.h
    task.h
    task_basestation.h
    controlmanager.h
    actuatormanager.h
    sensormanager.h
    wireguard.h
    simscentree.h
    versionchecker.h
)

# Collect all UI files
set(UI_FILES
    mainwindow.ui
    carinterface.ui
    rtcmwidget.ui
    basestation.ui
    networklogger.ui
    networkinterface.ui
    moteconfig.ui
    magcal.ui
    imuplot.ui
    nmeawidget.ui
    confcommonwidget.ui
    ncom.ui
    correctionanalysis.ui
    gpssim.ui
    wireguard.ui
)

# Collect all resource files
set(RESOURCE_FILES
    resources.qrc
)

# Qt automatic code generation
# Separate headers that should not be processed by MOC
list(REMOVE_ITEM HEADERS qcustomplot.h)
set(NON_MOC_HEADERS qcustomplot.h)

if(QT_VERSION EQUAL 5)
    qt5_wrap_cpp(MOC_SOURCES ${HEADERS})
    qt5_wrap_ui(UI_HEADERS ${UI_FILES})
    qt5_add_resources(RCC_SOURCES ${RESOURCE_FILES})
else()
    qt6_wrap_cpp(MOC_SOURCES ${HEADERS})
    qt6_wrap_ui(UI_HEADERS ${UI_FILES})
    qt6_add_resources(RCC_SOURCES ${RESOURCE_FILES})
endif()

# Add back the non-MOC headers to the sources
list(APPEND SOURCES ${NON_MOC_HEADERS})

# Add all generated sources
list(APPEND SOURCES ${MOC_SOURCES} ${UI_HEADERS} ${RCC_SOURCES})

# Create the executable
add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${HEADERS}
    ${UI_FILES}
    ${RESOURCE_FILES}
)

# Set target properties
target_compile_definitions(${PROJECT_NAME} PRIVATE
    HAS_JOYSTICK_CHECK
    HAS_JOYSTICK
)

if(HAS_OPENGL)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_OPENGL)
endif()

if(HAS_ASSIMP)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_ASSIMP)
endif()

if(HAS_SIM_SCEN)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_SIM_SCEN)
endif()

# Link Qt libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt${QT_VERSION}::Core
    Qt${QT_VERSION}::Gui
    Qt${QT_VERSION}::Widgets
    Qt${QT_VERSION}::PrintSupport
    Qt${QT_VERSION}::Network
    Qt${QT_VERSION}::Sql
    Qt${QT_VERSION}::SerialPort
    /usr/lib/x86_64-linux-gnu/libQt6DBus.so.6.2.4
)

# Link Charts if available (required for Qt6, optional for Qt5)
if(QT_VERSION EQUAL 6)
    target_link_libraries(${PROJECT_NAME} PRIVATE Qt${QT_VERSION}::Charts)
else()
    if(Qt${QT_VERSION}Charts_FOUND)
        target_link_libraries(${PROJECT_NAME} PRIVATE Qt${QT_VERSION}::Charts)
    endif()
endif()

# Link optional Qt components if found
foreach(COMPONENT IN LISTS QT_FOUND_COMPONENTS)
    target_link_libraries(${PROJECT_NAME} PRIVATE Qt${QT_VERSION}::${COMPONENT})
endforeach()

# Link SDL2 if found
if(SDL2_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2)
endif()

# Link OpenGL if enabled and available
if(HAS_OPENGL AND OpenGL_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE OpenGL::GL)
endif()

# Link Assimp if enabled and available
if(HAS_ASSIMP AND Assimp_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${Assimp_LIBRARIES})
endif()

# Find and link external libraries
# These are the libraries that were in the LIBS variable from the .pro file

# Find common system libraries
find_package(ZLIB QUIET)
find_package(PNG QUIET)
find_package(JPEG QUIET)
find_package(CURL QUIET)
find_package(OpenSSL QUIET)
find_package(SQLite3 QUIET)
find_package(Expat QUIET)
find_package(LibLZMA QUIET)
find_package(Blosc QUIET)

# Set up external libraries list
set(EXTERNAL_LIBS)

# Add found libraries
if(ZLIB_FOUND)
    if(TARGET ZLIB::ZLIB)
        list(APPEND EXTERNAL_LIBS ZLIB::ZLIB)
    else()
        list(APPEND EXTERNAL_LIBS ${ZLIB_LIBRARIES})
    endif()
    message(STATUS "Found ZLIB")
else()
    list(APPEND EXTERNAL_LIBS z)
endif()

if(PNG_FOUND)
    if(TARGET PNG::PNG)
        list(APPEND EXTERNAL_LIBS PNG::PNG)
    else()
        list(APPEND EXTERNAL_LIBS ${PNG_LIBRARIES})
    endif()
    message(STATUS "Found PNG")
else()
    list(APPEND EXTERNAL_LIBS png)
endif()

if(JPEG_FOUND)
    if(TARGET JPEG::JPEG)
        list(APPEND EXTERNAL_LIBS JPEG::JPEG)
    else()
        list(APPEND EXTERNAL_LIBS ${JPEG_LIBRARIES})
    endif()
    message(STATUS "Found JPEG")
else()
    list(APPEND EXTERNAL_LIBS jpeg)
endif()

if(CURL_FOUND)
    if(TARGET CURL::libcurl)
        list(APPEND EXTERNAL_LIBS CURL::libcurl)
    else()
        list(APPEND EXTERNAL_LIBS ${CURL_LIBRARIES})
    endif()
    message(STATUS "Found CURL")
else()
    list(APPEND EXTERNAL_LIBS curl)
endif()

if(OpenSSL_FOUND)
    if(TARGET OpenSSL::SSL AND TARGET OpenSSL::Crypto)
        list(APPEND EXTERNAL_LIBS OpenSSL::SSL OpenSSL::Crypto)
    else()
        list(APPEND EXTERNAL_LIBS ${OpenSSL_LIBRARIES})
    endif()
    message(STATUS "Found OpenSSL")
else()
    list(APPEND EXTERNAL_LIBS ssl crypto)
endif()

if(SQLite3_FOUND)
    if(TARGET SQLite::SQLite3)
        list(APPEND EXTERNAL_LIBS SQLite::SQLite3)
    else()
        list(APPEND EXTERNAL_LIBS ${SQLite3_LIBRARIES})
    endif()
    message(STATUS "Found SQLite3")
else()
    list(APPEND EXTERNAL_LIBS sqlite3)
endif()

if(Expat_FOUND)
    # Check if Expat provides imported targets
    if(TARGET Expat::Expat)
        list(APPEND EXTERNAL_LIBS Expat::Expat)
        message(STATUS "Found Expat (imported target)")
    elseif(Expat_LIBRARIES)
        list(APPEND EXTERNAL_LIBS ${Expat_LIBRARIES})
        message(STATUS "Found Expat (libraries: ${Expat_LIBRARIES})")
    else()
        # Fallback to system expat library
        list(APPEND EXTERNAL_LIBS expat)
        message(STATUS "Found Expat (using system library)")
    endif()
else()
    list(APPEND EXTERNAL_LIBS expat)
    message(WARNING "Expat not found, using system library")
endif()

# Add remaining libraries that don't have find_package modules
# These will be linked as system libraries
# Note: Link order matters! openjp2 must come before gdal
list(APPEND EXTERNAL_LIBS
    tiff
    webp
    deflate
    lz4
    zstd
    openjp2
    gdal
    df
    mfhdf
    qhull
    xerces-c
    kmlbase
    kmldom
    kmlengine
    json-c
    geotiff
    gif
    heif
    proj
    geos_c
    netcdf
    hdf5_serial
    hdf5_serial_hl
    xml2
    pq
    blosc
    pcre2-8
    odbc
    odbcinst
    IlmImf
    IlmThread
    Imath
    Half
    Iex
    cfitsio
    ogdi
    freexl
)

# Link external libraries
target_link_libraries(${PROJECT_NAME} PRIVATE ${EXTERNAL_LIBS})

# Handle GDAL and OpenJPEG linking carefully
# Use the exact OpenJPEG library that GDAL was compiled with
# GDAL in /usr/local/lib was compiled with OpenJPEG 2.5.0, system has 2.4.0
# Link order matters! OpenJPEG must be linked before GDAL since GDAL depends on it
message(STATUS "Using explicit GDAL and OpenJPEG linking for version compatibility")
target_link_directories(${PROJECT_NAME} PRIVATE
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu
    ${CMAKE_CURRENT_SOURCE_DIR}/local_libs
)

# Add linker flags to ensure all symbols are linked
# Use -Wl,--no-as-needed to link all symbols
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-as-needed)

# Set output directory based on build type
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(WIN32)
        set_target_properties(${PROJECT_NAME} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build/win
        )
    elseif(UNIX AND NOT APPLE)
        set_target_properties(${PROJECT_NAME} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build/lin
        )
        # Add static linking flags for Linux release builds (optional)
        # Note: This can cause issues with some system libraries
        # target_link_options(${PROJECT_NAME} PRIVATE -static-libstdc++ -static-libgcc)
    endif()
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    if(WIN32)
        set_target_properties(${PROJECT_NAME} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build/win
        )
    elseif(UNIX AND NOT APPLE)
        set_target_properties(${PROJECT_NAME} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build/lin
        )
    endif()
endif()

# Enable automatic MOC, UIC, and RCC processing
set_target_properties(${PROJECT_NAME} PROPERTIES
    AUTOMOC ON
    AUTOUIC ON
    AUTORCC ON
)

# Set C++ flags for permissive mode (as in qmake)
if(QT_VERSION GREATER 5)
    target_compile_options(${PROJECT_NAME} PRIVATE -fpermissive)
endif()

# Install target
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

# Print configuration summary
message(STATUS "Configuration summary:")
message(STATUS "  Project: ${PROJECT_NAME} ${PROJECT_VERSION}")
message(STATUS "  C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "  Qt Version: ${Qt${QT_VERSION}_VERSION}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  HAS_OPENGL: ${HAS_OPENGL}")
message(STATUS "  HAS_ASSIMP: ${HAS_ASSIMP}")
message(STATUS "  HAS_SIM_SCEN: ${HAS_SIM_SCEN}")
message(STATUS "  SDL2 Found: ${SDL2_FOUND}")

# CPack configuration for package generation
set(CPACK_PACKAGE_NAME "RControlStation")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_VENDOR "RISE")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Remote Control Station Application for vehicle control and monitoring")
set(CPACK_PACKAGE_HOMEPAGE_URL "https://www.ri.se")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

# Set package maintainer info
set(CPACK_PACKAGE_CONTACT "support@ri.se")

# Generate packages for multiple platforms
if(WIN32)
    set(CPACK_GENERATOR "NSIS")
    set(CPACK_NSIS_MODIFY_PATH ON)
    set(CPACK_NSIS_MUI_UNIICON "${CMAKE_CURRENT_SOURCE_DIR}/installer/icon.ico")
    set(CPACK_NSIS_MUI_UNIICON_FILE "${CMAKE_CURRENT_SOURCE_DIR}/installer/icon.ico")
    set(CPACK_PACKAGE_INSTALL_DIRECTORY "RControlStation")
    set(CPACK_NSIS_DISPLAY_NAME "RControlStation ${PROJECT_VERSION}")
    set(CPACK_NSIS_HELP_LINK "https://www.ri.se")
    set(CPACK_NSIS_URL_INFO_ABOUT "https://www.ri.se")
    set(CPACK_NSIS_CONTACT "support@ri.se")
    set(CPACK_NSIS_CREATE_ICONS_EXTRA "CreateShortcut,'$DESKTOP\\RControlStation.lnk','$INSTDIR\\bin\\${PROJECT_NAME}.exe'")
    set(CPACK_NSIS_CREATE_UNINSTALLER ON)
    set(CPACK_NSIS_ALLOW_ROOT_DIRECTORY_ON_PAGE ON)
elseif(UNIX AND NOT APPLE)
    set(CPACK_GENERATOR "DEB;RPM")
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "RISE Support <support@ri.se>")
    
    # Comprehensive dependency list for Debian/Ubuntu
    # These are the actual package names that provide the libraries your executable needs
    set(CPACK_DEBIAN_PACKAGE_DEPENDS 
        "libqt6core6 (>= 6.2.0), libqt6gui6 (>= 6.2.0), libqt6widgets6 (>= 6.2.0), libqt6serialport6 (>= 6.2.0), libqt6network6 (>= 6.2.0), libqt6sql6 (>= 6.2.0), libqt6printsupport6 (>= 6.2.0), libqt6charts6 (>= 6.2.0), libqt6dbus6 (>= 6.2.0), libsdl2-2.0-0 (>= 2.0.0), libgdal30 (>= 3.0.0), libgeotiff5, libproj25, libjson-c5, libxerces-c3.2, libkmlbase1, libkmldom1, libkmlengine1, libhdf5-103, libnetcdf19, libpq5, libgeos-c1v5, libfreexl1, libcfitsio-dev, libogdi4.1, libopenjp2-7, libqhull8.0, libexpat1, libsqlite3-0, libblosc1, libzstd1, liblz4-1, libwebp7, libtiff5, libjpeg8, libpng16-16, libgif7, libcurl4, libssl3, zlib1g, libbz2-1.0, liblzma5"
    )
    
    # Comprehensive dependency list for RPM/Fedora
    set(CPACK_RPM_PACKAGE_REQUIRES 
        "qt6-qtbase >= 6.2.0, qt6-qtserialport >= 6.2.0, qt6-qtcharts >= 6.2.0, qt6-qtbase-gui >= 6.2.0, qt6-qtdbus >= 6.2.0, qt6-qtnetwork >= 6.2.0, qt6-qtsql >= 6.2.0, qt6-qtprintsupport >= 6.2.0, qt6-qtwidgets >= 6.2.0, SDL2 >= 2.0.0, gdal >= 3.0.0, geotiff, proj, json-c, xerces-c, libkml, hdf5, netcdf, postgresql, blosc, zstd, lz4, libwebp, libtiff, libjpeg, libpng, libgif, curl, openssl, zlib, bzip2, xz"
    )
    
    message(STATUS "CPack DEB dependencies configured")
    message(STATUS "CPack RPM dependencies configured")
    set(CPACK_DEBIAN_PACKAGE_SECTION "science")
    set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
elseif(APPLE)
    set(CPACK_GENERATOR "DragNDrop")
    set(CPACK_DMG_FORMAT "UDBZ")
    set(CPACK_DMG_VOLUME_NAME "RControlStation-${PROJECT_VERSION}")
    set(CPACK_DMG_BACKGROUND_IMAGE "${CMAKE_CURRENT_SOURCE_DIR}/installer/dmg-background.png")
endif()

# Common CPack settings
set(CPACK_PACKAGE_EXECUTABLES "${PROJECT_NAME}")
set(CPACK_CREATE_DESKTOP_LINKS ON)
set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "RControlStation")

# Include CPack
include(CPack)
