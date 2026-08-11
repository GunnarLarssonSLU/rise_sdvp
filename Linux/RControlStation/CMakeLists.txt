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

# Add include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/include
    /usr/include/eigen3
    /usr/include/gdal
    /usr/include/SDL2
)

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
    gpsgen.h
    gpssim.h
    limesdr.h
    wireguard.h
    copterinterface.h
    orientationwidget.h
    simscentree.h
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
    copterinterface.ui
    wireguard.ui
)

# Collect all resource files
set(RESOURCE_FILES
    resources.qrc
)

# Qt automatic code generation
if(QT_VERSION EQUAL 5)
    qt5_wrap_cpp(MOC_SOURCES ${HEADERS})
    qt5_wrap_ui(UI_HEADERS ${UI_FILES})
    qt5_add_resources(RCC_SOURCES ${RESOURCE_FILES})
else()
    qt6_wrap_cpp(MOC_SOURCES ${HEADERS})
    qt6_wrap_ui(UI_HEADERS ${UI_FILES})
    qt6_add_resources(RCC_SOURCES ${RESOURCE_FILES})
endif()

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

# Add all the external libraries from the .pro file
# These are the libraries that were in the LIBS variable
set(EXTERNAL_LIBS
    openjp2
    gdal
    z
    sqlite3
    proj
    geos_c
    zstd
    tiff
    curl
    webp
    jpeg
    deflate
    ssl
    crypto
    expat
    png
    lz4
    json-c
    geotiff
    gif
    netcdf
    hdf5_serial
    hdf5_serial_hl
    mfhdf
    df
    heif
    pq
    blosc
    xml2
    pcre2-8
    xerces-c
    IlmImf
    IlmThread
    Imath
    Half
    Iex
    kmlbase
    kmldom
    kmlengine
    qhull
    cfitsio
    ogdi
    freexl
    odbc
    odbcinst
)

# Find and link external libraries
target_link_libraries(${PROJECT_NAME} PRIVATE ${EXTERNAL_LIBS})

# Add library directories
target_link_directories(${PROJECT_NAME} PRIVATE
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu
    ${CMAKE_CURRENT_SOURCE_DIR}/local_libs
)

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
        # Add static linking flags for Linux release builds
        target_link_options(${PROJECT_NAME} PRIVATE -static-libstdc++ -static-libgcc)
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
