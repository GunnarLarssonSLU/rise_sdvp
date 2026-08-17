# GenerateDependencies.cmake
# Automatically detects runtime dependencies using ldd and generates
# CPack dependency lists for DEB and RPM packages

# This script should be included AFTER the executable target is created

if(NOT UNIX OR APPLE)
    # Only works on Linux
    return()
endif()

# Find the executable path
get_target_property(EXECUTABLE_PATH ${PROJECT_NAME} RUNTIME_OUTPUT_DIRECTORY)
if(NOT EXECUTABLE_PATH)
    set(EXECUTABLE_PATH ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif()

set(EXECUTABLE_FILE "${EXECUTABLE_PATH}/${PROJECT_NAME}")

# Only generate dependencies if the executable exists
if(EXISTS "${EXECUTABLE_FILE}")
    message(STATUS "Generating package dependencies from: ${EXECUTABLE_FILE}")
    
    # Run ldd to get all dependencies
    execute_process(
        COMMAND ldd "${EXECUTABLE_FILE}"
        OUTPUT_VARIABLE LDD_OUTPUT
        ERROR_VARIABLE LDD_ERROR
        RESULT_VARIABLE LDD_RESULT
    )
    
    if(LDD_RESULT EQUAL 0)
        # Parse ldd output to extract library names
        string(REPLACE "\n" ";" LDD_LINES "${LDD_OUTPUT}")
        set(DEPENDENCIES)
        
        foreach(LINE ${LDD_LINES})
            # Extract library name from lines like: libQt6Core.so.6 => /usr/lib/x86_64-linux-gnu/libQt6Core.so.6
            string(REGEX MATCH "([a-zA-Z0-9_\-\.]+\.so[^ ]*)" LIB_NAME "${LINE}")
            if(LIB_NAME)
                list(APPEND DEPENDENCIES "${CMAKE_MATCH_1}")
            endif()
        endforeach()
        
        # Remove duplicates
        list(REMOVE_DUPLICATES DEPENDENCIES)
        
        # Convert library names to Debian package names
        set(DEB_DEPENDS)
        foreach(LIB ${DEPENDENCIES})
            # Skip standard libraries (libc, libstdc++, etc.)
            if(LIB MATCHES "^(libc|libstdc\+\+|libm|libpthread|libdl|librt|libgcc_s)")
                continue()
            endif()
            
            # Convert soname to package name
            # libQt6Core.so.6 -> libqt6core6
            string(REGEX REPLACE "^lib([a-zA-Z0-9_]+)\.so\.?[0-9]*" "lib\\L\\1" PKG_NAME "${LIB}")
            string(TOLOWER "${PKG_NAME}" PKG_NAME)
            
            # Handle special cases
            if(PKG_NAME MATCHES "libqt")
                # Qt libraries: libqt6core6, libqt6gui6, etc.
                string(REGEX REPLACE "libqt([0-9]+)([a-zA-Z]+)" "libqt\\1\\L\\2" PKG_NAME "${PKG_NAME}")
            elseif(PKG_NAME MATCHES "libgdal")
                set(PKG_NAME "libgdal30")
            elseif(PKG_NAME MATCHES "libgeotiff")
                set(PKG_NAME "libgeotiff5")
            elseif(PKG_NAME MATCHES "libproj")
                set(PKG_NAME "libproj22")
            elseif(PKG_NAME MATCHES "libjson-c")
                set(PKG_NAME "libjson-c5")
            elseif(PKG_NAME MATCHES "libxerces-c")
                set(PKG_NAME "libxerces-c3.2")
            elseif(PKG_NAME MATCHES "libkml")
                string(REGEX REPLACE "libkm([l a-zA-Z]+)" "libkml\\L\\1" PKG_NAME "${PKG_NAME}")
            elseif(PKG_NAME MATCHES "libhdf5")
                set(PKG_NAME "libhdf5-103")
            elseif(PKG_NAME MATCHES "libnetcdf")
                set(PKG_NAME "libnetcdf19")
            elseif(PKG_NAME MATCHES "libpq")
                set(PKG_NAME "libpq5")
            elseif(PKG_NAME MATCHES "libblosc")
                set(PKG_NAME "libblosc1")
            elseif(PKG_NAME MATCHES "libzstd")
                set(PKG_NAME "libzstd1")
            elseif(PKG_NAME MATCHES "liblz4")
                set(PKG_NAME "liblz4-1")
            elseif(PKG_NAME MATCHES "libwebp")
                set(PKG_NAME "libwebp7")
            elseif(PKG_NAME MATCHES "libtiff")
                set(PKG_NAME "libtiff6")
            elseif(PKG_NAME MATCHES "libgeos")
                set(PKG_NAME "libgeos-c1v5")
            elseif(PKG_NAME MATCHES "libsqlite3")
                set(PKG_NAME "libsqlite3-0")
            elseif(PKG_NAME MATCHES "libsdl2")
                set(PKG_NAME "libsdl2-2.0-0")
            endif()
            
            if(PKG_NAME)
                list(APPEND DEB_DEPENDS "${PKG_NAME}")
            endif()
        endforeach()
        
        # Sort and remove duplicates
        list(REMOVE_DUPLICATES DEB_DEPENDS)
        list(SORT DEB_DEPENDS)
        
        # Set the dependencies
        set(CPACK_DEBIAN_PACKAGE_DEPENDS "${DEB_DEPENDS}" CACHE STRING "Auto-generated Debian dependencies")
        message(STATUS "Generated Debian dependencies: ${CPACK_DEBIAN_PACKAGE_DEPENDS}")
        
        # Generate RPM dependencies (simpler - just use library names)
        set(RPM_DEPENDS)
        foreach(LIB ${DEPENDENCIES})
            if(LIB MATCHES "^(libc|libstdc\+\+|libm|libpthread|libdl|librt|libgcc_s)")
                continue()
            endif()
            
            # Convert to RPM package names
            string(REGEX REPLACE "^lib([a-zA-Z0-9_]+)\.so\.?[0-9]*" "\\1" RPM_PKG "${LIB}")
            string(TOLOWER "${RPM_PKG}" RPM_PKG)
            
            # Handle special cases for RPM
            if(RPM_PKG MATCHES "qt")
                string(REGEX REPLACE "qt([0-9]+)([a-zA-Z]+)" "qt\\1-\\L\\2" RPM_PKG "${RPM_PKG}")
            elseif(RPM_PKG MATCHES "gdal")
                set(RPM_PKG "gdal")
            elseif(RPM_PKG MATCHES "sdl2")
                set(RPM_PKG "SDL2")
            endif()
            
            if(RPM_PKG)
                list(APPEND RPM_DEPENDS "${RPM_PKG}")
            endif()
        endforeach()
        
        list(REMOVE_DUPLICATES RPM_DEPENDS)
        list(SORT RPM_DEPENDS)
        
        set(CPACK_RPM_PACKAGE_REQUIRES "${RPM_DEPENDS}" CACHE STRING "Auto-generated RPM dependencies")
        message(STATUS "Generated RPM dependencies: ${CPACK_RPM_PACKAGE_REQUIRES}")
        
    else()
        message(WARNING "Failed to run ldd: ${LDD_ERROR}")
    endif()
else()
    message(WARNING "Executable not found at ${EXECUTABLE_FILE}, using manual dependencies")
endif()
