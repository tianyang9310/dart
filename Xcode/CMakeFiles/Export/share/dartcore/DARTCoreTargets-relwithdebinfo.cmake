#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dart-core" for configuration "RelWithDebInfo"
set_property(TARGET dart-core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(dart-core PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "/usr/local/Cellar/libccd/2.0/lib/libccd.dylib;/usr/local/Cellar/fcl/0.3.2/lib/libfcl.dylib;/usr/local/Cellar/assimp/3.1.1/lib/libassimp.dylib;/usr/local/lib/libboost_regex-mt.dylib;/usr/local/lib/libboost_system-mt.dylib;/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.12.sdk/System/Library/Frameworks/AGL.framework;/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.12.sdk/System/Library/Frameworks/OpenGL.framework;/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.12.sdk/System/Library/Frameworks/GLUT.framework;/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.12.sdk/System/Library/Frameworks/Cocoa.framework;BulletSoftBody;BulletDynamics;BulletCollision;LinearMath"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libdart-core.5.1.2.dylib"
  IMPORTED_SONAME_RELWITHDEBINFO "@rpath/libdart-core.5.1.dylib"
  )

list(APPEND _IMPORT_CHECK_TARGETS dart-core )
list(APPEND _IMPORT_CHECK_FILES_FOR_dart-core "${_IMPORT_PREFIX}/lib/libdart-core.5.1.2.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
