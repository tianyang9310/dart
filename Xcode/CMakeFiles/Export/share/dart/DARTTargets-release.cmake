#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dart-core" for configuration "Release"
set_property(TARGET dart-core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(dart-core PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "/usr/local/Cellar/libccd/2.0/lib/libccd.dylib;/usr/local/Cellar/fcl/0.3.2/lib/libfcl.dylib;/usr/local/Cellar/assimp/3.1.1/lib/libassimp.dylib;/usr/local/lib/libboost_regex-mt.dylib;/usr/local/lib/libboost_system-mt.dylib;/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.12.sdk/System/Library/Frameworks/AGL.framework;/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.12.sdk/System/Library/Frameworks/OpenGL.framework;/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.12.sdk/System/Library/Frameworks/GLUT.framework;/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.12.sdk/System/Library/Frameworks/Cocoa.framework;BulletSoftBody;BulletDynamics;BulletCollision;LinearMath"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libdart-core.5.1.2.dylib"
  IMPORTED_SONAME_RELEASE "@rpath/libdart-core.5.1.dylib"
  )

list(APPEND _IMPORT_CHECK_TARGETS dart-core )
list(APPEND _IMPORT_CHECK_FILES_FOR_dart-core "${_IMPORT_PREFIX}/lib/libdart-core.5.1.2.dylib" )

# Import target "dart" for configuration "Release"
set_property(TARGET dart APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(dart PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "dart-core;/usr/local/Cellar/urdfdom/0.2.10/lib/liburdfdom_sensor.dylib;/usr/local/Cellar/urdfdom/0.2.10/lib/liburdfdom_model_state.dylib;/usr/local/Cellar/urdfdom/0.2.10/lib/liburdfdom_model.dylib;/usr/local/Cellar/urdfdom/0.2.10/lib/liburdfdom_world.dylib;/usr/local/lib/libconsole_bridge.dylib;/usr/local/opt/tinyxml/lib/libtinyxml.dylib;/usr/local/Cellar/tinyxml2/3.0.0/lib/libtinyxml2.dylib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libdart.5.1.2.dylib"
  IMPORTED_SONAME_RELEASE "@rpath/libdart.5.1.dylib"
  )

list(APPEND _IMPORT_CHECK_TARGETS dart )
list(APPEND _IMPORT_CHECK_FILES_FOR_dart "${_IMPORT_PREFIX}/lib/libdart.5.1.2.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
