#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "NodeEditor::nodes" for configuration ""
set_property(TARGET NodeEditor::nodes APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(NodeEditor::nodes PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libnodes.so"
  IMPORTED_SONAME_NOCONFIG "libnodes.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS NodeEditor::nodes )
list(APPEND _IMPORT_CHECK_FILES_FOR_NodeEditor::nodes "${_IMPORT_PREFIX}/lib/libnodes.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
