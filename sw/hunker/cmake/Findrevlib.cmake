# Findrevlib.cmake
# This file is used to locate the REVLib library and its headers

find_path(REVLIB_INCLUDE_DIR
  NAMES SparkMax.h
  PATHS /usr/include /usr/local/include/rev
  HINTS ${CMAKE_PREFIX_PATH}
)

find_library(REVLIB_LIBRARY
  NAMES revlib
  PATHS /usr/lib /usr/local/lib
  HINTS ${CMAKE_PREFIX_PATH}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(revlib DEFAULT_MSG REVLIB_INCLUDE_DIR REVLIB_LIBRARY)

if(REVLIB_FOUND)
  set(REVLIB_LIBRARIES ${REVLIB_LIBRARY})
  set(REVLIB_INCLUDE_DIRS ${REVLIB_INCLUDE_DIR})
else()
  set(REVLIB_LIBRARIES)
  set(REVLIB_INCLUDE_DIRS)
endif()

mark_as_advanced(REVLIB_INCLUDE_DIR REVLIB_LIBRARY)