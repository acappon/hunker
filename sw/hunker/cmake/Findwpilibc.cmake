# Findwpilibc.cmake
# This file is used to locate the wpilibc library and its headers

find_path(WPILIBC_INCLUDE_DIR
  NAMES PWM.h
  PATHS /usr/include /usr/local/include/frc
  HINTS ${CMAKE_PREFIX_PATH}
)

find_library(WPILIBC_LIBRARY
  NAMES wpilibc
  PATHS /usr/lib /usr/local/lib
  HINTS ${CMAKE_PREFIX_PATH}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(wpilibc DEFAULT_MSG WPILIBC_INCLUDE_DIR WPILIBC_LIBRARY)

if(WPILIBC_FOUND)
  set(WPILIBC_LIBRARIES ${WPILIBC_LIBRARY})
  set(WPILIBC_INCLUDE_DIRS ${WPILIBC_INCLUDE_DIR})
else()
  set(WPILIBC_LIBRARIES)
  set(WPILIBC_INCLUDE_DIRS)
endif()

mark_as_advanced(WPILIBC_INCLUDE_DIR WPILIBC_LIBRARY)