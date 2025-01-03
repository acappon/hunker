# Findlgpio.cmake
# This file is used to locate the lgpio library and its headers

find_path(LGPIO_INCLUDE_DIR
  NAMES lgpio.h
  PATHS /usr/include /usr/local/include
)

find_library(LGPIO_LIBRARY
  NAMES lgpio
  PATHS /usr/lib /usr/local/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(lgpio DEFAULT_MSG LGPIO_INCLUDE_DIR LGPIO_LIBRARY)

if(LGPIO_FOUND)
  set(LGPIO_LIBRARIES ${LGPIO_LIBRARY})
  set(LGPIO_INCLUDE_DIRS ${LGPIO_INCLUDE_DIR})
else()
  set(LGPIO_LIBRARIES)
  set(LGPIO_INCLUDE_DIRS)
endif()

mark_as_advanced(LGPIO_INCLUDE_DIR LGPIO_LIBRARY)