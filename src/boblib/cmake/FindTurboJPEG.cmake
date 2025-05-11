# FindTurboJPEG.cmake
find_path(TurboJPEG_INCLUDE_DIR
  NAMES turbojpeg.h
  PATHS /usr/include /usr/local/include
)

find_library(TurboJPEG_LIBRARY
  NAMES turbojpeg
  PATHS /usr/lib /usr/local/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TurboJPEG DEFAULT_MSG
  TurboJPEG_LIBRARY TurboJPEG_INCLUDE_DIR
)

if(TurboJPEG_FOUND)
  set(TurboJPEG_LIBRARIES ${TurboJPEG_LIBRARY})
  set(TurboJPEG_INCLUDE_DIRS ${TurboJPEG_INCLUDE_DIR})
endif()

mark_as_advanced(TurboJPEG_LIBRARY TurboJPEG_INCLUDE_DIR)