find_path(OTCSDK_ROOT_DIR
          NAMES include/camera.h
          HINTS
            "C:/Program Files (x86)/OptiTrack/CameraSDK"
            "$ENV{OTCSDK_ROOT_DIR}"
          NO_CMAKE_PATH)

find_path(OTCSDK_INCLUDE_DIR
          NAMES cameralibrary.h
          HINTS
            "C:/Program Files (x86)/OptiTrack/CameraSDK/include"
            "$ENV{OTCSDK_ROOT_DIR}/include"
            "${OTCSDK_ROOT_DIR}/include"
          NO_CMAKE_PATH)

message(STATUS "OTCSDK ROOT DIR ${OTCSDK_ROOT_DIR}")
message(STATUS "OTCSDK INCLUDE DIR ${OTCSDK_INCLUDE_DIR}")

if(WIN32)
  find_library(OTCSDK_IMPORT_LIB CameraLibrary2019x64S HINTS "${OTCSDK_ROOT_DIR}/lib")
  find_file(OTCSDK_SHARED_LIB NAMES "CameraLibrary2019x64S.dll" HINTS "${OTCSDK_ROOT_DIR}/lib")
else()
  find_library(OTCSDK_IMPORT_LIB CameraLibrary HINTS "${OTCSDK_ROOT_DIR}/lib")
  set(OTCSDK_SHARED_LIB "${OTCSDK_IMPORT_LIB}")
endif()

message(STATUS "OTCSDK IMPORT LIBRARY DIR ${OTCSDK_IMPORT_LIB}")
message(STATUS "OTCSDK SHARED LIBRARY DIR ${OTCSDK_SHARED_LIB}")

include_directories("${OTCSDK_INCLUDE_DIR}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OTCSDK DEFAULT_MSG OTCSDK_ROOT_DIR OTCSDK_INCLUDE_DIR OTCSDK_IMPORT_LIB OTCSDK_SHARED_LIB)

include(CreateImportTargetHelpers)
generate_import_target(OTCSDK SHARED TARGET CameraLibrary::CameraLibrary)
