set(OTCSDK_ROOT_DIR "$ENV{OTCSDK_ROOT_DIR}")

if(NOT OTCSDK_ROOT_DIR)
    set(OTCSDK_ROOT_DIR "$ENV{NP_CAMERASDK}")
endif()

if(NOT OTCSDK_ROOT_DIR)
    message(FATAL_ERROR "Neither OTCSDK_ROOT_DIR nor NP_CAMERASDK is set")
endif()

message(STATUS "OTCSDK_ROOT_DIR = ${OTCSDK_ROOT_DIR}")

find_path(OTCSDK_INCLUDE_DIR
    NAMES cameralibrary.h
    PATHS "${OTCSDK_ROOT_DIR}/include"
)

find_library(OTCSDK_IMPORT_LIB
    NAMES CameraLibrary2019x64S
    PATHS "${OTCSDK_ROOT_DIR}/lib"
)

find_file(OTCSDK_SHARED_LIB
    NAMES CameraLibrary2019x64S.dll
    PATHS "${OTCSDK_ROOT_DIR}/lib"
)

message(STATUS "OTCSDK_ROOT_DIR ${OTCSDK_ROOT_DIR}")
message(STATUS "OTCSDK_INCLUDE_DIR ${OTCSDK_INCLUDE_DIR}")

if(WIN32)
  find_library(OTCSDK_IMPORT_LIB CameraLibrary2019x64S HINTS "${OTCSDK_ROOT_DIR}/lib")
  find_file(OTCSDK_SHARED_LIB NAMES "CameraLibrary2019x64S.dll" HINTS "${OTCSDK_ROOT_DIR}/lib")
else()
  find_library(OTCSDK_IMPORT_LIB CameraLibrary HINTS "${OTCSDK_ROOT_DIR}/lib")
  set(OTCSDK_SHARED_LIB "${OTCSDK_IMPORT_LIB}")
endif()

message(STATUS "OTCSDK_IMPORT_LIB DIR ${OTCSDK_IMPORT_LIB}")
message(STATUS "OTCSDK_SHARED_LIB DIR ${OTCSDK_SHARED_LIB}")

include_directories("${OTCSDK_INCLUDE_DIR}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OTCSDK DEFAULT_MSG OTCSDK_ROOT_DIR OTCSDK_INCLUDE_DIR OTCSDK_IMPORT_LIB OTCSDK_SHARED_LIB)

include(CreateImportTargetHelpers)
generate_import_target(OTCSDK SHARED TARGET CameraLibrary::CameraLibrary)
