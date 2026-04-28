@echo off
if not defined OTCSDK_ROOT_DIR (
    echo Error: OTCSDK_ROOT_DIR is not set. Please set it to the OptiTrack Camera SDK root directory.
    echo   set OTCSDK_ROOT_DIR=C:\path\to\CameraSDK
) else (
    set CMAKE_ARGS=-DOTCSDK_ROOT_DIR="%OTCSDK_ROOT_DIR%"
    set PATH=%OTCSDK_ROOT_DIR%\bin;%PATH%
)