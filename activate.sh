#!/bin/bash

if [ -z "$OTCSDK_ROOT_DIR" ]; then
    echo "Error: OTCSDK_ROOT_DIR is not set. Please set it to the OptiTrack Camera SDK root directory."
    echo "  export OTCSDK_ROOT_DIR=/path/to/CameraSDK"
else
    export CMAKE_ARGS="-DOTCSDK_ROOT_DIR=$OTCSDK_ROOT_DIR"
    export LD_LIBRARY_PATH="$OTCSDK_ROOT_DIR/lib:$LD_LIBRARY_PATH"
    export LD_PRELOAD="libjpeg.so.8${LD_PRELOAD:+:$LD_PRELOAD}"
fi