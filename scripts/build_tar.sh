#!/bin/bash
# Convenience build script for vision_app standalone project
# Usage: ./build_tar.sh [--clean] [--cpu] [--gpu] [--mecheye]

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 获取项目根目录
PROJECT_ROOT_DIR="$(dirname "$SCRIPT_DIR")"

BUILD_DIR="$PROJECT_ROOT_DIR/build"
CMAKE_ARGS=""

for arg in "$@"; do
    case $arg in
        --clean)
            echo "Cleaning build directory..."
            rm -rf "$BUILD_DIR"
            ;;
        --cpu)
            CMAKE_ARGS="$CMAKE_ARGS -DENABLE_GPU=OFF"
            ;;
        --gpu)
            CMAKE_ARGS="$CMAKE_ARGS -DENABLE_GPU=ON"
            ;;
        --mechmind)
            CMAKE_ARGS="$CMAKE_ARGS -DENABLE_MECHEYE=ON"
            ;;
        --transfer)
            CMAKE_ARGS="$CMAKE_ARGS -DENABLE_TRANSFER=ON"
            ;;
        --d435bridge)
            CMAKE_ARGS="$CMAKE_ARGS -DENABLE_D435_BRIDGE=ON"
            ;;
        --fake-client)
            CMAKE_ARGS="$CMAKE_ARGS -DBUILD_FAKE_CLIENT=ON"
            ;;
        -D*)
            CMAKE_ARGS="$CMAKE_ARGS $arg"
            ;;
    esac
done

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
# Force system yaml-cpp to avoid ABI mismatch with ebox (conda yaml-cpp is incompatible)
SYSTEM_YAML_CPP_DIR=/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp
if [ -d "$SYSTEM_YAML_CPP_DIR" ]; then
    CMAKE_ARGS="$CMAKE_ARGS -Dyaml-cpp_DIR=$SYSTEM_YAML_CPP_DIR"
fi
cmake .. $CMAKE_ARGS
TOTAL_CORES=$(nproc)
BUILD_JOBS=$(( TOTAL_CORES * 80 / 100 ))
[ "$BUILD_JOBS" -lt 1 ] && BUILD_JOBS=1
echo "Using $BUILD_JOBS / $TOTAL_CORES cores for build"
cmake --build . --parallel "$BUILD_JOBS"
make package

# 修复 onnxruntime providers 的 RPATH，使其在任何机器上都能找到同目录下的 cuDNN
PATCHELF=$(find /home/vecow/miniconda3 -name "patchelf" -type f 2>/dev/null | head -1)
ORT_LIB="${THIRDPARTY_DIR:-/opt/openmind/hybrid/third_party}/onnxruntime/lib"
if [ -n "$PATCHELF" ] && [ -d "$ORT_LIB" ]; then
    for so in libonnxruntime_providers_cuda.so libonnxruntime_providers_tensorrt.so libonnxruntime_providers_shared.so; do
        [ -f "$ORT_LIB/$so" ] && "$PATCHELF" --set-rpath '$ORIGIN' "$ORT_LIB/$so" && echo "[PATCH] $so RPATH=\$ORIGIN"
    done
fi