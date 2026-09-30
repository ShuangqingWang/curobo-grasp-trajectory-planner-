#!/usr/bin/env bash
set -euo pipefail

ORIGINAL_DIR="$PWD"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "$PACKAGE_DIR"

HOST="${HOST:-127.0.0.1}"
START_PORT="${PORT:-8088}"

if [[ $# -gt 1 ]]; then
    echo "用法: bash scripts/simulation/serve_noetic_demo.sh [config/grasp_planner.json]" >&2
    exit 2
fi

CONFIG_INPUT="${1:-config/grasp_planner.json}"
if [[ "$CONFIG_INPUT" = /* ]]; then
    CONFIG_CANDIDATE="$CONFIG_INPUT"
elif [[ -f "$ORIGINAL_DIR/$CONFIG_INPUT" ]]; then
    CONFIG_CANDIDATE="$ORIGINAL_DIR/$CONFIG_INPUT"
else
    CONFIG_CANDIDATE="$PACKAGE_DIR/$CONFIG_INPUT"
fi
if [[ ! -f "$CONFIG_CANDIDATE" ]]; then
    echo "仿真 config 文件不存在: $CONFIG_INPUT" >&2
    exit 2
fi
CONFIG_ABS="$(realpath "$CONFIG_CANDIDATE")"
case "$CONFIG_ABS" in
    "$PACKAGE_DIR"/*) ;;
    *)
        echo "仿真 config 必须位于项目目录内: $PACKAGE_DIR" >&2
        exit 2
        ;;
esac
CONFIG_REL="${CONFIG_ABS#"$PACKAGE_DIR"/}"
CONFIG_QUERY="/${CONFIG_REL}"

SERVER="$PACKAGE_DIR/build/OpenmindVisualizerServer"
if [[ ! -x "$SERVER" ]]; then
    echo "找不到 $SERVER ，请先 bash scripts/build_tar.sh" >&2
    exit 2
fi

echo "AUBO-i12H visualizer config: ${CONFIG_ABS}"
echo "AUBO-i12H visualizer: http://${HOST}:${START_PORT}/scripts/simulation/?config=${CONFIG_QUERY}"
exec "$SERVER" --host "$HOST" --port "$START_PORT" --directory "$PACKAGE_DIR"
