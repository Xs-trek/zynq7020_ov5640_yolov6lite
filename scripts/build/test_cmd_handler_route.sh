#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/cmd_handler_route_test"

mkdir -p "${BUILD_DIR}"

gcc -Wall -Wextra -Werror -std=c11 \
    -I"${ROOT_DIR}/include" \
    -I"${ROOT_DIR}/src" \
    -I"${ROOT_DIR}/third_party" \
    "${ROOT_DIR}/tests/cmd_handler_route_test.c" \
    "${ROOT_DIR}/src/network/cmd_handler.c" \
    "${ROOT_DIR}/third_party/cjson/cJSON.c" \
    -lpthread \
    -o "${BIN}"

"${BIN}"
