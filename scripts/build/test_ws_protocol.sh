#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/ws_protocol_test"

mkdir -p "${BUILD_DIR}"

gcc -Wall -Wextra -Werror -std=c11 \
    -I"${ROOT_DIR}/src" \
    "${ROOT_DIR}/tests/ws_protocol_test.c" \
    "${ROOT_DIR}/src/network/ws_protocol.c" \
    -o "${BIN}"

"${BIN}"
