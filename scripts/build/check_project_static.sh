#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TMP_REPORT="$(mktemp)"
trap 'rm -f "${TMP_REPORT}"' EXIT

SCAN_DIRS=(
    "${ROOT_DIR}/src"
    "${ROOT_DIR}/include"
    "${ROOT_DIR}/packaging/petalinux/recipes-apps/cam-system"
    "${ROOT_DIR}/packaging/petalinux/recipes-kernel/cam-ov5640-control"
    "${ROOT_DIR}/packaging/petalinux/recipes-kernel/cam-dvp-bridge"
)

failures=0

check_absent() {
    local pattern="$1"
    local description="$2"

    : > "${TMP_REPORT}"
    for dir in "${SCAN_DIRS[@]}"; do
        if [[ -d "${dir}" ]]; then
            grep -RInE -- "${pattern}" "${dir}" >> "${TMP_REPORT}" || true
        fi
    done

    if [[ -s "${TMP_REPORT}" ]]; then
        echo "FAIL: ${description}" >&2
        cat "${TMP_REPORT}" >&2
        failures=$((failures + 1))
    fi
}

check_absent '/dev/mem' \
    'runtime source must not use raw /dev/mem access'
check_absent '/sys/class/gpio' \
    'runtime source must not use legacy sysfs GPIO control'
check_absent 'strstr[[:space:]]*\([[:space:]]*request[[:space:]]*,[[:space:]]*"POST /api' \
    'REST API routing must not use substring matching'
check_absent 'TRACK_CMD_TEST|POST /api/test|"test"[[:space:]]*:' \
    'development-only tracking test command must not be present in runtime source'

if [[ "${failures}" -ne 0 ]]; then
    exit 1
fi

echo "project_static_check: OK"
