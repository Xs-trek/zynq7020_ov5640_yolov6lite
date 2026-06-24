#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build}"
SDK_ENV="${PETALINUX_SDK_ENV:-${HOME}/workdir/plnx_prj/zynq_plnx/images/linux/sdk/environment-setup-cortexa9t2hf-neon-xilinx-linux-gnueabi}"

if [[ ! -f "${SDK_ENV}" ]]; then
    echo "PetaLinux SDK environment script not found: ${SDK_ENV}" >&2
    exit 1
fi

# The PetaLinux SDK script may reference unset variables such as
# LD_LIBRARY_PATH, so source it without nounset and re-enable nounset after.
set +u
# shellcheck disable=SC1090
source "${SDK_ENV}"
set -u

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_TOOLCHAIN_FILE="${ROOT_DIR}/cmake/toolchain.cmake" \
    -DCMAKE_INSTALL_PREFIX=/usr \
    "$@"

cmake --build "${BUILD_DIR}" -j"$(nproc)"
"${ROOT_DIR}/scripts/build/verify_elf.sh" "${BUILD_DIR}/cam_system"
sha256sum "${BUILD_DIR}/cam_system"
