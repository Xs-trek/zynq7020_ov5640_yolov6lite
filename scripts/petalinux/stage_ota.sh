#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLNX_PROJECT="${PLNX_PROJECT:-${HOME}/workdir/plnx_prj/zynq_plnx}"
META_USER="${PLNX_PROJECT}/project-spec/meta-user"

if [[ ! -d "${META_USER}" ]]; then
    echo "PetaLinux meta-user layer not found: ${META_USER}" >&2
    exit 1
fi

"${ROOT_DIR}/scripts/build/validate_ota_env.sh"
"${ROOT_DIR}/scripts/build/validate_boot_scr.sh"

install -d "${META_USER}/recipes-bsp/u-boot/files"
rm -f "${META_USER}/recipes-bsp/u-boot/u-boot-zynq-scr_%.bbappend"
install -m 0644 \
    "${ROOT_DIR}/packaging/petalinux/recipes-bsp/u-boot/u-boot-xlnx_%.bbappend" \
    "${META_USER}/recipes-bsp/u-boot/u-boot-xlnx_%.bbappend"
install -m 0644 \
    "${ROOT_DIR}/packaging/petalinux/recipes-bsp/u-boot/u-boot-zynq-scr.bbappend" \
    "${META_USER}/recipes-bsp/u-boot/u-boot-zynq-scr.bbappend"
install -m 0644 \
    "${ROOT_DIR}/packaging/petalinux/recipes-bsp/u-boot/files/bsp.cfg" \
    "${META_USER}/recipes-bsp/u-boot/files/bsp.cfg"
install -m 0644 \
    "${ROOT_DIR}/packaging/petalinux/recipes-bsp/u-boot/files/platform-top.h" \
    "${META_USER}/recipes-bsp/u-boot/files/platform-top.h"

install -d "${META_USER}/recipes-apps/cam-ota/files"
rm -f "${META_USER}/recipes-apps/cam-ota/files/S03cam-ota-bootguard"
install -m 0644 \
    "${ROOT_DIR}/packaging/petalinux/recipes-apps/cam-ota/cam-ota.bb" \
    "${META_USER}/recipes-apps/cam-ota/cam-ota.bb"
install -m 0755 \
    "${ROOT_DIR}/packaging/petalinux/recipes-apps/cam-ota/files/cam-ota" \
    "${META_USER}/recipes-apps/cam-ota/files/cam-ota"
install -m 0755 \
    "${ROOT_DIR}/packaging/petalinux/recipes-apps/cam-ota/files/cam-ota-bootguard" \
    "${META_USER}/recipes-apps/cam-ota/files/cam-ota-bootguard"
rm -f "${META_USER}/recipes-apps/cam-ota/files/fw_env.config"

image_append="${META_USER}/recipes-core/images/petalinux-image-minimal.bbappend"
if [[ -f "${image_append}" ]] && ! grep -Eq '(^|[[:space:]])cam-ota([[:space:]]|")' "${image_append}"; then
    printf '\nIMAGE_INSTALL:append = " cam-ota"\n' >> "${image_append}"
fi

echo "Staged bootloader OTA support in ${META_USER}"
