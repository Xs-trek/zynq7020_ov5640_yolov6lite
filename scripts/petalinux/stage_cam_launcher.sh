#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLNX_PROJECT="${PLNX_PROJECT:-${HOME}/workdir/plnx_prj/zynq_plnx}"
META_USER="${PLNX_PROJECT}/project-spec/meta-user"

if [[ ! -d "${META_USER}" ]]; then
    echo "PetaLinux meta-user layer not found: ${META_USER}" >&2
    exit 1
fi

"${ROOT_DIR}/scripts/build/validate_cam_launcher.sh"

install -d "${META_USER}/recipes-bsp/device-tree/files"
install -m 0644 \
    "${ROOT_DIR}/packaging/petalinux/recipes-bsp/device-tree/device-tree.bbappend" \
    "${META_USER}/recipes-bsp/device-tree/device-tree.bbappend"
install -m 0644 \
    "${ROOT_DIR}/packaging/petalinux/recipes-bsp/device-tree/files/system-user.dtsi" \
    "${META_USER}/recipes-bsp/device-tree/files/system-user.dtsi"
install -m 0644 \
    "${ROOT_DIR}/packaging/petalinux/recipes-bsp/device-tree/files/pl-custom.dtsi" \
    "${META_USER}/recipes-bsp/device-tree/files/pl-custom.dtsi"

install -d "${META_USER}/recipes-apps/cam-launcher/files"
install -m 0644 \
    "${ROOT_DIR}/packaging/petalinux/recipes-apps/cam-launcher/cam-launcher.bb" \
    "${META_USER}/recipes-apps/cam-launcher/cam-launcher.bb"
install -m 0755 \
    "${ROOT_DIR}/packaging/petalinux/recipes-apps/cam-launcher/files/cam-launcher" \
    "${META_USER}/recipes-apps/cam-launcher/files/cam-launcher"
install -m 0644 \
    "${ROOT_DIR}/packaging/petalinux/recipes-apps/cam-launcher/files/cam-launcher.c" \
    "${META_USER}/recipes-apps/cam-launcher/files/cam-launcher.c"

image_append="${META_USER}/recipes-core/images/petalinux-image-minimal.bbappend"
if [[ -f "${image_append}" ]] && ! grep -Eq '(^|[[:space:]])cam-launcher([[:space:]]|")' "${image_append}"; then
    printf '\nIMAGE_INSTALL:append = " cam-launcher"\n' >> "${image_append}"
fi

echo "Staged PS_KEY1 cam launcher in ${META_USER}"
