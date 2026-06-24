#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLNX_PROJECT="${PLNX_PROJECT:-${HOME}/workdir/plnx_prj/zynq_plnx}"
META_USER="${PLNX_PROJECT}/project-spec/meta-user"

if [[ ! -d "${META_USER}" ]]; then
    echo "PetaLinux meta-user layer not found: ${META_USER}" >&2
    exit 1
fi

install -d "${META_USER}/recipes-kernel/8812bu"
install -m 0644 \
    "${ROOT_DIR}/packaging/petalinux/recipes-kernel/8812bu/8812bu_git.bb" \
    "${META_USER}/recipes-kernel/8812bu/8812bu_git.bb"

rm -rf "${META_USER}/recipes-apps/8812bu"

install -d "${META_USER}/recipes-apps/wifi-init/files"
install -m 0644 \
    "${ROOT_DIR}/packaging/petalinux/recipes-apps/wifi-init/wifi-init.bb" \
    "${META_USER}/recipes-apps/wifi-init/wifi-init.bb"
install -m 0755 \
    "${ROOT_DIR}/packaging/petalinux/recipes-apps/wifi-init/files/S50wifi" \
    "${META_USER}/recipes-apps/wifi-init/files/S50wifi"

image_append="${META_USER}/recipes-core/images/petalinux-image-minimal.bbappend"
if [[ -f "${image_append}" ]]; then
    for pkg in 8812bu wifi-init; do
        if ! grep -Eq "(^|[[:space:]])${pkg}([[:space:]]|\")" "${image_append}"; then
            printf '\nIMAGE_INSTALL:append = " %s"\n' "${pkg}" >> "${image_append}"
        fi
    done
fi

echo "Staged 8812bu source recipe and wifi-init in ${META_USER}"
