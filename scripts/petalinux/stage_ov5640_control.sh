#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLNX_PROJECT="${PLNX_PROJECT:-${HOME}/workdir/plnx_prj/zynq_plnx}"
META_USER="${PLNX_PROJECT}/project-spec/meta-user"

RECIPE_SRC="${ROOT_DIR}/packaging/petalinux/recipes-kernel/cam-ov5640-control"
RECIPE_DST="${META_USER}/recipes-kernel/cam-ov5640-control"
KERNEL_CFG_SRC="${ROOT_DIR}/packaging/petalinux/recipes-kernel/linux/linux-xlnx/cam_ov5640_control.cfg"
KERNEL_CFG_DST_DIR="${META_USER}/recipes-kernel/linux/linux-xlnx"
KERNEL_BBAPPEND="${META_USER}/recipes-kernel/linux/linux-xlnx_%.bbappend"
IMAGE_APPEND="${META_USER}/recipes-core/images/petalinux-image-minimal.bbappend"

if [[ ! -d "${META_USER}" ]]; then
    echo "PetaLinux project not found or missing meta-user: ${PLNX_PROJECT}" >&2
    exit 1
fi

install -d "${RECIPE_DST}/files"
install -m 0644 "${RECIPE_SRC}/cam-ov5640-control_1.0.bb" \
    "${RECIPE_DST}/cam-ov5640-control_1.0.bb"
install -m 0644 "${RECIPE_SRC}/files/Makefile" \
    "${RECIPE_DST}/files/Makefile"
install -m 0644 "${RECIPE_SRC}/files/cam-ov5640-control.c" \
    "${RECIPE_DST}/files/cam-ov5640-control.c"
install -m 0644 "${RECIPE_SRC}/files/cam-dvp-bridge.c" \
    "${RECIPE_DST}/files/cam-dvp-bridge.c"

install -d "${KERNEL_CFG_DST_DIR}"
install -m 0644 "${KERNEL_CFG_SRC}" \
    "${KERNEL_CFG_DST_DIR}/cam_ov5640_control.cfg"

if [[ ! -f "${KERNEL_BBAPPEND}" ]]; then
    printf 'FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"\n' > "${KERNEL_BBAPPEND}"
fi
if ! grep -q 'cam_ov5640_control.cfg' "${KERNEL_BBAPPEND}"; then
    printf '\nSRC_URI += "file://cam_ov5640_control.cfg"\n' >> "${KERNEL_BBAPPEND}"
fi

install -d "$(dirname "${IMAGE_APPEND}")"
if [[ ! -f "${IMAGE_APPEND}" ]]; then
    : > "${IMAGE_APPEND}"
fi
if ! grep -Eq '(^|[[:space:]])kernel-module-cam-ov5640-control([[:space:]]|")' "${IMAGE_APPEND}"; then
    printf '\nIMAGE_INSTALL:append = " kernel-module-cam-ov5640-control"\n' >> "${IMAGE_APPEND}"
fi
if ! grep -Eq '(^|[[:space:]])kernel-module-cam-dvp-bridge([[:space:]]|")' "${IMAGE_APPEND}"; then
    printf '\nIMAGE_INSTALL:append = " kernel-module-cam-dvp-bridge"\n' >> "${IMAGE_APPEND}"
fi

echo "Staged cam-ov5640-control kernel module recipe in ${RECIPE_DST}"
sha256sum "${RECIPE_DST}/files/cam-ov5640-control.c"
sha256sum "${RECIPE_DST}/files/cam-dvp-bridge.c"
