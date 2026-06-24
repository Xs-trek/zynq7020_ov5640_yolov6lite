#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLNX_PROJECT="${PLNX_PROJECT:-${HOME}/workdir/plnx_prj/zynq_plnx}"
META_USER="${PLNX_PROJECT}/project-spec/meta-user"
RECIPE_SRC="${ROOT_DIR}/packaging/petalinux/recipes-support/cam-opencv/cam-opencv_4.13.0.bb"
RECIPE_DST_DIR="${META_USER}/recipes-support/cam-opencv"
PREFERRED_CONF="${META_USER}/conf/petalinuxbsp.conf"

if [[ ! -d "${META_USER}" ]]; then
    echo "PetaLinux meta-user not found: ${META_USER}" >&2
    exit 1
fi

install -d "${RECIPE_DST_DIR}"
install -m 0644 "${RECIPE_SRC}" "${RECIPE_DST_DIR}/cam-opencv_4.13.0.bb"

if [[ ! -f "${PREFERRED_CONF}" ]]; then
    install -d "$(dirname "${PREFERRED_CONF}")"
    : > "${PREFERRED_CONF}"
fi

if grep -q '^PREFERRED_VERSION_cam-opencv[[:space:]]*=' "${PREFERRED_CONF}"; then
    sed -i 's/^PREFERRED_VERSION_cam-opencv[[:space:]]*=.*/PREFERRED_VERSION_cam-opencv = "4.13.0"/' "${PREFERRED_CONF}"
else
    printf '\nPREFERRED_VERSION_cam-opencv = "4.13.0"\n' >> "${PREFERRED_CONF}"
fi

echo "Staged cam-opencv 4.13.0 recipe in ${RECIPE_DST_DIR}"
echo "Pinned PREFERRED_VERSION_cam-opencv in ${PREFERRED_CONF}"
