#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLNX_PROJECT="${PLNX_PROJECT:-${HOME}/workdir/plnx_prj/zynq_plnx}"
META_USER="${PLNX_PROJECT}/project-spec/meta-user"
RECIPE_SRC="${ROOT_DIR}/packaging/petalinux/recipes-graphics/jpeg/libjpeg-turbo_3.1.4.bb"
FILES_SRC_DIR="${ROOT_DIR}/packaging/petalinux/recipes-graphics/jpeg/files"
RECIPE_DST_DIR="${META_USER}/recipes-graphics/jpeg"
PREFERRED_CONF="${META_USER}/conf/petalinuxbsp.conf"

if [[ ! -d "${META_USER}" ]]; then
    echo "PetaLinux meta-user not found: ${META_USER}" >&2
    exit 1
fi

install -d "${RECIPE_DST_DIR}"
install -m 0644 "${RECIPE_SRC}" "${RECIPE_DST_DIR}/libjpeg-turbo_3.1.4.bb"
if [[ -d "${FILES_SRC_DIR}" ]]; then
    install -d "${RECIPE_DST_DIR}/files"
    find "${FILES_SRC_DIR}" -maxdepth 1 -type f -print0 |
        while IFS= read -r -d '' file; do
            install -m 0644 "${file}" "${RECIPE_DST_DIR}/files/$(basename "${file}")"
        done
fi

if [[ ! -f "${PREFERRED_CONF}" ]]; then
    install -d "$(dirname "${PREFERRED_CONF}")"
    : > "${PREFERRED_CONF}"
fi

if grep -q '^PREFERRED_VERSION_libjpeg-turbo[[:space:]]*=' "${PREFERRED_CONF}"; then
    sed -i 's/^PREFERRED_VERSION_libjpeg-turbo[[:space:]]*=.*/PREFERRED_VERSION_libjpeg-turbo = "3.1.4"/' "${PREFERRED_CONF}"
else
    printf '\nPREFERRED_VERSION_libjpeg-turbo = "3.1.4"\n' >> "${PREFERRED_CONF}"
fi

echo "Staged libjpeg-turbo 3.1.4 recipe in ${RECIPE_DST_DIR}"
echo "Pinned PREFERRED_VERSION_libjpeg-turbo in ${PREFERRED_CONF}"
