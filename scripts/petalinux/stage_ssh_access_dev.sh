#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLNX_PROJECT="${PLNX_PROJECT:-${HOME}/workdir/plnx_prj/zynq_plnx}"
META_USER="${PLNX_PROJECT}/project-spec/meta-user"
RECIPE_DIR="${META_USER}/recipes-apps/ssh-access-dev"
IMAGE_APPEND="${META_USER}/recipes-core/images/petalinux-image-minimal.bbappend"
AUTHORIZED_KEYS="${CAM_SSH_AUTHORIZED_KEYS:-${ROOT_DIR}/packaging/petalinux/recipes-apps/ssh-access-dev/files/authorized_keys}"

if [[ ! -d "${META_USER}" ]]; then
    echo "PetaLinux meta-user layer not found: ${META_USER}" >&2
    exit 1
fi

install -d "${RECIPE_DIR}/files"
install -m 0644 \
    "${ROOT_DIR}/packaging/petalinux/recipes-apps/ssh-access-dev/ssh-access-dev.bb" \
    "${RECIPE_DIR}/ssh-access-dev.bb"
if [[ ! -s "${AUTHORIZED_KEYS}" ]]; then
    cat >&2 <<EOF
ssh-access-dev requires an authorized_keys file that is not committed to the
public repository.

Provide one of:
  CAM_SSH_AUTHORIZED_KEYS=/path/to/authorized_keys $0
  cp packaging/petalinux/recipes-apps/ssh-access-dev/files/authorized_keys.example \\
     packaging/petalinux/recipes-apps/ssh-access-dev/files/authorized_keys
  # then replace the example line with your own public key
EOF
    exit 1
fi
install -m 0644 \
    "${AUTHORIZED_KEYS}" \
    "${RECIPE_DIR}/files/authorized_keys"

if [[ ! -f "${IMAGE_APPEND}" ]]; then
    install -d "$(dirname "${IMAGE_APPEND}")"
    : > "${IMAGE_APPEND}"
fi

if ! grep -Eq '(^|[[:space:]])ssh-access-dev([[:space:]]|")' "${IMAGE_APPEND}"; then
    printf '\nIMAGE_INSTALL:append = " ssh-access-dev"\n' >> "${IMAGE_APPEND}"
fi

echo "Staged ssh-access-dev in ${RECIPE_DIR}"
