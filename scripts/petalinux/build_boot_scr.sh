#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLNX_PROJECT="${PLNX_PROJECT:-${HOME}/workdir/plnx_prj/zynq_plnx}"
OUTPUT="${1:-${PLNX_PROJECT}/images/linux/boot.scr}"
BOOT_CMD_SRC="${BOOT_CMD_SRC:-}"

fail()
{
    echo "build_boot_scr: $*" >&2
    exit 1
}

find_boot_cmd()
{
    if [[ -n "${BOOT_CMD_SRC}" ]]; then
        printf '%s\n' "${BOOT_CMD_SRC}"
        return
    fi

    local work_dir="${PLNX_PROJECT}/build/tmp/work"
    local found
    found="$(find "${work_dir}" -path '*/u-boot-zynq-scr/*/boot.cmd' -type f 2>/dev/null | sort | tail -n 1 || true)"
    [[ -n "${found}" ]] || return 1
    printf '%s\n' "${found}"
}

for cmd in find grep install mkimage mkdir mv sed strings; do
    command -v "${cmd}" >/dev/null 2>&1 || fail "required command not found: ${cmd}"
done

if ! src="$(find_boot_cmd)"; then
    if ! command -v petalinux-build >/dev/null 2>&1; then
        fail "boot.cmd is unavailable and petalinux-build is not in PATH"
    fi
    "${ROOT_DIR}/scripts/build/validate_boot_scr.sh" >/dev/null
    (
        cd "${PLNX_PROJECT}"
        petalinux-build -c u-boot-zynq-scr
    )
    "${ROOT_DIR}/scripts/build/validate_boot_scr.sh" >/dev/null
    echo "build_boot_scr: recipe regenerated ${OUTPUT}"
    exit 0
fi

tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT

sed 's/if test -n $uenvcmd; then/if test -n "${uenvcmd}"; then/' "${src}" > "${tmp}/boot.cmd"
mkimage -A arm -T script -C none -n "Boot script" -d "${tmp}/boot.cmd" "${tmp}/boot.scr" >/dev/null
strings "${tmp}/boot.scr" > "${tmp}/boot.scr.strings"

grep -Fq 'if test -n "${uenvcmd}"; then' "${tmp}/boot.scr.strings" || fail "generated boot.scr misses quoted uenvcmd test"
if grep -Fq 'if test -n $uenvcmd; then' "${tmp}/boot.scr.strings"; then
    fail "generated boot.scr still contains unquoted uenvcmd test"
fi

mkdir -p "$(dirname "${OUTPUT}")"
install -m 0644 "${tmp}/boot.scr" "${tmp}/boot.scr.out"
mv "${tmp}/boot.scr.out" "${OUTPUT}"

echo "build_boot_scr: wrote ${OUTPUT} from ${src}"
