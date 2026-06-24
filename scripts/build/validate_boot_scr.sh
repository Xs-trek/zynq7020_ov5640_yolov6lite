#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLNX_PROJECT="${PLNX_PROJECT:-${HOME}/workdir/plnx_prj/zynq_plnx}"
BBAPPEND="${ROOT_DIR}/packaging/petalinux/recipes-bsp/u-boot/u-boot-zynq-scr.bbappend"
BOOT_CMD_SRC="${BOOT_CMD_SRC:-}"
BOOT_SCR="${BOOT_SCR:-${PLNX_PROJECT}/images/linux/boot.scr}"

fail()
{
    echo "validate_boot_scr: $*" >&2
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

validate_boot_scr_image()
{
    local image="$1"

    [[ -f "${image}" ]] || fail "boot.scr not found: ${image}"
    strings "${image}" > "${tmp}/boot.scr.strings"
    grep -Fq 'if test -n "${uenvcmd}"; then' "${tmp}/boot.scr.strings" || fail "boot.scr misses quoted uenvcmd test: ${image}"
    if grep -Fq 'if test -n $uenvcmd; then' "${tmp}/boot.scr.strings"; then
        fail "boot.scr still contains unquoted uenvcmd test: ${image}"
    fi
    mkimage -l "${image}" >/dev/null
}

for cmd in find grep mkimage sed strings; do
    command -v "${cmd}" >/dev/null 2>&1 || fail "required command not found: ${cmd}"
done

[[ -f "${BBAPPEND}" ]] || fail "bbappend not found: ${BBAPPEND}"
grep -Fq 'if test -n "\${uenvcmd}"; then' "${BBAPPEND}" || fail "bbappend does not add quoted uenvcmd test"
grep -Fq 'unquoted uenvcmd test remains' "${BBAPPEND}" || fail "bbappend does not guard against stale unquoted test"

tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT

src="$(find_boot_cmd || true)"
if [[ -n "${src}" ]]; then
    sed 's/if test -n $uenvcmd; then/if test -n "${uenvcmd}"; then/' "${src}" > "${tmp}/boot.cmd"
    grep -Fq 'if test -n "${uenvcmd}"; then' "${tmp}/boot.cmd" || fail "patched boot.cmd misses quoted uenvcmd test"
    if grep -Fq 'if test -n $uenvcmd; then' "${tmp}/boot.cmd"; then
        fail "patched boot.cmd still contains unquoted uenvcmd test"
    fi

    mkimage -A arm -T script -C none -n "Boot script" -d "${tmp}/boot.cmd" "${tmp}/derived.boot.scr" >/dev/null
    validate_boot_scr_image "${tmp}/derived.boot.scr"
fi

validate_boot_scr_image "${BOOT_SCR}"

if [[ -n "${src}" ]]; then
    echo "validate_boot_scr: OK ${src} ${BOOT_SCR}"
else
    echo "validate_boot_scr: OK ${BOOT_SCR}"
fi
