#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CAM_OTA="${CAM_OTA:-${ROOT_DIR}/packaging/petalinux/recipes-apps/cam-ota/files/cam-ota}"

fail()
{
    echo "validate_ota_env: $*" >&2
    exit 1
}

require_line()
{
    local file="$1"
    local pattern="$2"
    local description="$3"

    grep -Eq "${pattern}" "${file}" || fail "missing ${description}"
}

require_absent()
{
    local file="$1"
    local pattern="$2"
    local description="$3"

    if grep -Eq "${pattern}" "${file}"; then
        fail "unexpected ${description}"
    fi
}

require_no_file()
{
    local file="$1"
    local description="$2"

    [[ ! -e "${file}" ]] || fail "unexpected ${description}: ${file}"
}

for cmd in awk cmp grep sed sha256sum wc; do
    command -v "${cmd}" >/dev/null 2>&1 || fail "required command not found: ${cmd}"
done

[[ -x "${CAM_OTA}" ]] || fail "cam-ota is not executable: ${CAM_OTA}"
sh -n "${CAM_OTA}"

tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT

printf 'dummy-fit-for-env-validation\n' > "${tmp}/image.ub"
printf 'dummy-fit-for-pending-validation\n' > "${tmp}/candidate.ub"

CAM_OTA_BOOT_DIR="${tmp}" "${CAM_OTA}" init >/dev/null
CAM_OTA_BOOT_DIR="${tmp}" "${CAM_OTA}" verify >/dev/null 2>&1 || fail "cam-ota verify failed after init"

hook_txt="${tmp}/ota/state/uEnv.txt"
uenv_txt="${tmp}/uEnv.txt"
state_txt="${tmp}/ota/state/current.env"
rollback_txt="${tmp}/ota/state/uEnv.rollback"
[[ -s "${hook_txt}" ]] || fail "generated state uEnv copy is missing after init"
[[ -s "${uenv_txt}" ]] || fail "generated /uEnv.txt is missing after init"
cmp -s "${hook_txt}" "${uenv_txt}" || fail "state uEnv copy and boot uEnv differ after init"

require_no_file "${tmp}/uboot.env" "persistent U-Boot environment after init"
require_no_file "${tmp}/uboot-redund.env" "redundant persistent U-Boot environment after init"
require_no_file "${rollback_txt}" "rollback hook after init"
require_line "${uenv_txt}" '^ota_slot=a$' "init slot"
require_line "${uenv_txt}" '^ota_state=good$' "init state"
require_line "${uenv_txt}" '^uenvcmd=setenv bootargs console=ttyPS0,115200 earlycon root=/dev/ram0 rw; echo Loading OTA slot a: ota/slot_a/image\.ub; if fatload \$\{devtype\} \$\{devnum\}:\$\{distro_bootpart\} 0x10000000 ota/slot_a/image\.ub; then if iminfo 0x10000000; then setenv bootargs console=ttyPS0,115200 earlycon root=/dev/ram0 rw cam_ota_slot=a cam_ota_state=good cam_ota_prev_slot=a; echo Booting OTA slot a: ota/slot_a/image\.ub; bootm 0x10000000; else echo OTA image failed iminfo; fi; fi; echo OTA image load failed; setenv bootargs console=ttyPS0,115200 earlycon root=/dev/ram0 rw; setenv fitimage_name image\.ub$' "init slot iminfo boot with baseline fallback"
require_absent "${uenv_txt}" '^bootcmd=|^altbootcmd=|^upgrade_available=|^bootlimit=|^bootcount=' "persistent env replacement variables in good uEnv"

CAM_OTA_BOOT_DIR="${tmp}" "${CAM_OTA}" install "${tmp}/candidate.ub" pending-test >/dev/null
CAM_OTA_BOOT_DIR="${tmp}" "${CAM_OTA}" verify >/dev/null 2>&1 || fail "cam-ota verify failed after pending install"

[[ -s "${hook_txt}" ]] || fail "generated state uEnv copy is missing after pending install"
[[ -s "${uenv_txt}" ]] || fail "generated /uEnv.txt is missing after pending install"
cmp -s "${hook_txt}" "${uenv_txt}" || fail "state uEnv copy and boot uEnv differ after pending install"

require_no_file "${tmp}/uboot.env" "persistent U-Boot environment after pending install"
require_no_file "${tmp}/uboot-redund.env" "redundant persistent U-Boot environment after pending install"
[[ -s "${rollback_txt}" ]] || fail "generated rollback uEnv is missing after pending install"
require_line "${uenv_txt}" '^ota_slot=b$' "pending slot"
require_line "${uenv_txt}" '^ota_state=pending$' "pending state"
require_line "${rollback_txt}" '^ota_slot=a$' "rollback slot"
require_line "${rollback_txt}" '^ota_state=rollback$' "rollback state"
require_line "${rollback_txt}" '^uenvcmd=setenv bootargs console=ttyPS0,115200 earlycon root=/dev/ram0 rw; echo Loading rollback OTA slot a: ota/slot_a/image\.ub; if fatload \$\{devtype\} \$\{devnum\}:\$\{distro_bootpart\} 0x10000000 ota/slot_a/image\.ub; then if iminfo 0x10000000; then setenv bootargs console=ttyPS0,115200 earlycon root=/dev/ram0 rw cam_ota_slot=a cam_ota_state=rollback cam_ota_prev_slot=a; echo Rolling back to OTA slot a: ota/slot_a/image\.ub; bootm 0x10000000; else echo OTA rollback image failed iminfo; fi; fi; echo OTA rollback image load failed; setenv bootargs console=ttyPS0,115200 earlycon root=/dev/ram0 rw; setenv fitimage_name image\.ub$' "rollback slot iminfo boot with baseline fallback"
require_line "${uenv_txt}" '^uenvcmd=setenv bootargs console=ttyPS0,115200 earlycon root=/dev/ram0 rw; setenv ota_rollback_armed 0; echo Arming OTA rollback hook: ota/state/uEnv\.rollback; if fatload \$\{devtype\} \$\{devnum\}:\$\{distro_bootpart\} 0x02080000 ota/state/uEnv\.rollback; then setenv ota_rollback_size \$\{filesize\}; if fatwrite \$\{devtype\} \$\{devnum\}:\$\{distro_bootpart\} 0x02080000 uEnv\.txt \$\{ota_rollback_size\}; then if fatload \$\{devtype\} \$\{devnum\}:\$\{distro_bootpart\} 0x03000000 uEnv\.txt; then if test "\$\{filesize\}" = "\$\{ota_rollback_size\}"; then setenv ota_rollback_armed 1; echo OTA rollback hook armed and verified.*Loading pending OTA slot b: ota/slot_b/image\.ub; if fatload \$\{devtype\} \$\{devnum\}:\$\{distro_bootpart\} 0x10000000 ota/slot_b/image\.ub; then if iminfo 0x10000000; then.*setenv bootargs console=ttyPS0,115200 earlycon root=/dev/ram0 rw cam_ota_slot=b cam_ota_state=pending cam_ota_prev_slot=a; echo Booting pending OTA slot b: ota/slot_b/image\.ub; bootm 0x10000000.*Falling back to OTA slot a: ota/slot_a/image\.ub; if fatload \$\{devtype\} \$\{devnum\}:\$\{distro_bootpart\} 0x10000000 ota/slot_a/image\.ub; then if iminfo 0x10000000; then bootm 0x10000000.*setenv bootargs console=ttyPS0,115200 earlycon root=/dev/ram0 rw; setenv fitimage_name image\.ub$' "pending verified rollback hook and iminfo boot with baseline fallback"
require_absent "${uenv_txt}" 'saveenv|bootcount|altbootcmd|upgrade_available' "saveenv bootcount rollback dependency in pending uEnv"
pending_cmd="$(sed -n 's/^uenvcmd=//p' "${uenv_txt}")"
pre_iminfo="${pending_cmd%%iminfo 0x10000000; then*}"
if [[ "${pre_iminfo}" == *"cam_ota_state=pending"* ]]; then
    fail "pending bootargs are set before pending image iminfo"
fi
if CAM_OTA_BOOT_DIR="${tmp}" "${CAM_OTA}" install "${tmp}/candidate.ub" repeated-pending >/dev/null 2>&1; then
    fail "cam-ota install succeeded while previous install is still pending"
fi
[[ "$(CAM_OTA_BOOT_DIR="${tmp}" "${CAM_OTA}" state)" == "pending" ]] || fail "cam-ota state did not report pending"

CAM_OTA_BOOT_DIR="${tmp}" "${CAM_OTA}" disable >/dev/null
require_no_file "${tmp}/uEnv.txt" "boot hook after disable"
require_no_file "${hook_txt}" "state uEnv copy after disable"
require_no_file "${state_txt}" "OTA state file after disable"
require_no_file "${tmp}/uboot.env" "persistent U-Boot environment after disable"
require_no_file "${tmp}/uboot-redund.env" "redundant persistent U-Boot environment after disable"
require_no_file "${rollback_txt}" "rollback hook after disable"

echo "validate_ota_env: OK ${CAM_OTA}"
