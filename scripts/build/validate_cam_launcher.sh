#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LAUNCHER_DIR="${ROOT_DIR}/packaging/petalinux/recipes-apps/cam-launcher"
DT_DIR="${ROOT_DIR}/packaging/petalinux/recipes-bsp/device-tree"

fail()
{
    echo "validate_cam_launcher: $*" >&2
    exit 1
}

require_file()
{
    [[ -f "$1" ]] || fail "missing file: $1"
}

require_line()
{
    local path="$1"
    local pattern="$2"
    local description="$3"

    grep -Fq "$pattern" "$path" || fail "missing ${description}: ${pattern}"
}

require_file "${LAUNCHER_DIR}/cam-launcher.bb"
require_file "${LAUNCHER_DIR}/files/cam-launcher"
require_file "${LAUNCHER_DIR}/files/cam-launcher.c"
require_file "${DT_DIR}/device-tree.bbappend"
require_file "${DT_DIR}/files/system-user.dtsi"
require_file "${DT_DIR}/files/pl-custom.dtsi"

sh -n "${LAUNCHER_DIR}/files/cam-launcher"

tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT

gcc -Wall -Wextra -Werror -std=c11 -D_GNU_SOURCE \
    "${LAUNCHER_DIR}/files/cam-launcher.c" \
    -o "${tmp}/cam-launcher"

require_line "${DT_DIR}/files/system-user.dtsi" 'compatible = "gpio-keys";' "gpio-keys node"
require_line "${DT_DIR}/files/system-user.dtsi" 'pinctrl-0 = <&pinctrl_ps_key1_default>;' "PS_KEY1 pinctrl binding"
require_line "${DT_DIR}/files/system-user.dtsi" 'label = "cam-ps-key1";' "PS_KEY1 label"
require_line "${DT_DIR}/files/system-user.dtsi" 'gpios = <&gpio0 10 1>;' "PS_KEY1 active-low GPIO"
require_line "${DT_DIR}/files/system-user.dtsi" 'linux,code = <148>;' "KEY_PROG1 input code"
require_line "${DT_DIR}/files/system-user.dtsi" 'debounce-interval = <20>;' "software debounce interval"
require_line "${DT_DIR}/files/system-user.dtsi" 'pinctrl_ps_key1_default: ps-key1-default {' "PS_KEY1 pinctrl state"
require_line "${DT_DIR}/files/system-user.dtsi" 'groups = "gpio0_10_grp";' "MIO10 GPIO pin group"
require_line "${DT_DIR}/files/system-user.dtsi" 'function = "gpio0";' "MIO10 GPIO mux function"

require_line "${LAUNCHER_DIR}/cam-launcher.bb" 'INITSCRIPT_NAME = "cam-launcher"' "init script name"
require_line "${LAUNCHER_DIR}/cam-launcher.bb" 'RDEPENDS:${PN} += "cam-system"' "cam-system runtime dependency"
require_line "${LAUNCHER_DIR}/cam-launcher.bb" 'B = "${WORKDIR}/build"' "separate build directory"
require_line "${LAUNCHER_DIR}/cam-launcher.bb" '${sbindir}/cam-launcher' "sbin install path"

if grep -Rqs '/sys/class/gpio' "${LAUNCHER_DIR}"; then
    fail "cam-launcher must use input events, not GPIO sysfs"
fi

echo "validate_cam_launcher: OK"
