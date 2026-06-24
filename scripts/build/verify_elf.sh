#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 /path/to/cam_system" >&2
    exit 2
fi

elf="$1"
readelf_cmd="${READELF:-readelf}"
file_cmd="${FILE:-file}"

fail()
{
    echo "verify_elf: $*" >&2
    exit 1
}

[[ -f "${elf}" ]] || fail "file not found: ${elf}"
command -v "${readelf_cmd}" >/dev/null 2>&1 || fail "readelf not found: ${readelf_cmd}"
command -v "${file_cmd}" >/dev/null 2>&1 || fail "file not found: ${file_cmd}"

file_out="$("${file_cmd}" "${elf}")"
case "${file_out}" in
    *"ELF 32-bit"*"ARM"*)
        ;;
    *)
        fail "unexpected ELF identity: ${file_out}"
        ;;
esac

elf_header="$("${readelf_cmd}" -h "${elf}")"
grep -q 'Machine:[[:space:]]*ARM' <<<"${elf_header}" || fail "ELF machine is not ARM"

attrs="$("${readelf_cmd}" -A "${elf}" 2>/dev/null || true)"
grep -q 'Tag_CPU_arch:[[:space:]]*v7' <<<"${attrs}" || fail "missing ARMv7 build attribute"
grep -q 'Tag_Advanced_SIMD_arch:[[:space:]]*NEON' <<<"${attrs}" || fail "missing NEON build attribute"
grep -q 'Tag_ABI_VFP_args:[[:space:]]*VFP registers' <<<"${attrs}" || fail "missing hard-float ABI attribute"

if grep -Eq 'Tag_FP_arch:.*VFPv4|Tag_FP_arch:.*ARMv8|Tag_FP_arch:.*FP for ARMv8' <<<"${attrs}"; then
    fail "ELF advertises unsupported FP architecture for Cortex-A9"
fi

dynamic="$("${readelf_cmd}" -d "${elf}" 2>/dev/null || true)"
[[ -n "${dynamic}" ]] || fail "missing dynamic section"

if grep -Eq '\((RPATH|RUNPATH)\)' <<<"${dynamic}"; then
    fail "RPATH/RUNPATH must not be present"
fi

allowed_needed()
{
    case "$1" in
        ld-linux-armhf.so.*|libc.so.*|libcrypto.so.*|libdl.so.*|libgcc_s.so.*|libm.so.*|libpthread.so.*|librt.so.*|libssl.so.*|libstdc++.so.*|libturbojpeg.so.*|libz.so.*)
            return 0
            ;;
    esac

    for extra in ${CAM_SYSTEM_VERIFY_ALLOW_EXTRA_NEEDED:-}; do
        [[ "$1" == "${extra}" ]] && return 0
    done

    return 1
}

needed="$("${readelf_cmd}" -d "${elf}" | sed -n 's/.*Shared library: \[\(.*\)\]/\1/p')"
[[ -n "${needed}" ]] || fail "no NEEDED entries found"

while IFS= read -r lib; do
    [[ -n "${lib}" ]] || continue
    allowed_needed "${lib}" || fail "unexpected NEEDED library: ${lib}"
done <<<"${needed}"

echo "verify_elf: OK ${elf}"
