do_compile:append () {
	sed -i 's/if test -n $uenvcmd; then/if test -n "\${uenvcmd}"; then/' "${WORKDIR}/boot.cmd"
	if ! grep -Fq 'if test -n "${uenvcmd}"; then' "${WORKDIR}/boot.cmd"; then
		bbfatal "u-boot-zynq-scr: failed to quote uenvcmd test"
	fi
	if grep -Fq 'if test -n $uenvcmd; then' "${WORKDIR}/boot.cmd"; then
		bbfatal "u-boot-zynq-scr: unquoted uenvcmd test remains"
	fi
	mkimage -A arm -T script -C none -n "Boot script" -d "${WORKDIR}/boot.cmd" boot.scr
}
