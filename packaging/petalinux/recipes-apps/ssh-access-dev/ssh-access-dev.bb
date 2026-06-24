SUMMARY = "Development SSH access policy for local lab debugging"
DESCRIPTION = "Adds a key-only debug user for local development images."
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = " \
    file://authorized_keys \
"

S = "${WORKDIR}"

inherit useradd

USERADD_PACKAGES = "${PN}"
GROUPADD_PARAM:${PN} = "-g 1001 debug"
USERADD_PARAM:${PN} = "-u 1001 -g debug -d /home/debug -s /bin/sh debug"

RDEPENDS:${PN} += "dropbear"

do_install() {
    if [ ! -s ${WORKDIR}/authorized_keys ]; then
        bbfatal "ssh-access-dev requires a non-empty authorized_keys file supplied outside the public repository"
    fi

    install -d -m 0755 ${D}/home
    install -d -m 0700 ${D}/home/debug/.ssh
    install -m 0600 ${WORKDIR}/authorized_keys ${D}/home/debug/.ssh/authorized_keys
    chown -R 1001:1001 ${D}/home/debug
}

FILES:${PN} += " \
    /home/debug \
    /home/debug/.ssh \
    /home/debug/.ssh/authorized_keys \
"
