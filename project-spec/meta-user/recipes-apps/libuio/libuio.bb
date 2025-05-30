SUMMARY = "Simple libuio application"
SECTION = "libs"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "git://github.com/digilent/libuio.git;protocol=https;branch=master"
SRCREV = "${AUTOREV}"

RDEPENDS_${PN} = "glibc"
DEPENDS = "glibc"

S = "${WORKDIR}/git"

PACKAGE_ARCH = "${MACHINE_ARCH}"
TARGET_CC_ARCH += "${LDFLAGS}"

do_install() {
    install -d ${D}${libdir}
    oe_libinstall -so libuio ${D}${libdir}
    
    install -d ${D}${includedir}
    install -m 0644 ${S}/libuio.h ${D}${includedir}
}

FILES_${PN} += " \
    ${libdir}/libuio.so.* \
    ${includedir}/libuio.h \
"
