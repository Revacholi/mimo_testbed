#
# This file is the zmoddac recipe.
#

SUMMARY = "Simple zmoddac application"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"


DEPENDS += "libpthread-stubs"

SRC_URI = " \	
    file://zmodstart.cpp \
    file://zmoddac.cpp \
    file://zmodadc.cpp \
    file://zmodlib/Zmod/zmod.h \
    file://zmodlib/Zmod/zmod.cpp \
	file://zmodlib/Zmod/dma.h \
	file://zmodlib/Zmod/flash.h \
	file://zmodlib/Zmod/reg.h \
    file://zmodlib/Zmod/linux/utils.h \
    file://zmodlib/Zmod/linux/utils.c \
    file://zmodlib/Zmod/linux/dma/dma.c  \
	file://zmodlib/Zmod/linux/dma/libaxidma.c \
    file://zmodlib/Zmod/linux/dma/libaxidma.h \
    file://zmodlib/Zmod/linux/dma/axidma_ioctl.h \
    file://zmodlib/Zmod/linux/flash/flash.c \
    file://zmodlib/Zmod/linux/reg/reg.c \
    file://zmodlib/Zmod/linux/reg/libuio.c \
    file://zmodlib/Zmod/linux/reg/libuio.h \
    file://zmodlib/ZmodDAC1411/zmoddac1411.h \
    file://zmodlib/ZmodDAC1411/zmoddac1411.cpp \
    file://zmodlib/ZmodADC1410/zmodadc1410.h \
    file://zmodlib/ZmodADC1410/zmodadc1410.cpp \
    file://Makefile \
          "
S = "${WORKDIR}"


do_compile() {
        oe_runmake
}

do_install() {
    install -d ${D}${bindir}
    install -m 0755 zmodstart ${D}${bindir}
    install -m 0755 zmoddac ${D}${bindir}
    install -m 0755 zmodadc ${D}${bindir}
}
