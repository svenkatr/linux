#!/bin/bash
. ./scripts/kbuild-env.sh
image=zImage-${DT}

grep 'CONFIG_ARM_APPENDED_DTB=y' .config || {
	echo "CONFIG_ARM_APPENDED_DTB is not set, this makes no sense"
	exit 1
}

kbuild
kbuild_modules_install || true

cat arch/${ARCH}/boot/zImage arch/${ARCH}/boot/dts/${DT}.dtb > ${image}
echo "${image} is ready"
