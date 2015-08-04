#!/bin/bash
# Just a bunch of stuff useful for kernel development.
# Feel free to tweak to suit your needs/environment.
# It can be sourced directly to the interactive shell.

# Fail on errors
set -e

export ARCH=arm
export DT=imx28-evk
export LOCALVERSION=-imxv5-x0.1

# You may want to change this if toolchain is not in $PATH
export CROSS_COMPILE=arm-none-linux-gnueabi-

# This address is valid for i.MX23/28 SoCs
export LOADADDR=0x42008000

kbuild() {
	# It's not rebuilt when .dtsi's are changed, so just for sure...
	rm arch/arm/boot/dts/${DT}.dtb
	make -j 4 zImage ${DT}.dtb modules
}

kbuild_modules_install() {
	[ -d "$INSTALL_MOD_PATH" ] || {
		echo "INSTALL_MOD_PATH unset, don't installing modules"
		return 1
	}
	rm -rf $INSTALL_MOD_PATH/lib/modules/*
	export INSTALL_MOD_PATH
	make -j 4 modules_install
}
