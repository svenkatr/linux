   zreladdr-y	:= 0x60008000
params_phys-y	:= 0x60000100
initrd_phys-y	:= 0x60800000

dtb-$(CONFIG_ARCH_VEXPRESS_CA9X4) += vexpress.dtb
