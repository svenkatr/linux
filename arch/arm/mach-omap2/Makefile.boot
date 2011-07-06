  zreladdr-y		:= 0x80008000
params_phys-y		:= 0x80000100
initrd_phys-y		:= 0x80800000

dtb-$(CONFIG_MACH_OMAP3_BEAGLE) += omap3-beagle.dtb
dtb-$(CONFIG_MACH_OMAP4_PANDA) += omap4-panda.dtb
dtb-$(CONFIG_MACH_OVERO) += omap3-overo.dtb
dtb-$(CONFIG_MACH_IGEP0020) += isee-igep-v2.dtb
dtb-$(CONFIG_MACH_IGEP0030) += isee-igep-v3.dtb
