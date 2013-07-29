# MXS Platform Support
snd-soc-mxs-objs := mxs-saif.o
snd-soc-mxs-pcm-objs := mxs-pcm.o

obj-$(CONFIG_SND_MXS_SOC) += snd-soc-mxs.o snd-soc-mxs-pcm.o

# i.MX Machine Support
snd-soc-mxs-sgtl5000-objs := mxs-sgtl5000.o

obj-$(CONFIG_SND_SOC_MXS_SGTL5000) += snd-soc-mxs-sgtl5000.o

# i.MX23 built-in audio Machine and Platform support
snd-soc-mxs-builtin-pcm-objs := mxs-builtin-pcm.o
snd-soc-mxs-builtin-dai-objs := mxs-builtin-dai.o
snd-soc-mxs-builtin-audio-objs := mxs-builtin-audio.o

obj-$(CONFIG_SND_MXS_SOC_BUILTIN) += snd-soc-mxs-builtin-pcm.o
obj-$(CONFIG_SND_MXS_SOC_BUILTIN) += snd-soc-mxs-builtin-dai.o
obj-$(CONFIG_SND_MXS_SOC_BUILTIN) += snd-soc-mxs-builtin-audio.o
