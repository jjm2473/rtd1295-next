#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI)
#

ccflags-y := -Iinclude/drm -Werror

fdb-drm-y := \
	fdb-drv.o \
	fdb-fbdev.o \
	fdb-connector.o \
	fdb-fb.o \
	fdb-gem.o \
	fdb-gem-helpers.o \
	fdb-gem-dmabuf.o \
	fdb-crtc.o \
	fdb-plane.o \
	fdb-encoder.o

obj-$(CONFIG_DRM_FUJITSU_FDB)	+= fdb-drm.o
