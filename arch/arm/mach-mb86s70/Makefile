#
# Makefile for the linux kernel.
#
ccflags-$(CONFIG_ARCH_MULTIPLATFORM) := -I$(srctree)/$(src)/include

obj-y := sleep.o common.o devices.o headsmp.o scb_mhu.o mcpm.o mb86s70_pm_setup.o nonsec.o
obj-$(CONFIG_PM_GENERIC_DOMAINS) += pm_domains.o
CFLAGS_REMOVE_mcpm.o	= -pg
