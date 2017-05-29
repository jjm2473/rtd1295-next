obj-$(CONFIG_USB_F_USB20HDC_OTG)	+= f_usb20hdc_otg.o
f_usb20hdc_otg-y			:= f_usb20hdc_phy.o

ifneq ($(filter y,$(CONFIG_USB_F_USB20HDC_OTG_HOST_ONLY) $(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)),)
	f_usb20hdc_otg-y		+= f_usb20hdc-hcd.o
endif

ifneq ($(filter y,$(CONFIG_USB_F_USB20HDC_OTG_GADGET_ONLY) $(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)),)
	f_usb20hdc_otg-y		+= f_usb20hdc_udc.o
endif
