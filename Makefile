include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=fbled
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/fbled
    SUBMENU:= Other Modules
    TITLE:=   FBLED device
    FILES:= $(PKG_BUILD_DIR)/fbled-gpio.ko \
		    $(PKG_BUILD_DIR)/fbled-gpio-custom.ko \
			$(PKG_BUILD_DIR)/fbled-algo-bit.ko
    KCONFIG:=
    AUTOLOAD:=$(call AutoLoad,30,fbled)
endef

define KernelPackage/fbled/description
	Kernel module for register a custom fbled platform device
endef

EXTRA_CFLAGS := -DSIMCOM_NETWORK_MANAGER

MAKE_OPTS:= \
	ARCH="$(LINUX_KARCH)" \
	CROSS_COMPILE="$(TARGET_CROSS)" \
	SUBDIRS="$(PKG_BUILD_DIR)" \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	$(EXTRA_KCONFIG)

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		$(MAKE_OPTS) \
        modules
endef

$(eval $(call KernelPackage,fbled))
