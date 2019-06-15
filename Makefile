BUILD_DIR := $(shell ls -d *_core)

all:
	$(foreach bdir,$(BUILD_DIR),$(MAKE) -C $(bdir);)

install_nfsdir:
	$(foreach bdir,$(BUILD_DIR),$(MAKE) -C $(bdir) install;)

clean:
	$(foreach dir, $(BUILD_DIR), $(MAKE) -C $(dir) clean;)

