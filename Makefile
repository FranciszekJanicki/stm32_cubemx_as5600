include make/build.mk
include make/linters.mk
include make/cubemx.mk
include make/target.mk
include make/components.mk
include make/third_party.mk
include make/tools.mk
include make/scripts.mk

.DEFAULT_GOAL := build

.PHONY: setup
setup:
	$(MAKE) setup_scripts
	$(MAKE) setup_components
	$(MAKE) setup_third_party
	$(MAKE) setup_cmake

.PHONY: all
all:
	$(MAKE) lint
	$(MAKE) build
	$(MAKE) flash_uart
	$(MAKE) monitor_uart
