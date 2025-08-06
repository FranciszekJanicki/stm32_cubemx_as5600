include make/common.mk

.PHONY: clang_tidy
clang_tidy:
	"$(SCRIPTS_DIR)/linters.sh" tidy "$(MAIN_DIR)" "$(COMPONENTS_DIR)"

.PHONY: clang_format
clang_format:
	"$(SCRIPTS_DIR)/linters.sh" format "$(MAIN_DIR)" "$(COMPONENTS_DIR)"

.PHONY: lint
lint: clang_tidy clang_format
