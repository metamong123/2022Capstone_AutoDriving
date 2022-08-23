XSENS_INSTALL_PREFIX?=$(shell dirname $$(dirname $$(pwd))))

TARGETS:=dll_cpp
CLEAN_TARGETS=$(patsubst %,%,$(TARGETS))

.PHONY: all $(TARGETS) clean
all: $(TARGETS)

$(TARGETS):
	$(MAKE) -C $@

clean:
	-$(MAKE) -C $(CLEAN_TARGETS) clean
