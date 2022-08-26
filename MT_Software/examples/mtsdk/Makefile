XSENS_INSTALL_PREFIX?=$(shell dirname $$(dirname $$(pwd)))
XSENS_PUBLIC_SOURCE?=$(XSENS_INSTALL_PREFIX)/public

COMMON_TARGETS=xda_public_cpp xda_cpp
TARGETS=$(COMMON_TARGETS)
ifneq ($(wildcard awindamonitor_cpp/.*),)
	TARGETS+=awindamonitor_cpp
endif
CLEAN_COMMON_TARGETS=$(patsubst %,%.clean,$(COMMON_TARGETS))
CLEAN_TARGETS=$(patsubst %,%.clean,$(TARGETS))

.PHONY: all $(TARGETS) xda_public_cpp_prereqs clean
all: $(TARGETS)

awindamonitor_cpp/Makefile:
	(cd awindamonitor_cpp; qmake "XSENS_INSTALL_PREFIX=$(XSENS_INSTALL_PREFIX)")

awindamonitor_cpp: awindamonitor_cpp/Makefile
xda_public_cpp: xda_public_cpp_prereqs
$(TARGETS):
	$(MAKE) -C $@

$(CLEAN_COMMON_TARGETS):
	-$(MAKE) -C $(patsubst %.clean,%,$@) clean

awindamonitor_cpp.clean:
	-$(MAKE) -C $(patsubst %.clean,%,$@) distclean

clean: $(CLEAN_TARGETS)
