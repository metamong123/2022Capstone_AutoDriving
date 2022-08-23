XSENS_INSTALL_PREFIX?=$(shell dirname $$(dirname $$(dirname $$(pwd))))
BASIC_CFLAGS := -g -Wall -Wextra
INCLUDE:=-I. -Ixsens -I$(XSENS_INSTALL_PREFIX)/include
LFLAGS:=-lm -lxsensdeviceapi -lmagfieldmapper -lxstypes -lpthread -L$(XSENS_INSTALL_PREFIX)/lib -Wl,-rpath,$(XSENS_INSTALL_PREFIX)/lib

CFLAGS := $(BASIC_CFLAGS) $(INCLUDE) $(CFLAGS) -std=c++11 

%.cpp.o: %.cpp
	$(CXX) $(CFLAGS) -c $^ -o $@

%.c.o: %.c
	$(CC) $(CFLAGS) -c $^ -o $@

TARGETS:=example_mfm_cpp

all: $(TARGETS)

example_mfm_cpp: example_mfm_cpp.cpp.o custom_channel.cpp.o
$(TARGETS):
	$(CXX) $(CFLAGS) $^ -o $@ $(LFLAGS)

clean:
	-$(RM) *.o $(TARGETS)
