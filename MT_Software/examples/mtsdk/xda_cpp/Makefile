XSENS_INSTALL_PREFIX?=$(shell dirname $$(dirname $$(dirname $$(pwd))))
BASIC_CFLAGS := -g -Wall -Wextra
INCLUDE:=-I$(XSENS_INSTALL_PREFIX)/include -I$(XSENS_INSTALL_PREFIX)/include/xsensdeviceapi
LFLAGS:=-lm -lxsensdeviceapi -lxstypes -lpthread -L$(XSENS_INSTALL_PREFIX)/lib -Wl,-rpath,$(XSENS_INSTALL_PREFIX)/lib

CFLAGS:=$(BASIC_CFLAGS) $(INCLUDE) $(CFLAGS)
CXXFLAGS:=$(BASIC_CFLAGS) -std=c++11 $(INCLUDE) $(CXXFLAGS)

TARGETS:=example_mti_receive_data example_mti_parse_logfile example_mtw
all: $(TARGETS)

example_mti_receive_data: example_mti_receive_data.cpp
example_mti_parse_logfile: example_mti_parse_logfile.cpp
example_mtw: example_mtw.cpp conio.c.o
$(TARGETS):
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LFLAGS)

-include $(FILES:.cpp=.dpp)
%.cpp.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $< -o $@
	@$(CXX) -MM $(CXXFLAGS) $< > $*.dpp
	@mv -f $*.dpp $*.dpp.tmp
	@sed -e 's|.*:|$*.cpp.o:|' < $*.dpp.tmp > $*.dpp
	@sed -e 's/.*://' -e 's/\\$$//' < $*.dpp.tmp | fmt -1 | \
	  sed -e 's/^ *//' -e 's/$$/:/' >> $*.dpp
	@rm -f $*.dpp.tmp

-include $(FILES:.c=.d)
%.c.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@
	@$(CC) -MM $(CFLAGS) $< > $*.d
	@mv -f $*.d $*.d.tmp
	@sed -e 's|.*:|$*.c.o:|' < $*.d.tmp > $*.d
	@sed -e 's/.*://' -e 's/\\$$//' < $*.d.tmp | fmt -1 | \
	  sed -e 's/^ *//' -e 's/$$/:/' >> $*.d
	@rm -f $*.d.tmp

clean:
	-$(RM) *.o *.d *.dpp $(TARGETS) $(PREBUILDARTIFACTS)
