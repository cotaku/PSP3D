TARGET = cotaku
OBJS = $(patsubst %.cpp,%.o,$(wildcard *.cpp))
CFLAGS = -O3 -G0 -w
CXXFLAGS = $(CFLAGS) -fno-exceptions -fno-rtti
ASFLAGS = $(CFLAGS)
EXTRA_TARGETS = EBOOT.PBP
LIBS = -lpsplibc -lpsprtc
PSP_EBOOT_TITLE = PSP for cotaku39
PSP_EBOOT_ICON = icon.png
PSPSDK=$(shell psp-config --pspsdk-path)
include $(PSPSDK)/lib/build.mak