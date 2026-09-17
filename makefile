TARGET = mView
PSPSDK=$(shell psp-config --pspsdk-path)
PSPBIN = $(PSPSDK)/../bin

PSP_EBOOT_PIC1 = PIC1.PNG
PSP_EBOOT_ICON = ICON0.PNG

# Build mode: user (PSP-290/USB GPS, user-mode PRX, runs under PPSSPP)
#          or kernel (HOLUX GPSlim236+, kernel-mode module, needs real PSP + CFW)
# Use "make user" / "make kernel" instead of editing this file.
MODE ?= user

###################################################################
ifeq ($(MODE),user)
###################################################################
#PSP-290/USB version - USER MODE - runs in PPSSPP
###################################################################
PSP_FW_VERSION=371
BUILD_PRX = 1
CFLAGS = -O2  -G0 -Wall -g -DDANZEFF_SCEGU -DNDEBUG -fcommon
LIBS =  -lpspdebug  -lpsprtc -lpspgum -lpspgu  -lpsppower  -lpspusb -lpng -lz  -ljpeg -lm -lc -lpspwlan -lmad -lpspaudiolib -lpspaudio -g

###################################################################
else ifeq ($(MODE),kernel)
###################################################################
#HOLUX GPSlim236+ version - KERNEL MODE - real PSP + CFW only
###################################################################
CFLAGS = -O2  -G0 -Wall -g -DDANZEFF_SCEGU -DNDEBUG -DGENERIC -fcommon
LIBS =  -lpspdebug  -lpsphprm_driver  -lpsprtc   -lpspvfpu -lpspgum   -lpsppower   -lpng -lz  -ljpeg -lm -lpspwlan -lmad -lpspaudiolib -lpspaudio -lpspgu -lpspkernel

###################################################################
else
###################################################################
$(error Unknown MODE '$(MODE)'. Use "make user" or "make kernel")
endif


OBJS =  main.o \
        graphics.o \
	font.o \
	utils.o \
	attractions.o \
	line.o \
	nmeap01.o \
	danzeff.o \
	geocalc.o \
	sceUsbGps.o \
	geodata.o \
	mp3player.o \
	basic.o \
	menu.o \
	sioprx.o \
	display.o \
	mapsforge.o



CXXFLAGS = $(CFLAGS) -fno-exceptions -fno-rtti
ASFLAGS = $(CFLAGS)


EXTRA_TARGETS = EBOOT.PBP
PSP_EBOOT_TITLE = mView

.PHONY: user kernel

# Build objects from a previous MODE are incompatible (different CFLAGS/ABI),
# so switching modes forces a clean rebuild automatically.
user:
	@if [ -f .buildmode ] && [ "$$(cat .buildmode)" != "user" ]; then \
		echo "Switching build mode: $$(cat .buildmode) -> user, cleaning..."; \
		$(MAKE) clean; \
	fi
	@echo user > .buildmode
	$(MAKE) MODE=user all

kernel:
	@if [ -f .buildmode ] && [ "$$(cat .buildmode)" != "kernel" ]; then \
		echo "Switching build mode: $$(cat .buildmode) -> kernel, cleaning..."; \
		$(MAKE) clean; \
	fi
	@echo kernel > .buildmode
	$(MAKE) MODE=kernel all

include $(PSPSDK)/lib/build.mak
