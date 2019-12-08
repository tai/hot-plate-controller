
TOPDIR = .
TC = /opt/microchip/xc16/v1.35

CPU = 30F2010

# FCY = FOSC / 4 (FRM 7.2, Equation 7-1) = 8MHz@FRC / 4 = 2MHz
FCY = 2000000

CC = $(TC)/bin/xc16-gcc -mcpu=$(CPU)
LD = $(CC)
BIN2HEX = $(TC)/bin/xc16-bin2hex

LDSCRIPT = p$(CPU).gld

CDEFS  = -I$(TOPDIR)/src -DFCY=$(FCY)ULL

#
# NOTE:
# For xc-gcc compiler, -0s or -O2+ requires a license purchase.
# However, see below:
# - https://www.microchip.com/forums/m713820.aspx
# - https://www.eevblog.com/forum/microcontrollers/is-microchip-violating-the-gpl/
# - https://hackaday.io/project/27250-mcu-how-tos-reviews-rants/log/72734-exploring-microchip-xc16-compiler
#
# > So, entering -mafrlcsj option into command line should be
# > equal to having proper license.
#
CFLAGS = -Wall -g3 -std=c99 -omf=elf \
	-fno-short-double -ffunction-sections -fdata-sections \
	-msmart-io=1 -msfr-warn=off \
	-mafrlcsj -Os

LDFLAGS = -Wl,--data-init -Wl,--heap=0 -Wl,--gc-sections

TOP_SRC = $(wildcard $(TOPDIR)/src/*.c)

SRCS = $(notdir $(TOP_SRC))
OBJS = $(SRCS:.c=.p1)

VPATH = $(dir $(TOP_SRC))

TARGET = firmware.elf
HEX = $(TARGET:.elf=.hex)

.SUFFIXES:
.SUFFIXES: .elf .hex .p1 .c .h

.c.p1:
	$(CC) -c $(CFLAGS) $(CDEFS) -o $@ $<

.elf.hex:
	$(BIN2HEX) $<

all:
	@echo "Type: make (build|upload|clean)"

$(TARGET): $(OBJS)
	$(LD) -Wl,-Map=$*.map -Wl,--memorysummary,$*.xml \
	-T $(LDSCRIPT) $(LDFLAGS) -o $@ $(OBJS)

.PHONY: build
build:
	mkdir -p build
	$(MAKE) -C build -f ../Makefile TOPDIR=.. $(HEX)

upload: build
	pk2cmd -P -T -R -M -F build/$(TARGET:.elf=.hex)

upload-mplab:
	pk2cmd -P -T -R -M -F dist/*/*/*.hex

reset:
	pk2cmd -P -T -R

clean:
	$(RM) *.d *.i *.p1 *.old *.bak *~
	$(RM) -r build
