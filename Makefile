#
# Makefile for RC700 simulator
#

all: rc700

clean:
	rm rc700 bootrom

SRCFILES=cpu0.c cpu1.c cpu2.c cpu3.c cpu4.c cpu5.c cpu6.c cpu7.c  \
         rom.c charrom.c charram.c pio.c sio.c ctc.c dma.c crt.c fdc.c wdc.c ftp.c disk.c fifo.c \
         monitor.c disasm.c

HDRFILES=cpu.h rc700.h disk.h bootrom

#AUTOLOAD=roa375.rom
AUTOLOAD=rob358.rom

rc700: $(SRCFILES) $(HDRFILES) rc700.c rcterm-sdl.c bootrom
	$(CC) -o $@ -O3 -Wno-unused-result $(SRCFILES) rc700.c rcterm-sdl.c -lSDL

rc700-curses: $(SRCFILES) $(HDRFILES) rc700.c rcterm-curses.c bootrom
	$(CC) -o $@ -O3 -Wno-unused-result $(SRCFILES) rc700.c rcterm-curses.c -lncursesw

rc700d: $(SRCFILES) $(HDRFILES) rc700d.c websock.c sha1.c sha1.h bootrom
	$(CC) -m32 -o $@ -O3 -Wno-unused-result $(SRCFILES) rc700d.c websock.c sha1.c

rom2struct: rom2struct.c
	$(CC) -o $@ rom2struct.c

bootrom: rom2struct $(AUTOLOAD)
	./rom2struct $(AUTOLOAD) > bootrom

ana2imd: ana2imd.c disk.c disk.h sim.h
	$(CC) -o $@ ana2imd.c disk.c

imd2bin: imd2bin.c disk.c disk.h
	$(CC) -o $@ imd2bin.c disk.c

rom2hex: rom2hex.c
	$(CC) -o $@ rom2hex.c

cpmdisk: cpmdisk.c disk.c disk.h
	$(CC) -o $@ cpmdisk.c disk.c

comaldisk: cpmdisk.c disk.c disk.h
	$(CC) -o $@ cpmdisk.c disk.c -DCOMAL
	
blankimd: blankimd.c disk.c disk.h
	$(CC) -o $@ blankimd.c disk.c

rxtext: rtext.c
	$(CC) -o $@ rctext.c

char2raw: char2raw.c charrom.c charram.c
	$(CC) -o $@ char2raw.c charrom.c charram.c

