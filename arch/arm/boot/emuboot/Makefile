# Build an image for emulation.

AS		= $(CROSS_COMPILE)gcc -c
LD		= $(CROSS_COMPILE)ld
OBJCOPY         = $(CROSS_COMPILE)objcopy

DTC = ../../../../scripts/dtc/dtc
DTS = ../dts/axm55xx.dts
IMAGE = ../zImage

all: clean linux.img

clean:
	rm -f linux.img emuboot.o image.emu axm55xx.dtb

linux.img: emuboot.o emuboot.lds image.emu
	$(LD) -o linux.tmp --script=emuboot.lds
	$(OBJCOPY) -O binary -R .note -R .comment -S linux.tmp $@
	rm -f linux.tmp

image.emu: $(ZIMAGE) axm55xx.dtb
	python pack.py $(IMAGE) axm55xx.dtb > $@

axm55xx.dtb: $(DTS)
	$(DTC) -O dtb -o $@ $<

emuboot.o: emuboot.S
	$(AS) -o $@ $<
