# Build an image for Fast Models

AS		= $(CROSS_COMPILE)gcc -c
LD		= $(CROSS_COMPILE)ld

DTC = $(obj)/../../../../scripts/dtc/dtc
DTS = $(obj)/../dts/axm5504-sim-mmc.dts
ZIMAGE = $(obj)/../zImage

clean:
	rm -f $(obj)/../linux.img $(obj)/fmboot.o $(obj)/zImage.fm $(obj)/axm5504-sim.dtb

arch/arm/boot/linux.img: $(obj)/fmboot.o $(srctree)/$(obj)/fmboot.lds $(obj)/zImage.fm
	cd $(obj) && $(LD) -o ../linux.img --script=$(srctree)/$(obj)/fmboot.lds
	tar jcf $(obj)/../linux.img.tar.bz2 $(obj)/../linux.img

$(obj)/zImage.fm: $(ZIMAGE) $(obj)/axm5504-sim.dtb
	python $(srctree)/$(obj)/pack.py $(ZIMAGE) $(obj)/axm5504-sim.dtb > $@

$(obj)/axm5504-sim.dtb: $(DTS)
	$(DTC) -O dtb -o $@ $<

$(obj)/fmboot.o: $(obj)/fmboot.S
	$(AS) -o $@ $<
