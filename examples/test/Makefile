TESTBENCH=test
CSOURCE=testapp

GHDL?=ghdl
SDCC?=sdcc
SDCC_CFLAGS=-mpdk14 --opt-code-size --fomit-frame-pointer --fverbose-asm --i-code-in-asm

HDL=../../hdl


COREHDL= $(HDL)/pdk14pkg.vhd \
	$(HDL)/pdkclock.vhd \
	$(HDL)/pdkport.vhd \
	$(HDL)/pdkscaler.vhd \
	$(HDL)/pdktim.vhd \
	$(HDL)/pdkpwmg.vhd \
	$(HDL)/pdkreg.vhd \
	$(HDL)/txt_util.vhd \
	$(HDL)/measperiod.vhd \
	$(HDL)/pdk14decode.vhd \
	$(HDL)/pdk14.vhd \
	$(HDL)/pdkpkg.vhd

all: $(TESTBENCH)

SOURCES=$(COREHDL) $(TESTBENCH).vhd pdkrom.vhd

$(TESTBENCH): $(SOURCES)
	$(GHDL) -i $(SOURCES)
	$(GHDL) -m --ieee=synopsys -fexplicit $(TESTBENCH)

pdkrom.vhd: $(CSOURCE).bin
	../../tools/bin2rom -t $(HDL)/pdkrom.vhd.in -i $(CSOURCE).bin -o pdkrom.vhd

$(CSOURCE).bin: $(CSOURCE).ihex
	objcopy -I ihex -O binary $+ $@

$(CSOURCE).ihex: $(CSOURCE).c
	$(SDCC) $(SDCC_CFLAGS) --out-fmt-ihx $+ -o $@

