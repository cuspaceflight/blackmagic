ifneq ($(V), 1)
MFLAGS += --no-print-dir
Q := @
endif

all:
	$(Q)if [ ! -f libopencm3/Makefile ]; then \
		echo "Initialising git submodules..." ;\
		git submodule init ;\
		git submodule update ;\
	fi
	$(Q)$(MAKE) $(MFLAGS) -C libopencm3 lib
	$(Q)$(MAKE) $(MFLAGS) -C src

clean:
	$(Q)$(MAKE) $(MFLAGS) -C libopencm3 $@
	$(Q)$(MAKE) $(MFLAGS) -C src $@


##############################################################################
# Black Magic Probe flashing via GDB
#
flash: src/blackmagic
	arm-none-eabi-gdb --batch \
		              -ex 'target extended-remote /dev/ttyACM0' \
					  -ex 'monitor version' \
					  -ex 'monitor swdp_scan' \
					  -ex 'attach 1' \
					  -ex 'load src/blackmagic'

debug: src/blackmagic
	arm-none-eabi-gdb -ex 'target extended-remote /dev/ttyACM0' \
		              -ex 'monitor swdp_scan' \
					  -ex 'attach 1' \
					  -ex "file src/blackmagic"
power:
	arm-none-eabi-gdb --batch \
	                  -ex 'target extended-remote /dev/ttyACM0' \
		              -ex 'monitor tpwr enable'
unpower:
	arm-none-eabi-gdb --batch \
		              -ex 'target extended-remote /dev/ttyACM0' \
		              -ex 'monitor tpwr disable'

#
# End BMP flashing
##############################################################################
