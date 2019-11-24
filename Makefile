.SUFFIXES: .bin .ino

PLATFORM = photon

OBJS = basic-modbus.bin modbus-sensor.bin
SRCPATH = examples/usage

all: $(OBJS)

%.bin: $(SRCPATH)/%.ino
	particle compile $(PLATFORM) $<


