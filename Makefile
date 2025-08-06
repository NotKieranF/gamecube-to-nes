PROJECT = gamecube_to_nes
# Final target is the attiny202, current prototype is running on an atmega4809 devboard
AVR_ARCH = atmega4809
ATPACK_DIR = vendor/atpack-atmega
SRC_FILES += $(wildcard src/*.cpp) $(wildcard src/*.c) $(wildcard src/*.s) $(wildcard src/*.S)
DEP_FILES = $(foreach file,$(SRC_FILES),build/$(basename $(notdir $(file))).d)
OBJ_FILES = $(foreach file,$(SRC_FILES),build/$(basename $(notdir $(file))).o)

CC = avr-gcc
CXX = avr-g++
OBJCOPY = avr-objcopy
CFLAGS += -MMD -Os -g -Wall -flto -mmcu=$(AVR_ARCH) -B $(ATPACK_DIR)/gcc/dev/$(AVR_ARCH) -isystem $(ATPACK_DIR)/include
CXXFLAGS += -MMD -Os -g -Wall -flto -mmcu=$(AVR_ARCH) -B $(ATPACK_DIR)/gcc/dev/$(AVR_ARCH) -isystem $(ATPACK_DIR)/include
LDFLAGS += -g -mmcu=$(AVR_ARCH) -B $(ATPACK_DIR)/gcc/dev/$(AVR_ARCH)

build/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

build/%.o: src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

build/%.o: src/%.s
	$(CC) $(CFLAGS) -c $< -o $@

build/%.o: src/%.S
	$(CC) $(CFLAGS) -c $< -o $@

build/%.hex: build/%.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

build/$(PROJECT).elf: dir $(OBJ_FILES)
	$(CXX) $(LDFLAGS) -o build/$(PROJECT).elf $(OBJ_FILES)

.PHONY: upload
upload: build/$(PROJECT).hex
	python3 1200baud.py /dev/ttyACM0
	avrdude -v -V -p $(AVR_ARCH) -c jtag2updi -P /dev/ttyACM0 -b 115200 -e -D -U flash:w:build/$(PROJECT).hex:i

.PHONY: clean
clean:
	rm -rf build

.PHONY: dir
dir:
	mkdir -p build/

-include $(DEP_FILES)