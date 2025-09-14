# Makefile for Dreidel M5 ESP32 Project
# Provides convenient targets for building and flashing

# Default serial port - override with: make flash PORT=/dev/ttyUSB0
PORT ?= COM4
BAUD ?= 115200

# Default target
.PHONY: all
all: build

# Build the project
.PHONY: build
build:
	idf.py build

# Clean build artifacts
.PHONY: clean
clean:
	idf.py fullclean

# Flash to device (depends on build)
.PHONY: flash
flash: build
	idf.py -p $(PORT) -b $(BAUD) flash

# Flash and monitor (depends on build)
.PHONY: flash-monitor
flash-monitor: build
	idf.py -p $(PORT) -b $(BAUD) flash monitor

# Monitor serial output only
.PHONY: monitor
monitor:
	idf.py -p $(PORT) -b $(BAUD) monitor

# Show build size information
.PHONY: size
size: build
	idf.py size

# Configure project settings
.PHONY: menuconfig
menuconfig:
	idf.py menuconfig

# Help target
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  build         - Build the project"
	@echo "  clean         - Clean all build artifacts"
	@echo "  flash         - Build and flash to device (default port: $(PORT))"
	@echo "  flash-monitor - Build, flash, and start serial monitor"
	@echo "  monitor       - Start serial monitor only"
	@echo "  size          - Show build size information"
	@echo "  menuconfig    - Open ESP-IDF configuration menu"
	@echo "  help          - Show this help message"
	@echo ""
	@echo "Override default port with: make flash PORT=COM3"
	@echo "Override baud rate with: make flash BAUD=460800"