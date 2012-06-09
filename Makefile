PORT=/dev/ttyUSB0
BOARD = atmega328
# BOARD = atmega168

# GLFAG = 
OFLAG = -Os
# AVRDUDECONFIG = -D -C /etc/avrdude.conf
SKETCH        = MultiWii
SKETCHBOOK    = .
#ARDUINO       = /usr/share/arduino
#HARDWARE_DIR  = $(SKETCHBOOK)/arduino
BUILDROOT     = ./build

include Arduino.mk
