#!/bin/sh
set -e

if [ $# -gt 2 -o $1 = "--help" ]; then
    echo "usage: $0 <elf-name>" >&2
    exit 1
fi

TARGET="$(realpath --relative-to="$(pwd)" "$(dirname "$0")/target")"
HEX="$TARGET/$1.hex"
ELF="$(echo "$TARGET"/avr-*/"$BUILD/$1.elf")"

if [ ! -e "$HEX" ]; then
    echo "No $1.hex found.  The following binaries exist:" >&2
    for bin in "$target_dir/"*.hex; do
        echo "  - $(basename -s.hex "$bin")" >&2
    done
    exit 1
fi

/home/ggreen/apps/arduino-1.8.13/hardware/tools/avr/bin/avrdude -C/home/ggreen/apps/arduino-1.8.13/hardware/tools/avr/etc/avrdude.conf -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -U flash:w:"$HEX"
