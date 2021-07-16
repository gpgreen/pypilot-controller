#!/bin/sh
set -e

if [ $# -gt 2 -o $1 = "--help" ]; then
    echo "usage: $0 [--release|--debug] <elf-name>" >&2
    exit 1
fi

BUILD="debug"
if [ "$1" = "--debug" ]; then
    shift 1
    BUILD="debug"
elif [ "$1" = "--release" ]; then
    shift 1
    BUILD="release"
fi

TARGET="$(realpath --relative-to="$(pwd)" "$(dirname "$0")/target")"
ELF="$(echo "$TARGET"/avr-*/"$BUILD/$1.elf")"
LSS="${ELF%elf}"lss

if [ ! -e "$ELF" ]; then
    echo "No $1.elf found.  The following binaries exist:" >&2
    for target_dir in "$TARGET"/avr-*; do
        for bin in "$target_dir/$BUILD"/*.elf; do
            echo "  - $(basename -s.elf "$bin")" >&2
        done
    done
    exit 1
fi

avr-objdump -h -S "$ELF" > "$LSS"
