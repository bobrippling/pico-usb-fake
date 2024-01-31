#!/bin/sh

set -e

if test $# -ne 0; then
	echo >&2 "Usage: $0"
	exit 2
fi

cargo build
elf2uf2-rs target/thumbv6m-none-eabi/debug/pico-usb-fake pico-usb-fake.uf2 >/dev/null
