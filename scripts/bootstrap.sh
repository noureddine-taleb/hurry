#!/bin/sh

set -xe

function die() {
    echo $@ >&2
    exit 1
}

if ! command -v rustup
then
    curl https://sh.rustup.rs | sh
fi

if ! command -v qemu-system-x86_64
then
    die you need to install qemu for you distribution, and rerun this again !
fi


rustup component add rust-src --toolchain nightly-x86_64-unknown-linux-gnu

rustup component add llvm-tools-preview

cargo install bootimage
