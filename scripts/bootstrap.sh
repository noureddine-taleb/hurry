#!/bin/sh

set -xe

if ! command -v rustup
then
    curl https://sh.rustup.rs | sh
fi

rustup component add rust-src --toolchain nightly-x86_64-unknown-linux-gnu

rustup component add llvm-tools-preview

cargo install bootimage
