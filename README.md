# Rust OS

a basic os written in rust, and is intended to be compatible with linux in terms of ABI's.

## Features

* [X] booting
* [X] disk driver
* [X] ext2 fs
* [X] nic driver
* [ ] tcp/ip
* [ ] riscv
* [ ] smp

## Building

for the first time of running this project you will need to run the bootstrap script, If you already done that, you can skip this step:

```bash
./scripts/bootstrap.sh
```

Now, you can run the project by:

```
cargo run
```
