use alloc::string::String;

use crate::drivers::ide::IDEDisk;

struct Inode {
	ino: u32,

}

static mut ext2_block_size: u8 = 1 << 10;

fn read_block(disk: IDEDisk) {
	
}

fn mount(disk: IDEDisk, mnt_point: &String) {
	
}

fn read_inode(ino: u32) -> Inode {

}