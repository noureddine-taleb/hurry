#![no_std]
#![no_main]
#![feature(custom_test_frameworks)]
#![test_runner(blog_os::test_runner)]
#![reexport_test_harness_main = "test_main"]

use blog_os::{println, drivers::ide::{IDEDisk, IDE_PRIMARY_CHANNEL_BUS}};
use core::panic::PanicInfo;

#[no_mangle] // don't mangle the name of this function
pub extern "C" fn _start() -> ! {
    test_main();

    loop {}
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    blog_os::test_panic_handler(info)
}

#[test_case]
fn test_read_write_sector() {
    let mut disk = unsafe {IDEDisk::new(IDE_PRIMARY_CHANNEL_BUS)};
    unsafe { disk.init().unwrap() };
    
    let mut mbr: [u8; 512] = [0; 512];
	// read MBR
    unsafe { disk.read_sector(0, &mut mbr).unwrap() };
    println!("original ide sector read: {:#x?}", &mbr[510..512]);
    assert_eq!(&mbr[510..512], [0x55, 0xaa]);

    mbr[510] = 0x88;
    mbr[511] = 0x99;
    unsafe { disk.write_sector(0, &mbr).unwrap() };
    mbr = [0; 512]; // init buffer
    unsafe { disk.read_sector(0, &mut mbr).unwrap() };
    assert_eq!(&mbr[510..512], &[0x88, 0x99]);
}
