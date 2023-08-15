#![no_std]
#![no_main]
#![feature(custom_test_frameworks)]
#![test_runner(blog_os::test_runner)]
#![reexport_test_harness_main = "test_main"]

extern crate alloc;

use alloc::string::String;
use blog_os::drivers::e1000::E1000;
use blog_os::ext2::Ext2Fs;
use blog_os::{println, phys_offset};
use blog_os::task::{executor::Executor, keyboard, Task};
use bootloader::{entry_point, BootInfo};
use core::panic::PanicInfo;
use blog_os::drivers::ide::{IDEDisk, IDE_PRIMARY_CHANNEL_BUS, IDE_SECONDARY_CHANNEL_BUS};
use blog_os::drivers::pci::pci_init;
use alloc::vec::Vec;

entry_point!(kernel_main);

fn kernel_main(boot_info: &'static BootInfo) -> ! {
    use blog_os::allocator;
    use blog_os::memory::{self, BootInfoFrameAllocator};
    use x86_64::VirtAddr;

    println!("Hello World{}", "!");
    blog_os::init();

    let phys_mem_offset = VirtAddr::new(boot_info.physical_memory_offset);
    unsafe {phys_offset = phys_mem_offset.as_u64() as usize};
    let mut mapper = unsafe { memory::init(phys_mem_offset) };
    let mut frame_allocator = unsafe { BootInfoFrameAllocator::init(&boot_info.memory_map) };

    allocator::init_heap(&mut mapper, &mut frame_allocator).expect("heap initialization failed");

    // let mut disk = unsafe {IDEDisk::new(IDE_PRIMARY_CHANNEL_BUS)};
    let mut disk = unsafe {IDEDisk::new(IDE_SECONDARY_CHANNEL_BUS).unwrap()};
    
    let mut rootfs = Ext2Fs::new(&mut disk).unwrap();
    
    println!("-------------------");
    let inode = rootfs.path_to_inode(String::from("/dir/file51")).unwrap();
    println!("inode of /file: {:?}", inode);
    println!("inode content: {:?}", inode.file_get_content(&mut disk).unwrap().iter().map(|c| *c as char).collect::<Vec<_>>());

    // let mut mbr: [u8; 512] = [0; 512];
	// // read MBR
    // unsafe { disk.read_sector(0, &mut mbr).unwrap() };
    // println!("original ide sector read: {:#x?}", &mbr[510..512]);

    // mbr[510] = 0x88;
    // mbr[511] = 0x99;
    // unsafe { disk.write_sector(0, &mbr).unwrap() };
    // mbr = [0; 512]; // init buffer
    // unsafe { disk.read_sector(0, &mut mbr).unwrap() };
    // assert_eq!(&mbr[510..512], &[0x88, 0x99]);

    pci_init(&mut mapper, &mut frame_allocator);

    #[cfg(test)]
    test_main();

    let mut executor = Executor::new();
    executor.spawn(Task::new(example_task()));
    executor.spawn(Task::new(keyboard::print_keypresses()));
    executor.run();
}

/// This function is called on panic.
#[cfg(not(test))]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    println!("{}", info);
    blog_os::hlt_loop();
}

#[cfg(test)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    blog_os::test_panic_handler(info)
}

async fn async_number() -> u32 {
    42
}

async fn example_task() {
    let number = async_number().await;
    println!("async number: {}", number);
}

#[test_case]
fn trivial_assertion() {
    assert_eq!(1, 1);
}
