// Simple PCI-Express for qemu
use core::slice::from_raw_parts_mut;
use alloc::vec::Vec;
use volatile::Volatile;
use x86_64::{VirtAddr, instructions::port::Port, structures::paging::{Size4KiB, FrameAllocator, Mapper}};

use crate::println;

use super::e1000::E1000;

/// E1000 registers which were mapped
pub const E1000_REGS: u32 = 0x40000000;
/// Qemu virt PCIe config space
pub const ECAM: u32 = 0x30000000;
pub const PCI_ADD_PORT: u16 = 0xCF8;
pub const PCI_DATA_PORT: u16 = 0xCFC;

pub const PCI_COMMAND_STATUS_REG: u32 = 0x4;
pub const PCI_HEADER_TYPE_REG: u32 = 0xC;
pub const PCI_BAR_START_REG: u32 = 0x10;
pub const PCI_BAR_END_REG: u32 = 0x28;
pub const PCI_INTERRUPT_LINE_REG: u32 = 0x3C;

pub const PCI_COMMAND_IO_ENABLE: u32 = 0x1;
pub const PCI_COMMAND_MEM_ENABLE: u32 = 0x2;
pub const PCI_COMMAND_MASTER_ENABLE: u32 = 0x4;
pub const PCI_DEVICE_TYPE_GENERAL: u32 = 0x0;
pub const PCI_BAR_TYPE_MEM: u32 = 0x0;
pub const PCI_BAR_TYPE_MEM_32: u32 = 0x0;
pub const PCI_BAR_TYPE_MEM_64: u32 = 0x2;
pub const PCI_BAR_TYPE_IO_PORT: u32 = 0x1;

pub struct PciBAR {
    pub base: u32,
    pub size: u32,
}

pub fn pci_read_reg(addr: u32) -> u32 {
    assert!(addr % 4 == 0);
    let mut address_port: Port<u32> = Port::new(PCI_ADD_PORT);
    let mut data_port: Port<u32> = Port::new(PCI_DATA_PORT);
 
    unsafe {address_port.write(addr);};
    unsafe{data_port.read()}
}

pub fn pci_write_reg(addr: u32, val: u32) {
    assert!(addr % 4 == 0);
    let mut address_port: Port<u32> = Port::new(PCI_ADD_PORT);
    let mut data_port: Port<u32> = Port::new(PCI_DATA_PORT);
 
    unsafe {address_port.write(addr);};
    unsafe{data_port.write(val)};
}

pub fn pci_bar_type(bar: u32) -> u32 {
    return bar & 0x1;
}

pub fn pci_bar_mem_type(bar: u32) -> u32 {
    return bar & 0x6;
}

pub fn pci_type_0_interrupt_line(pci_addr: u32) -> u32 {
    return pci_read_reg(pci_addr + PCI_INTERRUPT_LINE_REG) & 0xff;
}

pub fn pci_header_type(addr: u32) -> u32 {
    pci_read_reg(addr + PCI_HEADER_TYPE_REG) & 0xff0000
}

pub fn pci_resolve_bars(pci_addr: u32) -> Vec<PciBAR>
{
    assert!(pci_header_type(pci_addr) == PCI_DEVICE_TYPE_GENERAL);
	let mut bar_width: usize = 0x4;
    let mut bars = Vec::<PciBAR>::new();
	for bar in (PCI_BAR_START_REG..PCI_BAR_END_REG).step_by(bar_width)
	{
		let raw_base: u32 = pci_read_reg(pci_addr + bar);
		bar_width = 4;
		pci_write_reg(pci_addr + bar, u32::MAX);
		let raw_size: u32 = pci_read_reg(pci_addr + bar);

		if raw_size == 0 {
			continue;
        }

        let mut base: u32;
        let mut size: u32;
		if pci_bar_type(raw_size) == PCI_BAR_TYPE_MEM {
			if pci_bar_mem_type(raw_size) == PCI_BAR_TYPE_MEM_64 {
				bar_width = 8;
            }

			size = (raw_size & 0xfffffff0) & -((raw_size & 0xfffffff0) as i32) as u32;
			base = raw_base & 0xfffffff0;
		} else {
			size = (raw_size & 0xfffffffc) & -((raw_size & 0xfffffffc) as i32) as u32;
			base = (raw_base) & 0xfffffffc;
		}

		pci_write_reg(pci_addr + bar, raw_base);
        bars.push(PciBAR { base, size });

		if size == 0 || base == 0 {
			println!("PCI device has empty invalid address (base: {base}, size: {size})");
        }
	}
    bars
}

pub fn pci_init(
    mapper: &mut impl Mapper<Size4KiB>,
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,    
) {
    // Look at each PCI device on bus 0
    for bus in 0..256 {
        for dev in 0..32 {
            let func = 0;
            let offset = 0;
    
            let addr: u32 = 0x80000000 | (bus << 16) | (dev << 11) | (func << 8) | offset;
            let deve_id = pci_read_reg(addr);
            if deve_id != 0xffffffff {
                println!("PCI device id: {:#x} @ {:#x}", deve_id, addr);
            }
    
            // E1000 ID = 100e:8086
            if deve_id == 0x100e8086 {
                let mut nic = unsafe {E1000::new(addr, mapper, frame_allocator)};
                break;
            }
        }
    }

}
