use alloc::{string::{String, ToString}, borrow::ToOwned, boxed::Box, vec};
use x86_64::instructions::port::Port;
use alloc::vec::Vec;
use crate::println;

/**
 * todo
 * document io ports
 * advanced detection of ide in pci bus
 * advanced detection of wether the drive uses pci native or compatibility mode
 * probe other ide channels and drives
 * test cases for disk operation:
 * let mut mbr: [u8; 512] = [0; 512];
 * unsafe { disk.read_sector(0, &mut mbr) };
 * println!("ide sector read: {:#x?}", &mbr[510..512]);
 * mbr[510] = 0x88;
 * mbr[511] = 0x99;
 * unsafe { disk.write_sector(0, &mbr) };
*/

pub const IDE_PRIMARY_CHANNEL_BUS: u16 = 0x1F0;
pub const IDE_SECONDARY_CHANNEL_BUS: u16 = 0x170;
const ATA_CMD_IDENTIFY: u8 = 0xEC;

// command register flags
const ATA_CMD_READ_PIO: u8 = 0x20;
const ATA_CMD_WRITE_PIO: u8 = 0x30;
const ATA_CMD_CACHE_FLUSH: u8 = 0xE7;

// status register flags
const ATA_ST_BSY: u8 = 0x80;
/// Drive write fault
const ATA_ST_DF: u8 = 0x20;
/// Data request ready
const ATA_ST_DRQ: u8 = 0x08;
const ATA_ST_ERR: u8 = 0x01;


#[repr(C)]
#[repr(packed)]
struct AtaIdentify {
    flags: u16,
    unused1: [u16; 9],
    serial: [u8; 20],
    unused2: [u16; 3],
    firmware: [u8; 8],
    model: [u16; 20],
    sectors_per_int: u16,
    unused3: u16,
    capabilities: [u16; 2],
    unused4: [u16; 2],
    valid_ext_data: u16,
    unused5: [u16; 5],
    size_of_rw_mult: u16,
    sectors_28: u32,
    unused6: [u16; 19],
    unused7: [u16; 19],
    sectors_48: u64,
    unused8: [u16; 152],
}

impl AtaIdentify {
    const fn default() -> AtaIdentify {
        AtaIdentify {
            flags: 0,
            unused1: [0; 9],
            serial: [0; 20],
            unused2: [0; 3],
            firmware: [0; 8],
            model: [0; 20],
            sectors_per_int: 0,
            unused3: 0,
            capabilities: [0; 2],
            unused4: [0; 2],
            valid_ext_data: 0,
            unused5: [0; 5],
            size_of_rw_mult: 0,
            sectors_28: 0,
            unused6: [0; 19],
            unused7: [0; 19],
            sectors_48: 0,
            unused8: [0; 152],
        }
    }
}
pub struct IDEDisk {
    raw_base_port: u16,
    base_w_port: Port<u16>,
    control_port: Port<u8>,
    hddevsel_port: Port<u8>,
    control_altstatus_port: Port<u8>,
    command_status_port: Port<u8>,
    features_errors_port: Port<u8>,
    seccount0_port: Port<u8>,
    lba0_port: Port<u8>,
    lba1_port: Port<u8>,
    lba2_port: Port<u8>,
    device: AtaIdentify,
    model: String,
    size: u32,
}

unsafe fn any_as_u8_slice<T: Sized>(p: &mut T) -> &mut [u8] {
    ::core::slice::from_raw_parts_mut(
        (p as *mut T) as *mut u8,
        ::core::mem::size_of::<T>(),
    )
}

impl IDEDisk {
    pub const SECTOR_SIZE: usize = 512;

    pub fn new(port: u16) -> Result<IDEDisk, String>  {
        let mut ide = IDEDisk {
            raw_base_port: port,
            base_w_port: Port::new(port),
            features_errors_port: Port::new(port + 0x01),
            seccount0_port: Port::new(port + 0x02),
            lba0_port: Port::new(port + 0x03),
            lba1_port: Port::new(port + 0x04),
            lba2_port: Port::new(port + 0x05),
            /// Bits 0 : 3: Head Number for CHS. or the upper 4 bits for lba
            /// Bit 4: Slave Bit. (0: Selecting Master Drive, 1: Selecting Slave Drive).
            /// Bit 5: Obsolete and isn't used, but should be set.
            /// Bit 6: LBA (0: CHS, 1: LBA).
            /// Bit 7: Obsolete and isn't used, but should be set.
            hddevsel_port: Port::new(port + 0x06), // used to select master/slave drive on the channel
            command_status_port: Port::new(port + 0x07),
            control_port: Port::new(port + 0x206), // todo: maybe this is 0x3f6 (off: 0x206)
            /// Bit 1: disable interrupts
            control_altstatus_port: Port::new(port + 0x0C), // todo: this probably invalid in portability mode because max port is at off: 0x7
            device: AtaIdentify::default(),
            model: String::new(),
            size: 0
        };
        unsafe { ide.init()?; };
        Ok(ide)
    }

    // todo: add support for the slave drive
    pub unsafe fn init(&mut self) -> Result<(), String> {
        println!("initializing IDE device on port {:#x} ...", self.raw_base_port);

        // not PIO Mode: just in initialization stage
        self.features_errors_port.write(1);
        self.control_port.write(0); // wtf ?
    
        // select master drive, lba mode
	    self.hddevsel_port.write(0xE0);

        self.ata_io_delay();

        self.command_status_port.write(ATA_CMD_IDENTIFY);

        self.ata_wait_data()?;
    
        let buf = unsafe { any_as_u8_slice(&mut self.device) };

        for i in 0..Self::SECTOR_SIZE/2 {
            let word = self.base_w_port.read();
            buf[i * 2] = word as u8;
            buf[i * 2 + 1] = (word >> 8) as u8;
        }

        // fix the name: translate to big endian
        let mut disk_name = self.device.model;
        for i in 0..disk_name.len() {
            disk_name[i] = disk_name[i].to_be();
        }
        self.model = String::from_raw_parts(disk_name.as_mut_ptr() as *mut u8, disk_name.len(), disk_name.len());
        self.size = self.device.sectors_28;
        // println!("sectors_48 = {:?}", device.sectors_48);
        // println!("sectors_28 = {}", device.sectors_28);
        
        // disable interrupts
        self.ata_disable_interrupts();
        self.ata_wait_and_check()?;
        println!("ide disk: {:?} with {} sector initialized", self.model, self.size);
        Ok(())
    }

    unsafe fn ata_io_delay(&mut self) {
        self.control_altstatus_port.read();
        self.control_altstatus_port.read();
        self.control_altstatus_port.read();
        self.control_altstatus_port.read();
    }

    unsafe fn ata_wait_ready(&mut self) {
        loop {
            if self.command_status_port.read() & ATA_ST_BSY == 0 {
                break;
            }
        }
    }

    unsafe fn ata_wait_and_check(&mut self) -> Result<u8, String> {
        self.ata_io_delay();
    
        self.ata_wait_ready();

        let status = self.command_status_port.read();
        if status & ATA_ST_ERR != 0 {
            return Err(String::from("ide: ATA_ST_ERR"));
        }
        if status & ATA_ST_DF != 0 {
            return Err(String::from("ide: ATA_ST_DF"));
        }

        Ok(status)
    }

    unsafe fn ata_wait_data(&mut self) -> Result<(), String> {
        let status = self.ata_wait_and_check()?;

        if status & ATA_ST_DRQ == 0 {
            return Err(String::from("ide: !ATA_ST_DRQ"));
        }

        Ok(())
    }

    unsafe fn ata_disable_interrupts(&mut self) {
        self.control_altstatus_port.write(0x02);
    }

    pub unsafe fn read_sector(&mut self, lba: u32, buf: &mut [u8]) -> Result<(), String> {
        if lba >= self.size {
            return Err("out of range lba ".to_owned() + lba.to_string().as_str());
        }
        self.ata_disable_interrupts();

        self.ata_wait_ready();

        const SLAVE: u8 = 0;
        // lba, slave, top 4 bits of lba
        self.hddevsel_port.write((0xe0 | SLAVE << 4 | ((lba & 0x0f000000) >> 24) as u8) as u8);
        // PIO Mode
        self.features_errors_port.write(0x00);
        self.seccount0_port.write(1);
        self.lba0_port.write(((lba & 0x000000ff) >>  0) as u8);
        self.lba1_port.write(((lba & 0x0000ff00) >> 8) as u8);
        self.lba2_port.write(((lba & 0x00ff0000) >> 16) as u8);
        self.command_status_port.write(ATA_CMD_READ_PIO);

        self.ata_wait_data()?;

        for i in 0..Self::SECTOR_SIZE/2 {
            let word = self.base_w_port.read();
            buf[i * 2] = word as u8;
            buf[i * 2 + 1] = (word >> 8) as u8;
        }        
        self.ata_wait_and_check()?;
        Ok(())
    }
    
    pub unsafe fn write_sector(&mut self, lba: u32, buf: &[u8]) -> Result<(), String> {
        if lba >= self.size {
            return Err("out of range lba ".to_owned() + lba.to_string().as_str());
        }

    	self.ata_disable_interrupts();
    
    	self.ata_wait_ready();

        const SLAVE: u8 = 0;
        self.hddevsel_port.write((0xe0 | SLAVE << 4 | ((lba & 0x0f000000) >> 24) as u8) as u8);
        // PIO Mode
        self.features_errors_port.write(0x00);
        self.seccount0_port.write(1);
        self.lba0_port.write(((lba & 0x000000ff) >>  0) as u8);
        self.lba1_port.write(((lba & 0x0000ff00) >> 8) as u8);
        self.lba2_port.write(((lba & 0x00ff0000) >> 16) as u8);
        self.command_status_port.write(ATA_CMD_WRITE_PIO);

        self.ata_wait_data()?;

        for i in 0..Self::SECTOR_SIZE/2 {
            let word: u16 = ((buf[i * 2 + 1] as u16) << 8) | buf[i * 2] as u16;
            self.base_w_port.write(word);
        }

        self.command_status_port.write(ATA_CMD_CACHE_FLUSH);
        self.ata_wait_and_check()?;
        Ok(())
    }

    pub fn read_sectors(&mut self, lba: usize, count: usize) -> Result<Vec<u8>, String> {
        let mut buf = vec![0_u8; Self::SECTOR_SIZE * count as usize];
        for sector in 0..count {
            unsafe {self.read_sector(lba + sector, &mut buf[Self::SECTOR_SIZE*(sector as usize)..])?};
        }
        Ok(buf)
    }
    
    pub fn write_sectors(&mut self, lba: usize, buf: &[u8], count: usize) -> Result<(), String> {
        for sector in 0..count {
            unsafe {self.write_sector(lba + sector, &buf[Self::SECTOR_SIZE*(sector as usize)..])?};
        }
        Ok(())
    }

    pub unsafe fn write_sector_retry(&mut self, lba: u32, buf: &[u8]) -> Result<(), String> {
        let mut read_buf: [u8; 512] = [0; 512];
        loop {
            self.write_sector(lba, buf)?;
            self.ata_wait_and_check()?;
            self.read_sector(lba, &mut read_buf)?;
            if buf == &read_buf {
                return Ok(());
            }
        }
    }

}

