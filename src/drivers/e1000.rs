use alloc::borrow::ToOwned;
use alloc::string::String;
// e1000 Driver for Intel 82540EP/EM
use volatile::Volatile;
use alloc::format;
use alloc::vec::Vec;
use linked_list_allocator::align_up;
use x86_64::structures::paging::FrameAllocator;
use x86_64::structures::paging::Mapper;
use x86_64::structures::paging::Size4KiB;
use core::{mem::size_of, slice::from_raw_parts_mut};
use crate::allocator::alloc_mapped_frame;
use crate::allocator::alloc_mapped_frames;
use crate::drivers::pci::PCI_COMMAND_IO_ENABLE;
use crate::drivers::pci::PCI_COMMAND_MASTER_ENABLE;
use crate::drivers::pci::PCI_COMMAND_MEM_ENABLE;
use crate::drivers::pci::PCI_COMMAND_STATUS_REG;
use crate::drivers::pci::pci_resolve_bars;
use crate::drivers::pci::pci_type_0_interrupt_line;
use crate::drivers::pci::pci_write_reg;
use crate::println;

// E1000 hardware definitions: registers and DMA ring format.
// from the Intel 82540EP/EM &c manual.

/* Registers */
pub const E1000_CTL: usize = 0x00000 / 4; /* Device Control Register - RW */
pub const E1000_STAT: usize = 0x00008 / 4; /* Device Status Register - R */
pub const E1000_ICR: usize = 0x000C0 / 4; /* Interrupt Cause Read - R */
pub const E1000_ITR: usize = 0x000C4 / 4; /* Interrupt Throttling Rate - RW */
pub const E1000_ICS: usize = 0x000C8 / 4; /* Interrupt Cause Set - WO */
pub const E1000_IMS: usize = 0x000D0 / 4; /* Interrupt Mask Set - RW */
pub const E1000_IMC: usize = 0x000D8 / 4; /* Interrupt Mask Clear - WO */
pub const E1000_RCTL: usize = 0x00100 / 4; /* RX Control - RW */
pub const E1000_TCTL: usize = 0x00400 / 4; /* TX Control - RW */
pub const E1000_TIPG: usize = 0x00410 / 4; /* TX Inter-packet gap -RW */
pub const E1000_RDBAL: usize = 0x02800 / 4; /* RX Descriptor Base Address Low - RW */
pub const E1000_RDBAH: usize = 0x02804 / 4; /* RX Descriptor Base Address High - RW */
pub const E1000_RDTR: usize = 0x02820 / 4; /* RX Delay Timer */
pub const E1000_RADV: usize = 0x0282C / 4; /* RX Interrupt Absolute Delay Timer */
pub const E1000_RDH: usize = 0x02810 / 4; /* RX Descriptor Head - RW */
pub const E1000_RDT: usize = 0x02818 / 4; /* RX Descriptor Tail - RW */
pub const E1000_RDLEN: usize = 0x02808 / 4; /* RX Descriptor Length - RW */
pub const E1000_RSRPD: usize = 0x02C00 / 4; /* RX Small Packet Detect Interrupt */
pub const E1000_TDBAL: usize = 0x03800 / 4; /* TX Descriptor Base Address Low - RW */
pub const E1000_TDBAH: usize = 0x03804 / 4; /* TX Descriptor Base Address High - RW */
pub const E1000_TDLEN: usize = 0x03808 / 4; /* TX Descriptor Length - RW */
pub const E1000_TDH: usize = 0x03810 / 4; /* TX Descriptor Head - RW */
pub const E1000_TDT: usize = 0x03818 / 4; /* TX Descripotr Tail - RW */
pub const E1000_TIDV: usize = 0x03820 / 4; /* TX Interrupt Delay Value - RW */
pub const E1000_TADV: usize = 0x0382C / 4; /* TX Interrupt Absolute Delay Val - RW */
pub const E1000_MTA: usize = 0x05200 / 4; /* Multicast Table Array - RW Array */
pub const E1000_RA: usize = 0x05400 / 4; /* Receive Address - RW Array */

/* This defines the bits that are set in the Interrupt Mask
 * Set/Read Register.  Each bit is documented below:
 *   o RXT0   = Receiver Timer Interrupt (ring 0)
 *   o TXDW   = Transmit Descriptor Written Back
 *   o RXDMT0 = Receive Descriptor Minimum Threshold hit (ring 0)
 *   o RXSEQ  = Receive Sequence Error
 *   o LSC    = Link Status Change
 */
pub const IMS_ENABLE_MASK: u32 = E1000_IMS_RXT0 | E1000_IMS_RXDMT0 | E1000_IMS_RXSEQ
    | E1000_IMS_LSC /* | E1000_IMS_TXQE | E1000_IMS_TXDW */;

pub const E1000_IMS_TXDW: u32 = 0x00000001;
pub const E1000_IMS_TXQE: u32 = 0x00000002;
pub const E1000_IMS_LSC: u32 = 0x00000004;
pub const E1000_IMS_RXSEQ: u32 = 0x00000008;
pub const E1000_IMS_RXDMT0: u32 = 0x00000010;
pub const E1000_IMS_RXT0: u32 = 0x00000080;

pub const E1000_ICR_LSC: u32 = 0x00000004; /* Link Status Change */

/* Device Control */
pub const E1000_CTL_SLU: u32 = 0x00000040; /* set link up */
pub const E1000_CTL_FRCSPD: u32 = 0x00000800; /* force speed */
pub const E1000_CTL_FRCDPLX: u32 = 0x00001000; /* force duplex */
pub const E1000_CTL_RST: u32 = 0x00400000; /* full reset */

/* Transmit Control */
pub const E1000_TCTL_RST: u32 = 0x00000001; /* software reset */
pub const E1000_TCTL_EN: u32 = 0x00000002; /* enable tx */
pub const E1000_TCTL_BCE: u32 = 0x00000004; /* busy check enable */
pub const E1000_TCTL_PSP: u32 = 0x00000008; /* pad short packets */
pub const E1000_TCTL_CT: u32 = 0x00000ff0; /* collision threshold */
pub const E1000_TCTL_CT_SHIFT: u32 = 4;
pub const E1000_TCTL_COLD: u32 = 0x003ff000; /* collision distance */
pub const E1000_TCTL_COLD_SHIFT: u32 = 12;
pub const E1000_TCTL_SWXOFF: u32 = 0x00400000; /* SW Xoff transmission */
pub const E1000_TCTL_PBE: u32 = 0x00800000; /* Packet Burst Enable */
pub const E1000_TCTL_RTLC: u32 = 0x01000000; /* Re-transmit on late collision */
pub const E1000_TCTL_NRTU: u32 = 0x02000000; /* No Re-transmit on underrun */
pub const E1000_TCTL_MULR: u32 = 0x10000000; /* Multiple request support */

/* Receive Control */
pub const E1000_RCTL_RST: u32 = 0x00000001; /* Software reset */
pub const E1000_RCTL_EN: u32 = 0x00000002; /* enable */
pub const E1000_RCTL_SBP: u32 = 0x00000004; /* store bad packet */
pub const E1000_RCTL_UPE: u32 = 0x00000008; /* unicast promiscuous enable */
pub const E1000_RCTL_MPE: u32 = 0x00000010; /* multicast promiscuous enab */
pub const E1000_RCTL_LPE: u32 = 0x00000020; /* long packet enable */
pub const E1000_RCTL_LBM_NO: u32 = 0x00000000; /* no loopback mode */
pub const E1000_RCTL_LBM_MAC: u32 = 0x00000040; /* MAC loopback mode */
pub const E1000_RCTL_LBM_SLP: u32 = 0x00000080; /* serial link loopback mode */
pub const E1000_RCTL_LBM_TCVR: u32 = 0x000000C0; /* tcvr loopback mode */
pub const E1000_RCTL_DTYP_MASK: u32 = 0x00000C00; /* Descriptor type mask */
pub const E1000_RCTL_DTYP_PS: u32 = 0x00000400; /* Packet Split descriptor */
pub const E1000_RCTL_RDMTS_HALF: u32 = 0x00000000; /* rx desc min threshold size */
pub const E1000_RCTL_RDMTS_QUAT: u32 = 0x00000100; /* rx desc min threshold size */
pub const E1000_RCTL_RDMTS_EIGTH: u32 = 0x00000200; /* rx desc min threshold size */
pub const E1000_RCTL_MO_SHIFT: u32 = 12; /* multicast offset shift */
pub const E1000_RCTL_MO_0: u32 = 0x00000000; /* multicast offset 11:0 */
pub const E1000_RCTL_MO_1: u32 = 0x00001000; /* multicast offset 12:1 */
pub const E1000_RCTL_MO_2: u32 = 0x00002000; /* multicast offset 13:2 */
pub const E1000_RCTL_MO_3: u32 = 0x00003000; /* multicast offset 15:4 */
pub const E1000_RCTL_MDR: u32 = 0x00004000; /* multicast desc ring 0 */
pub const E1000_RCTL_BAM: u32 = 0x00008000; /* broadcast enable */
/* these buffer sizes are valid if E1000_RCTL_BSEX is 0 */
pub const E1000_RCTL_SZ_2048: u32 = 0x00000000; /* rx buffer size 2048 */
pub const E1000_RCTL_SZ_1024: u32 = 0x00010000; /* rx buffer size 1024 */
pub const E1000_RCTL_SZ_512: u32 = 0x00020000; /* rx buffer size 512 */
pub const E1000_RCTL_SZ_256: u32 = 0x00030000; /* rx buffer size 256 */
/* these buffer sizes are valid if E1000_RCTL_BSEX is 1 */
pub const E1000_RCTL_SZ_16384: u32 = 0x00010000; /* rx buffer size 16384 */
pub const E1000_RCTL_SZ_8192: u32 = 0x00020000; /* rx buffer size 8192 */
pub const E1000_RCTL_SZ_4096: u32 = 0x00030000; /* rx buffer size 4096 */
pub const E1000_RCTL_VFE: u32 = 0x00040000; /* vlan filter enable */
pub const E1000_RCTL_CFIEN: u32 = 0x00080000; /* canonical form enable */
pub const E1000_RCTL_CFI: u32 = 0x00100000; /* canonical form indicator */
pub const E1000_RCTL_DPF: u32 = 0x00400000; /* discard pause frames */
pub const E1000_RCTL_PMCF: u32 = 0x00800000; /* pass MAC control frames */
pub const E1000_RCTL_BSEX: u32 = 0x02000000; /* Buffer size extension */
pub const E1000_RCTL_SECRC: u32 = 0x04000000; /* Strip Ethernet CRC */
pub const E1000_RCTL_FLXBUF_MASK: u32 = 0x78000000; /* Flexible buffer size */
pub const E1000_RCTL_FLXBUF_SHIFT: u32 = 27; /* Flexible buffer shift */

pub const DATA_MAX: u32 = 1518;

/* Transmit Descriptor command definitions [E1000 3.3.3.1] */
pub const E1000_TXD_CMD_EOP: u32 = 0x01; /* End of Packet */
pub const E1000_TXD_CMD_RS: u32 = 0x08; /* Report Status */

/* Transmit Descriptor status definitions [E1000 3.3.3.2] */
pub const E1000_TXD_STAT_DD: u32 = 0x00000001; /* Descriptor Done */

/* Receive Descriptor bit definitions [E1000 3.2.3.1] */
pub const E1000_RXD_STAT_DD: u32 = 0x01; /* Descriptor Done */
pub const E1000_RXD_STAT_EOP: u32 = 0x02; /* End of Packet */

const TX_RING_SIZE: usize = 256; // % 128 == 0
const RX_RING_SIZE: usize = 256; // % 128 == 0
const MBUF_SIZE: usize = 2048;

use crate::phys_offset;

/// Main structure of the e1000 driver.
/// Used to save members such as ring buffer.
pub struct E1000<'a> {
    regs: &'a mut [Volatile<u32>],
    rx_ring_dma: usize,
    tx_ring_dma: usize,
    rx_ring: &'a mut [RxDesc],
    tx_ring: &'a mut [TxDesc],
    rx_mbufs: &'a mut [[u8; MBUF_SIZE]],
    tx_mbufs: &'a mut [[u8; MBUF_SIZE]],
    mbuf_size: usize,

    // frame_allocator: &'a mut A,
}

// struct spinlock e1000_lock;

/// [E1000 3.3.3]
/// The dma descriptor for transmitting
#[derive(Debug, Clone)]
#[repr(C, align(16))]
pub struct TxDesc {
    addr: u64,
    length: u16,
    cso: u8,
    cmd: u8,
    status: u8,
    css: u8,
    special: u16,
}

/// [E1000 3.2.3]
/// The dma descriptor for receiving
#[derive(Debug, Clone)]
#[repr(C, align(16))]
pub struct RxDesc {
    addr: u64,   /* Address of the descriptor's data buffer */
    length: u16, /* Length of data DMAed into data buffer */
    csum: u16,   /* Packet checksum */
    status: u8,  /* Descriptor status */
    errors: u8,  /* Descriptor Errors */
    special: u16,
}

// todo free allocated frames on destruction
impl<'a> E1000<'a> {
    /// New an e1000 device by Allocating memory
    pub fn new(
        pci_addr: u32,
        mapper: &mut impl Mapper<Size4KiB>,
        frame_allocator: &mut impl FrameAllocator<Size4KiB>,    
        ) -> Result<Self, String> {

        pci_write_reg(pci_addr + PCI_COMMAND_STATUS_REG, PCI_COMMAND_IO_ENABLE |
                                                                PCI_COMMAND_MEM_ENABLE |
                                                                PCI_COMMAND_MASTER_ENABLE);
        let bars = pci_resolve_bars(pci_addr);
        let mut mapped_regs_base: usize = 0;
        let mut mapped_regs_len: usize = 0;
        for bar in bars.iter() {
            if bar.base > 0xffff {
                assert!(bar.size == (1 << 17));
                println!("New E1000 device @ {:#x} irq={}", bar.base, pci_type_0_interrupt_line(pci_addr));
                mapped_regs_base = unsafe {
                    phys_offset + bar.base as usize
                };
                mapped_regs_len = bar.size as usize;
                break;
            }
        }
        assert_ne!(mapped_regs_base, 0);
        assert_ne!(mapped_regs_len, 0);
    
        let regs = unsafe { from_raw_parts_mut(mapped_regs_base as *mut Volatile<u32>, mapped_regs_len) };
        let mut e1000dev = E1000::<'a> {
            regs,
            rx_ring_dma: 0,
            tx_ring_dma: 0,
            rx_ring: &mut [],
            tx_ring: &mut [],
            rx_mbufs: &mut [],
            tx_mbufs: &mut [],
            mbuf_size: MBUF_SIZE,
            // frame_allocator,
        };
        e1000dev.e1000_init();
        e1000dev.e1000_init_tx(mapper, frame_allocator)?;
        e1000dev.e1000_init_rx(mapper, frame_allocator)?;

        println!("e1000_init has been completed");
        Ok(e1000dev)
    }

    fn e1000_init_tx(&mut self, 
        mapper: &mut impl Mapper<Size4KiB>,
        frame_allocator: &mut impl FrameAllocator<Size4KiB>,
    ) -> Result<(), String> {
        // todo allocation failure
        let alloc_tx_ring_pages = align_up(TX_RING_SIZE * size_of::<TxDesc>(), 0x1000) >> 12;
        assert_eq!(alloc_tx_ring_pages, 1);
        let (tx_ring_vaddr, tx_ring_dma) = alloc_mapped_frame(mapper, frame_allocator).map_err(|e| e.to_owned())?;
        let tx_ring = unsafe { from_raw_parts_mut(tx_ring_vaddr as *mut TxDesc, TX_RING_SIZE) };
        
        assert_eq!(0x1000 % MBUF_SIZE, 0);
        let alloc_tx_buffer_pages = align_up(TX_RING_SIZE * MBUF_SIZE, 0x1000) >> 12;
        let (tx_mbufs_vaddr, mut tx_mbufs_dma) = alloc_mapped_frames(mapper, frame_allocator, alloc_tx_buffer_pages).map_err(|e| e.to_owned())?;
        let tx_mbufs =  unsafe { from_raw_parts_mut(tx_mbufs_vaddr as *mut [u8; MBUF_SIZE], TX_RING_SIZE) };

        self.tx_mbufs = tx_mbufs;
        for i in 0..TX_RING_SIZE {
            tx_ring[i] = TxDesc {
                addr: tx_mbufs_dma as u64,
                length: 0,
                cso: 0,
                cmd: 0,
                status: E1000_TXD_STAT_DD as u8,
                css: 0,
                special: 0,
            };
            tx_mbufs_dma += MBUF_SIZE;
        }

        self.tx_ring_dma = tx_ring_dma;
        self.tx_ring = tx_ring;

        self.regs[E1000_TDBAL].write(self.tx_ring_dma as u32);

        self.regs[E1000_TDLEN].write((self.tx_ring.len() * size_of::<TxDesc>()) as u32);
        self.regs[E1000_TDT].write(0);
        self.regs[E1000_TDH].write(0);

        // transmitter control bits.
        self.regs[E1000_TCTL].write(
            E1000_TCTL_EN |  // enable
            E1000_TCTL_PSP |                  // pad short packets
            (0x10 << E1000_TCTL_CT_SHIFT) |   // collision stuff
            (0x40 << E1000_TCTL_COLD_SHIFT),
        );
        self.regs[E1000_TIPG].write(10 | (8 << 10) | (6 << 20)); // inter-pkt gap

        // transmit interrupt delay
        self.regs[E1000_TIDV].write(0);
        self.regs[E1000_TADV].write(0);
        
        Ok(())
    }

    fn e1000_init_rx(&mut self,
        mapper: &mut impl Mapper<Size4KiB>,
        frame_allocator: &mut impl FrameAllocator<Size4KiB>,
    ) -> Result<(), String> {
        let alloc_rx_ring_pages = RX_RING_SIZE * size_of::<RxDesc>();
        let alloc_rx_buffer_pages = RX_RING_SIZE * MBUF_SIZE;
        let (rx_ring_vaddr, rx_ring_dma) = alloc_mapped_frames(mapper, frame_allocator, alloc_rx_ring_pages).map_err(|e| e.to_owned())?;
        let (rx_mbufs_vaddr, mut rx_mbufs_dma) = alloc_mapped_frames(mapper, frame_allocator, alloc_rx_buffer_pages).map_err(|e| e.to_owned())?;

        let rx_ring = unsafe { from_raw_parts_mut(rx_ring_vaddr as *mut RxDesc, RX_RING_SIZE) };

        let rx_mbufs =  unsafe { from_raw_parts_mut(rx_mbufs_vaddr as *mut [u8; MBUF_SIZE], RX_RING_SIZE) };
        self.rx_mbufs = rx_mbufs;

        for i in 0..RX_RING_SIZE {
            rx_ring[i] = RxDesc {
                addr: rx_mbufs_dma as u64,
                length: 0,
                status: 0,
                csum: 0,
                errors: 0,
                special: 0,
            };
            rx_mbufs_dma += MBUF_SIZE;
        }

        self.rx_ring_dma = rx_ring_dma;
        self.rx_ring = rx_ring;

        self.regs[E1000_RDBAL].write(self.rx_ring_dma as u32);

        self.regs[E1000_RDH].write(0);
        self.regs[E1000_RDT].write((RX_RING_SIZE - 1) as u32);
        self.regs[E1000_RDLEN].write((self.rx_ring.len() * size_of::<RxDesc>()) as u32);

        // receiver control bits.
        self.regs[E1000_RCTL].write(
            E1000_RCTL_EN | // enable receiver
            E1000_RCTL_BAM |                 // enable broadcast
            E1000_RCTL_SZ_2048 |             // 2048-byte rx buffers
            E1000_RCTL_SECRC, // strip CRC
        );

        // ask e1000 for receive interrupts.
        self.regs[E1000_RDTR].write(0); // interrupt after every received packet (no timer)
        self.regs[E1000_RADV].write(0); // interrupt after every packet (no timer)
        self.regs[E1000_IMS].write(1 << 7); // RXT0 - Receiver Timer Interrupt , RXDW -- Receiver Descriptor Write Back
        Ok(())
    }

    /// Initialize e1000 driver  
    /// mapped_regs is the memory address at which the e1000's registers are mapped.
    pub fn e1000_init(&mut self) {
        let stat = self.regs[E1000_STAT].read();
        let ctl = self.regs[E1000_CTL].read();

        // Reset the device
        self.regs[E1000_IMS].write(0); // disable interrupts
        self.regs[E1000_CTL].write(ctl | E1000_CTL_RST);
        self.regs[E1000_IMS].write(0); // redisable interrupts

        // multicast table
        for i in 0..(4096 / 32) {
            self.regs[E1000_MTA + i].write(0);
        }

        self.regs[E1000_ICR].read(); // clear ints
        self.e1000_write_flush();
    }

    /// Transmitting network packets
    pub fn e1000_transmit(&mut self, packet: &[u8]) -> Result<(), String> {
        let tindex = self.regs[E1000_TDT].read() as usize;
        println!("Read E1000_TDT = {:#x}", tindex);
        if (self.tx_ring[tindex].status & E1000_TXD_STAT_DD as u8) == 0 {
            return Err("E1000 hasn't finished the corresponding previous transmission request".to_owned());
        }

        let length = packet.len();
        if length > self.mbuf_size {
            return Err(format!("The packet: {} to be send is TOO LARGE", length));
        }

        let mbuf = &mut self.tx_mbufs[tindex];
        mbuf.copy_from_slice(packet);

        println!(">>>>>>>>> TX PKT {}", length);
        println!("\n");

        self.tx_ring[tindex].length = length as u16;
        self.tx_ring[tindex].status = 0;
        self.tx_ring[tindex].cmd = (E1000_TXD_CMD_RS | E1000_TXD_CMD_EOP) as u8;

        self.regs[E1000_TDT].write(((tindex + 1) % TX_RING_SIZE) as u32);

        self.e1000_write_flush();

        Ok(())
    }

    // Todo: send and recv lock
    /// Receiving network packets
    pub fn e1000_recv(&mut self) -> Option<Vec<u8>> {
        // Check for packets that have arrived from the e1000
        let rindex = (self.regs[E1000_RDT].read() as usize + 1) % RX_RING_SIZE;

        let desc = &mut self.rx_ring[rindex]; 
        if (desc.status & E1000_RXD_STAT_DD as u8) == 0 {
            return None;
        }
        if desc.length < 60 {
            println!("[e1000] short packet ({} bytes)\n", desc.length);
        }
        if (desc.status & E1000_RXD_STAT_EOP as u8) == 0 {
            println!("[e1000] not EOP! this driver does not support packet that do not fit in one buffer");
        }
        if desc.errors != 0 {
            println!("[e1000] rx errors ({:#x})", desc.errors);
        }

        let len = desc.length as usize;
        let mbuf = self.rx_mbufs[rindex];
        println!("RX PKT {} <<<<<<<<<", len);

        desc.status = 0;
        self.regs[E1000_RDT].write(rindex as u32);

        self.e1000_write_flush();

        println!("e1000_recv");
        Some(mbuf.to_vec())
    }
    
    /// Clear Interrupt
    pub fn e1000_irq_disable(&mut self) {
        self.regs[E1000_IMC].write(!0);
        self.e1000_write_flush();
    }

    /// Enable Interrupts
    pub fn e1000_irq_enable(&mut self) {
        self.regs[E1000_IMS].write(IMS_ENABLE_MASK);
        self.e1000_write_flush();
    }

    /// flush e1000 status
    pub fn e1000_write_flush(&mut self) {
        self.regs[E1000_STAT].read();
    }

    /// Cause a link status change interrupt
    pub fn e1000_cause_lsc_int(&mut self) {
        self.regs[E1000_ICS].write(E1000_ICR_LSC);
    }

    /// To handle e1000 interrupt
    pub fn e1000_intr(&mut self) -> u32 {
        self.e1000_recv();

        // tell the e1000 we've seen this interrupt;
        // without this the e1000 won't raise any
        // further interrupts.
        self.regs[E1000_ICR].read()
    }
}

