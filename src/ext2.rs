use alloc::borrow::ToOwned;
use alloc::{format, vec};
use alloc::string::String;
use alloc::vec::Vec;
use core::convert::TryInto;
use core::mem;

use crate::drivers::ide::IDEDisk;
use crate::println;
use bitflags::bitflags;

bitflags! {
    /// Optional features
    #[derive(Clone, Copy, Debug)]
    pub struct FeaturesOptional: u32 {
        /// Preallocate some number of (contiguous?) blocks (see
        /// `Superblock::prealloc_blocks_dirs`) to a directory when creating a new one
        const PREALLOCATE = 0x0001;
        /// AFS server inodes exist
        const AFS = 0x0002;
        /// File system has a journal (Ext3)
        const JOURNAL = 0x0004;
        /// Inodes have extended attributes
        const EXTENDED_INODE = 0x0008;
        /// File system can resize itself for larger partitions
        const SELF_RESIZE = 0x0010;
        /// Directories use hash index
        const HASH_INDEX = 0x0020;
    }
}

bitflags! {
    /// Required features. If these are not supported; can't mount
    #[derive(Clone, Copy, Debug)]
    pub struct FeaturesRequired: u32 {
        /// Compression is used
        const REQ_COMPRESSION = 0x0001;
        /// Directory entries contain a type field
        const REQ_DIRECTORY_TYPE = 0x0002;
        /// File system needs to replay its journal
        const REQ_REPLAY_JOURNAL = 0x0004;
        /// File system uses a journal device
        const REQ_JOURNAL_DEVICE = 0x0008;
    }
}

bitflags! {
    /// ROnly features. If these are not supported; remount as read-only
    #[derive(Clone, Copy, Debug)]
    pub struct FeaturesROnly: u32 {
        /// Sparse superblocks and group descriptor tables
        const RONLY_SPARSE = 0x0001;
        /// File system uses a 64-bit file size
        const RONLY_FILE_SIZE_64 = 0x0002;
        /// Directory contents are stored in the form of a Binary Tree
        const RONLY_BTREE_DIRECTORY = 0x0004;
    }
}

/// Ext2 signature (0xef53), used to help confirm the presence of Ext2 on a
/// volume
pub const EXT2_MAGIC: u16 = 0xef53;

/// Filesystem is free of errors
pub const FS_CLEAN: u16 = 1;
/// Filesystem has errors
pub const FS_ERR: u16 = 2;

/// Ignore errors
pub const ERR_IGNORE: u16 = 1;
/// Remount as read-only on error
pub const ERR_RONLY: u16 = 2;
/// Panic on error
pub const ERR_PANIC: u16 = 3;

/// Creator OS is Linux
pub const OS_LINUX: u32 = 0;
/// Creator OS is Hurd
pub const OS_HURD: u32 = 1;
/// Creator OS is Masix
pub const OS_MASIX: u32 = 2;
/// Creator OS is FreeBSD
pub const OS_FREEBSD: u32 = 3;
/// Creator OS is a BSD4.4-Lite derivative
pub const OS_LITE: u32 = 4;

/// The Superblock contains all information about the layout of the file system
/// and possibly contains other important information like what optional
/// features were used to create the file system.
///
/// The Superblock is always located at byte 1024 from the beginning of the
/// volume and is exactly 1024 bytes in length. For example, if the disk uses
/// 512 byte sectors, the Superblock will begin at LBA 2 and will occupy all of
/// sector 2 and 3.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug)]
pub struct Superblock {
    // taken from https://wiki.osdev.org/Ext2
    /// Total number of inodes in file system
    pub inodes_count: u32,
    /// Total number of blocks in file system
    pub blocks_count: u32,
    /// Number of blocks reserved for superuser (see offset 80)
    pub r_blocks_count: u32,
    /// Total number of unallocated blocks
    pub free_blocks_count: u32,
    /// Total number of unallocated inodes
    pub free_inodes_count: u32,
    /// Block number of the block containing the superblock
    pub first_data_block: u32,
    /// log2 (block size) - 10. (In other words, the number to shift 1,024
    /// to the left by to obtain the block size)
    pub log_block_size: u32,
    /// log2 (fragment size) - 10. (In other words, the number to shift
    /// 1,024 to the left by to obtain the fragment size)
    pub log_frag_size: i32,
    /// Number of blocks in each block group
    pub blocks_per_group: u32,
    /// Number of fragments in each block group
    pub frags_per_group: u32,
    /// Number of inodes in each block group
    pub inodes_per_group: u32,
    /// Last mount time (in POSIX time)
    pub mtime: u32,
    /// Last written time (in POSIX time)
    pub wtime: u32,
    /// Number of times the volume has been mounted since its last
    /// consistency check (fsck)
    pub mnt_count: u16,
    /// Number of mounts allowed before a consistency check (fsck) must be
    /// done
    pub max_mnt_count: i16,
    /// Ext2 signature (0xef53), used to help confirm the presence of Ext2
    /// on a volume
    pub magic: u16,
    /// File system state (see `FS_CLEAN` and `FS_ERR`)
    pub state: u16,
    /// What to do when an error is detected (see `ERR_IGNORE`, `ERR_RONLY` and
    /// `ERR_PANIC`)
    pub errors_handling: u16,
    /// Minor portion of version (combine with Major portion below to
    /// construct full version field)
    pub rev_minor: u16,
    /// POSIX time of last consistency check (fsck)
    pub lastcheck: u32,
    /// Interval (in POSIX time) between forced consistency checks (fsck)
    pub checkinterval: u32,
    /// Operating system ID from which the filesystem on this volume was
    /// created
    pub creator_os: u32,
    /// Major portion of version (combine with Minor portion above to
    /// construct full version field)
    pub rev_major: u32,
    /// User ID that can use reserved blocks
    pub block_uid: u16,
    /// Group ID that can use reserved blocks
    pub block_gid: u16,

    /// First non-reserved inode in file system.
    pub first_inode: u32,
    /// SectorSize of each inode structure in bytes.
    pub inode_size: u16,
    /// Block group that this superblock is part of (if backup copy)
    pub block_group: u16,
    /// Optional features present (features that are not required to read
    /// or write, but usually result in a performance increase)
    pub features_opt: FeaturesOptional,
    /// Required features present (features that are required to be
    /// supported to read or write)
    pub features_req: FeaturesRequired,
    /// Features that if not supported, the volume must be mounted
    /// read-only)
    pub features_ronly: FeaturesROnly,
    /// File system ID (what is output by blkid)
    pub fs_id: [u8; 16],
    /// Volume name (C-style string: characters terminated by a 0 byte)
    pub volume_name: [u8; 16],
    /// Path volume was last mounted to (C-style string: characters
    /// terminated by a 0 byte)
    pub last_mnt_path: [u8; 64],
    /// Compression algorithms used (see Required features above)
    pub compression: u32,
    /// Number of blocks to preallocate for files
    pub prealloc_blocks_files: u8,
    /// Number of blocks to preallocate for directories
    pub prealloc_blocks_dirs: u8,
    #[doc(hidden)]
    _unused: [u8; 2],
    /// Journal ID (same style as the File system ID above)
    pub journal_id: [u8; 16],
    /// Journal inode
    pub journal_inode: u32,
    /// Journal device
    pub journal_dev: u32,
    /// Head of orphan inode list
    pub journal_orphan_head: u32,
    #[doc(hidden)]
    _reserved: [u8; 788],
}

bitflags! {
    #[derive(Clone, Copy, Debug)]
    pub struct TypePerm: u16 {
        /// FIFO
        const FIFO = 0x1000;
        /// Character device
        const CHAR_DEVICE = 0x2000;
        /// Directory
        const DIRECTORY = 0x4000;
        /// Block device
        const BLOCK_DEVICE = 0x6000;
        /// Regular file
        const FILE = 0x8000;
        /// Symbolic link
        const SYMLINK = 0xA000;
        /// Unix socket
        const SOCKET = 0xC000;
        /// Other—execute permission
        const O_EXEC = 0x001;
        /// Other—write permission
        const O_WRITE = 0x002;
        /// Other—read permission
        const O_READ = 0x004;
        /// Group—execute permission
        const G_EXEC = 0x008;
        /// Group—write permission
        const G_WRITE = 0x010;
        /// Group—read permission
        const G_READ = 0x020;
        /// User—execute permission
        const U_EXEC = 0x040;
        /// User—write permission
        const U_WRITE = 0x080;
        /// User—read permission
        const U_READ = 0x100;
        /// Sticky Bit
        const STICKY = 0x200;
        /// Set group ID
        const SET_GID = 0x400;
        /// Set user ID
        const SET_UID = 0x800;
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug)]
    pub struct Flags: u32 {
        /// Secure deletion (not used)
        const SECURE_DEL = 0x00000001;
        /// Keep a copy of data when deleted (not used)
        const KEEP_COPY = 0x00000002;
        /// File compression (not used)
        const COMPRESSION = 0x00000004;
        /// Synchronous updates—new data is written immediately to disk
        const SYNC_UPDATE = 0x00000008;
        /// Immutable file (content cannot be changed)
        const IMMUTABLE = 0x00000010;
        /// Append only
        const APPEND_ONLY = 0x00000020;
        /// File is not included in 'dump' command
        const NODUMP = 0x00000040;
        /// Last accessed time should not updated
        const DONT_ATIME = 0x00000080;
        /// Hash indexed directory
        const HASH_DIR = 0x00010000;
        /// AFS directory
        const AFS_DIR = 0x00020000;
        /// Journal file data
        const JOURNAL_DATA = 0x00040000;
    }
}


/// An inode is a structure on the disk that represents a file, directory,
/// symbolic link, etc. Inodes do not contain the data of the file / directory /
/// etc. that they represent. Instead, they link to the blocks that actually
/// contain the data. This lets the inodes themselves have a well-defined size
/// which lets them be placed in easily indexed arrays. Each block group has an
/// array of inodes it is responsible for, and conversely every inode within a
/// file system belongs to one of such tables (and one of such block groups).
#[repr(C, packed)]
#[derive(Clone, Copy, Debug)]
pub struct Ext2Inode {
    /// Type and Permissions (see below)
    pub type_perm: TypePerm,
    /// User ID
    pub uid: u16,
    /// Lower 32 bits of size in bytes
    pub size_low: u32,
    /// Last Access Time (in POSIX time)
    pub atime: u32,
    /// Creation Time (in POSIX time)
    pub ctime: u32,
    /// Last Modification time (in POSIX time)
    pub mtime: u32,
    /// Deletion time (in POSIX time)
    pub dtime: u32,
    /// Group ID
    pub gid: u16,
    /// Count of hard links (directory entries) to this inode. When this
    /// reaches 0, the data blocks are marked as unallocated.
    pub hard_links: u16,
    /// Count of disk sectors (not Ext2 blocks) in use by this inode, not
    /// counting the actual inode structure nor directory entries linking
    /// to the inode.
    pub sectors_count: u32,
    /// Flags
    pub flags: Flags,
    /// Operating System Specific value #1
    pub _os_specific_1: [u8; 4],
    /// Direct block pointers
    pub direct_pointer: [u32; 12],
    /// Singly Indirect Block Pointer (Points to a block that is a list of
    /// block pointers to data)
    pub indirect_pointer: u32,
    /// Doubly Indirect Block Pointer (Points to a block that is a list of
    /// block pointers to Singly Indirect Blocks)
    pub doubly_indirect: u32,
    /// Triply Indirect Block Pointer (Points to a block that is a list of
    /// block pointers to Doubly Indirect Blocks)
    pub triply_indirect: u32,
    /// Generation number (Primarily used for NFS)
    pub gen_number: u32,
    /// In Ext2 version 0, this field is reserved. In version >= 1,
    /// Extended attribute block (File ACL).
    pub ext_attribute_block: u32,
    /// In Ext2 version 0, this field is reserved. In version >= 1, Upper
    /// 32 bits of file size (if feature bit set) if it's a file,
    /// Directory ACL if it's a directory
    pub size_high: u32,
    /// Block address of fragment
    pub frag_block_addr: u32,
    /// Operating System Specific Value #2
    pub _os_specific_2: [u8; 12],
}

/// The Block Group Descriptor Table contains a descriptor for each block group
/// within the file system. The number of block groups within the file system,
/// and correspondingly, the number of entries in the Block Group Descriptor
/// Table, is described above. Each descriptor contains information regarding
/// where important data structures for that group are located.
///
/// The (`BlockGroupDescriptor`) table is located in the block immediately
/// following the Superblock. So if the block size (determined from a field in
/// the superblock) is 1024 bytes per block, the Block Group Descriptor Table
/// will begin at block 2. For any other block size, it will begin at block 1.
/// Remember that blocks are numbered starting at 0, and that block numbers
/// don't usually correspond to physical block addresses.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug)]
pub struct BlockGroupDescriptor {
    /// Block address of block usage bitmap
    pub block_usage_addr: u32,
    /// Block address of inode usage bitmap
    pub inode_usage_addr: u32,
    /// Starting block address of inode table
    pub inode_table_block: u32,
    /// Number of unallocated blocks in group
    pub free_blocks_count: u16,
    /// Number of unallocated inodes in group
    pub free_inodes_count: u16,
    /// Number of directories in group
    pub dirs_count: u16,
    #[doc(hidden)]
    _reserved: [u8; 14],
}

static mut ext2_block_size: usize = 1 << 10;
static mut ext2_fragment_size: usize = 1 << 10;

fn read_block(disk: &mut IDEDisk, block: usize) -> Result<Vec<u8>, String> {
	let sector_per_block = unsafe {
		ext2_block_size / IDEDisk::SECTOR_SIZE
	};
	disk.read_sectors(block * sector_per_block, sector_per_block)
}

fn read_blocks(disk: &mut IDEDisk, block: usize, count: usize) -> Result<Vec<u8>, String> {
	let mut ret = vec![];
	for i in 0..count {
		let mut blk = read_block(disk, block + i)?;
		ret.append(&mut blk);
	}
	Ok(ret)
}

#[derive(Clone)]
pub struct DirectoryEntry {
    pub name: String,
    pub inode: u32,
    pub typ_e: u8,
}
#[derive(Debug)]
pub struct Inode {
	ino: u32,
    ext2_inode: Ext2Inode,
}

impl Inode {
    pub fn size(&self) -> usize {
        (self.ext2_inode.size_high as usize) << 32 | self.ext2_inode.size_low as usize
    }

    pub fn dir_find_ino(&self, part: &str, disk: &mut IDEDisk) -> Result<u32, String> {
        let type_perm = self.ext2_inode.type_perm;
        if type_perm.contains(TypePerm::DIRECTORY) == false {
            return Err(format!("inode is not dir ({})", part));
        }

        if self.size() == 0 {
            return Err("dir empty".to_owned());
        }

        let mut blki = 0_usize;
        while blki * unsafe {ext2_block_size} < self.size() {
            let blka = self.get_block_address(blki, disk)?; 
            let dentries = self.block_to_dentries(disk, blka as u32)?;
            for dentry in dentries.iter() {
                if dentry.name == part.to_owned() {
                    return Ok(dentry.inode);
                }
            }
            blki += 1;
        }
        Err("dentry not found".to_owned())
    }

    pub fn file_get_content(&self, disk: &mut IDEDisk) -> Result<Vec<u8>, String> {
        let type_perm = self.ext2_inode.type_perm;
        if type_perm.contains(TypePerm::FILE) == false {
            return Err("inode is not a regular file".to_owned());
        }

        let mut content: Vec<u8> = vec![];
        if self.size() == 0 {
            return Ok(content);
        }

        let mut blki = 0_usize;
        while blki * unsafe {ext2_block_size} < self.size() {
            let blka = self.get_block_address(blki, disk)?;
            let chunk = read_block(disk, blka)?;
            let rem = self.size() - blki * unsafe {ext2_block_size};
            content.append(&mut chunk[0..self.size().min(rem)].to_vec());
            blki += 1;
        }
        Ok(content)
    }

    fn block_to_dentries(&self, disk: &mut IDEDisk, block: u32) -> Result<Vec<DirectoryEntry>, String> {
        let raw_block = read_block(disk, block as usize)?;
        let mut v: Vec<DirectoryEntry> = vec![];
        let mut off = 0_usize;

        while off < unsafe {ext2_block_size} {
            let buffer = &raw_block[off..];

            let inode = buffer[0] as u32 | (buffer[1] as u32) << 8
                | (buffer[2] as u32) << 16
                | (buffer[3] as u32) << 24;
    
            let size = buffer[4] as u16 | (buffer[5] as u16) << 8;
            let name_len = buffer[6];
            let typ_e = buffer[7];
    
            let name = buffer[8..8 + name_len as usize].to_vec();
            let name = String::from_utf8(name).map_err(|_| "invalid name for dentry".to_owned())?;
            off += size as usize;
            
            if inode != 0 {
                v.push(DirectoryEntry {
                    name,
                    inode,
                    typ_e,
                })
            }
        }
        Ok(v)
    }

    fn get_block_address(&self, mut i: usize, disk: &mut IDEDisk) -> Result<usize, String> {
        let indirect_blk_addresses = unsafe { ext2_block_size } / 4;
        loop {
            if i < 12 {
                return Ok(self.ext2_inode.direct_pointer[i] as usize);
            }
            
            i -= 12;
            if i < indirect_blk_addresses {
                let mut block = read_block(disk, self.ext2_inode.indirect_pointer as usize)?;
                let ptr = block.as_mut_ptr() as *mut u32;
                let _block = unsafe {Vec::from_raw_parts(ptr, indirect_blk_addresses, indirect_blk_addresses)};
                mem::forget(block);
                return Ok(_block[i] as usize);
            }
            
            i -= indirect_blk_addresses;
            if i < indirect_blk_addresses * indirect_blk_addresses {
                let blocki = i / indirect_blk_addresses;
                let blockoff = i % indirect_blk_addresses;
                let mut blockr = read_block(disk, self.ext2_inode.doubly_indirect as usize)?;
                let ptr = blockr.as_mut_ptr() as *mut u32;
                let _block = unsafe {Vec::from_raw_parts(ptr, indirect_blk_addresses, indirect_blk_addresses)};
                mem::forget(blockr);

                let mut blockr2 = read_block(disk, _block[blocki] as usize)?;
                let ptr = blockr2.as_mut_ptr() as *mut u32;
                let __block = unsafe {Vec::from_raw_parts(ptr, indirect_blk_addresses, indirect_blk_addresses)};
                mem::forget(blockr2);

                return Ok(__block[blockoff] as usize);
            }
            
            i -= indirect_blk_addresses * indirect_blk_addresses;
            if i < indirect_blk_addresses * indirect_blk_addresses * indirect_blk_addresses {
                let blocki = i / (indirect_blk_addresses * indirect_blk_addresses);
                let blockoff = (i % (indirect_blk_addresses * indirect_blk_addresses)) / indirect_blk_addresses;
                let blockoff_off = i % (indirect_blk_addresses);
                let mut blockr = read_block(disk, self.ext2_inode.triply_indirect as usize)?;
                let ptr = blockr.as_mut_ptr() as *mut u32;
                let _block = unsafe {Vec::from_raw_parts(ptr, indirect_blk_addresses, indirect_blk_addresses)};
                mem::forget(blockr);

                let mut blockr2 = read_block(disk, _block[blocki] as usize)?;
                let ptr = blockr2.as_mut_ptr() as *mut u32;
                let __block = unsafe {Vec::from_raw_parts(ptr, indirect_blk_addresses, indirect_blk_addresses)};
                mem::forget(blockr2);

                let mut blockr3 = read_block(disk, __block[blockoff] as usize)?;
                let ptr = blockr3.as_mut_ptr() as *mut u32;
                let ___block = unsafe {Vec::from_raw_parts(ptr, indirect_blk_addresses, indirect_blk_addresses)};
                mem::forget(blockr3);

                return Ok(___block[blockoff_off] as usize);
            }

            return Err("blk index too big for ext2".to_owned());
        }

    }
}
pub struct Ext2Fs<'a> {
	superblock: Superblock,
	blkgroups: Vec<BlockGroupDescriptor>,
	disk: &'a mut IDEDisk,
}

struct MntPoint<'a> {
	path: String,
	fs: &'a Ext2Fs<'a>,
}

static mut MNT_POINTS: Vec<MntPoint> = vec![];

impl<'a> Ext2Fs<'a> {
	pub fn new(disk: &'a mut IDEDisk) -> Result<Self, String> {
		let superblock = read_block(disk, 1)?;
		let superblock = unsafe {*(superblock.as_ptr() as *mut Superblock)};
		if superblock.magic != EXT2_MAGIC {
			return Err("not ext2 filesystem".to_owned());
		}
	
		if superblock.state != FS_CLEAN && superblock.state != 0 {
            let ext2_state = superblock.state;
			return Err(format!("ext2 is not clean state={}", ext2_state))
		}
	
		unsafe {
			ext2_block_size <<= superblock.log_block_size;
			ext2_fragment_size <<= superblock.log_frag_size;
			println!("ext2 block size={ext2_block_size} fragment size={ext2_fragment_size}");
		}

		let blkgroups_block = unsafe {
			if ext2_block_size == 1024 { 2 } else { 1 }
		};
		let blkgroups_count: usize = superblock.blocks_count as usize / superblock.blocks_per_group as usize + if superblock.blocks_count as usize % superblock.blocks_per_group as usize != 0 { 1 } else { 0 };
		let blkgroups_blk_count: usize = blkgroups_count * mem::size_of::<BlockGroupDescriptor>() / unsafe { ext2_block_size } + if blkgroups_count * mem::size_of::<BlockGroupDescriptor>() % unsafe { ext2_block_size } != 0 { 1 } else { 0 };
		
		let mut __blkgroups = read_blocks(disk, blkgroups_block, blkgroups_blk_count)?;
		let raw_ptr = __blkgroups.as_mut_ptr() as *mut BlockGroupDescriptor;
        let blkgroups = unsafe {Vec::from_raw_parts(raw_ptr, blkgroups_blk_count, blkgroups_blk_count)};
        mem::forget(__blkgroups); // this is necessary to avoid double free
		Ok(Ext2Fs { superblock, disk, blkgroups })
	}

	// pub fn mount(&'a self, mnt_point: &String) {
	// 	unsafe {
    //         let mnt = MntPoint {
    //             path: mnt_point.clone(), fs: self
    //         };
	// 		MNT_POINTS.push(mnt);
	// 	}
	// }

	pub fn read_inode(&mut self, ino: u32) -> Result<Inode, String> {
		let inode_i: usize = (ino - 1) as usize;
		let superblock = &self.superblock;
		let inode_grp: usize = inode_i / superblock.inodes_per_group as usize;
		let grp_off: usize = inode_i % superblock.inodes_per_group as usize;

		let grp_block = grp_off * superblock.inode_size as usize / unsafe {ext2_block_size};
		let grp_block_off = grp_off * superblock.inode_size as usize % unsafe {ext2_block_size};

		let grp: &BlockGroupDescriptor = &self.blkgroups[inode_grp];

		let block = read_block(self.disk, grp.inode_table_block as usize + grp_block)?;
		let ext2_inode = unsafe {&*(block[grp_block_off..grp_block_off+mem::size_of::<Ext2Inode>()].as_ptr() as *const Ext2Inode)};
		
		Ok(Inode { ino, ext2_inode: ext2_inode.clone() })
	}

    pub fn path_to_inode(&mut self, path: String) -> Result<Inode, String> {

        println!("++++path={path}");
        if path.len() == 0 || path.as_bytes()[0] != b'/' {
            return Err("invalid path".to_owned());
        }

        let root_inode = self.read_inode(2)?;
        let mut current_inode = root_inode;

        let mut parts = path.split('/');
        parts.next();
        for part in parts {
            println!("part = {:?}", part);
            if part.len() == 0 {
                break;
            }
            let ino = current_inode.dir_find_ino(part, self.disk)?;
            println!("part={} has inode={}", part, ino);
            current_inode = self.read_inode(ino)?;
        }

        Ok(current_inode)
    }
}
