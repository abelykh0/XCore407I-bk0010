//! Dhara-backed logical block device for LittleFS.
//!
//! Dhara owns all 1,024 physical NAND blocks. LittleFS sees a fixed logical
//! geometry of 740 erase blocks; its reads, writes and erases are translated
//! to Dhara sectors. The NAND OOB layout remains unchanged: factory markers
//! at bytes 0..1 and STM32 FSMC Hamming ECC/tag at bytes 2..15.

#![cfg(feature = "dhara-littlefs")]

use core::ffi::c_int;
use core::slice;

use littlefs2::{
    consts::{U2048, U32},
    driver::Storage,
    fs::Filesystem,
    io::{Error as LfsError, Result as LfsResult},
};

use crate::nand_fs::{
    self, ERASE_BLOCK_SIZE, PAGE_SIZE, PHYSICAL_BLOCKS, PAGES_PER_BLOCK, SPARE_SIZE,
};

const LITTLEFS_BLOCK_COUNT: usize = 700;
const SECTORS_PER_LITTLEFS_BLOCK: usize = ERASE_BLOCK_SIZE / PAGE_SIZE;
const VOLUME_MARKER_SECTOR: u32 =
    (LITTLEFS_BLOCK_COUNT * SECTORS_PER_LITTLEFS_BLOCK) as u32;
const VOLUME_MARKER: &[u8; 8] = b"XCDHARA1";
const DHARA_E_BAD_BLOCK: i32 = 1;
const DHARA_E_ECC: i32 = 2;

#[repr(C)]
struct DharaNand {
    log2_page_size: u8,
    log2_ppb: u8,
    num_blocks: u32,
}

unsafe extern "C" {
    fn xcore_dhara_map_init();
    fn xcore_dhara_map_resume(error: *mut i32) -> c_int;
    fn xcore_dhara_map_capacity() -> u32;
    fn xcore_dhara_map_read(sector: u32, data: *mut u8, error: *mut i32) -> c_int;
    fn xcore_dhara_map_write(sector: u32, data: *const u8, error: *mut i32) -> c_int;
    fn xcore_dhara_map_trim(sector: u32, error: *mut i32) -> c_int;
    fn xcore_dhara_map_sync(error: *mut i32) -> c_int;
}

static mut NAND_PAGE_SCRATCH: [u8; PAGE_SIZE + SPARE_SIZE] =
    [0; PAGE_SIZE + SPARE_SIZE];
static mut VOLUME_MARKER_BUFFER: [u8; PAGE_SIZE] = [0xFF; PAGE_SIZE];

fn valid_nand(nand: *const DharaNand) -> bool {
    if nand.is_null() {
        return false;
    }
    unsafe {
        (*nand).log2_page_size == 11
            && (*nand).log2_ppb == 6
            && (*nand).num_blocks == PHYSICAL_BLOCKS
    }
}

fn block_is_bad(block: u32) -> Result<bool, ()> {
    if block >= PHYSICAL_BLOCKS {
        return Err(());
    }
    let first_page = block * PAGES_PER_BLOCK;
    let mut marker = [0u8; 1];
    if !nand_fs::read_page(first_page, PAGE_SIZE as u16, &mut marker) {
        return Err(());
    }
    if marker[0] != 0xFF {
        return Ok(true);
    }
    if !nand_fs::read_page(first_page + 1, PAGE_SIZE as u16, &mut marker) {
        return Err(());
    }
    Ok(marker[0] != 0xFF)
}

fn program_bad_marker(page: u32) -> bool {
    // K9F1G08U0E factory/grown-bad marker: spare-area byte zero.
    nand_fs::command(0x80);
    nand_fs::address(0);
    nand_fs::address((PAGE_SIZE >> 8) as u8);
    nand_fs::address(page as u8);
    nand_fs::address((page >> 8) as u8);
    cortex_m::asm::delay(1_000);
    nand_fs::write_data(0x00);
    nand_fs::command(0x10);
    if !nand_fs::wait_ready_rb() {
        return false;
    }
    nand_fs::command(0x70);
    nand_fs::read_data() & 0x01 == 0
}

fn mark_block_bad(block: u32) -> bool {
    if block >= PHYSICAL_BLOCKS {
        return false;
    }
    let first = block * PAGES_PER_BLOCK;
    let first_ok = program_bad_marker(first);
    let second_ok = program_bad_marker(first + 1);
    first_ok || second_ok
}

fn mark_bad_and_verify(block: u32) -> bool {
    mark_block_bad(block) && matches!(block_is_bad(block), Ok(true))
}

fn erase_all_good_blocks() -> Result<u16, crate::nand_fs::InitError> {
    let mut bad_blocks = 0u16;
    let scratch = unsafe { &mut *core::ptr::addr_of_mut!(NAND_PAGE_SCRATCH) };

    for block in 0..PHYSICAL_BLOCKS {
        match block_is_bad(block) {
            Ok(true) => {
                bad_blocks = bad_blocks.saturating_add(1);
                continue;
            }
            Ok(false) => {}
            Err(()) => return Err(crate::nand_fs::InitError::MarkerRead),
        }

        if !nand_fs::erase_physical_block(block as u16) {
            if !mark_bad_and_verify(block) {
                return Err(crate::nand_fs::InitError::DharaErase);
            }
            bad_blocks = bad_blocks.saturating_add(1);
            continue;
        }

        // Verify both ends of each erased block, including OOB/ECC bytes.
        for page_in_block in [0, PAGES_PER_BLOCK - 1] {
            let page = block * PAGES_PER_BLOCK + page_in_block;
            if !nand_fs::read_page(page, 0, scratch)
                || scratch.iter().any(|&byte| byte != 0xFF)
            {
                if !mark_block_bad(block) {
                    return Err(crate::nand_fs::InitError::DharaErase);
                }
                bad_blocks = bad_blocks.saturating_add(1);
                break;
            }
        }
    }

    Ok(bad_blocks)
}

unsafe fn set_error(error: *mut i32, value: i32) {
    if !error.is_null() {
        unsafe { *error = value; }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn dhara_nand_is_bad(nand: *const DharaNand, block: u32) -> c_int {
    if !valid_nand(nand) {
        return 1;
    }
    match block_is_bad(block) {
        Ok(true) => 1,
        Ok(false) => 0,
        Err(()) => 1,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn dhara_nand_mark_bad(nand: *const DharaNand, block: u32) {
    if valid_nand(nand) {
        let _ = mark_block_bad(block);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn dhara_nand_erase(
    nand: *const DharaNand,
    block: u32,
    error: *mut i32,
) -> c_int {
    if !valid_nand(nand) || block >= PHYSICAL_BLOCKS {
        unsafe { set_error(error, DHARA_E_BAD_BLOCK); }
        return -1;
    }
    if nand_fs::erase_physical_block(block as u16) {
        unsafe { set_error(error, 0); }
        0
    } else {
        unsafe { set_error(error, DHARA_E_BAD_BLOCK); }
        -1
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn dhara_nand_prog(
    nand: *const DharaNand,
    page: u32,
    data: *const u8,
    error: *mut i32,
) -> c_int {
    if !valid_nand(nand)
        || page >= PHYSICAL_BLOCKS * PAGES_PER_BLOCK
        || data.is_null()
    {
        unsafe { set_error(error, DHARA_E_BAD_BLOCK); }
        return -1;
    }
    let input = unsafe { slice::from_raw_parts(data, PAGE_SIZE) };
    if nand_fs::program_physical_page(page, input) {
        unsafe { set_error(error, 0); }
        0
    } else {
        unsafe { set_error(error, DHARA_E_BAD_BLOCK); }
        -1
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn dhara_nand_is_free(nand: *const DharaNand, page: u32) -> c_int {
    if !valid_nand(nand) || page >= PHYSICAL_BLOCKS * PAGES_PER_BLOCK {
        return 0;
    }
    let scratch = unsafe { &mut *core::ptr::addr_of_mut!(NAND_PAGE_SCRATCH) };
    if !nand_fs::read_page(page, 0, scratch) {
        return 0;
    }
    if scratch.iter().all(|&byte| byte == 0xFF) { 1 } else { 0 }
}

#[unsafe(no_mangle)]
pub extern "C" fn dhara_nand_read(
    nand: *const DharaNand,
    page: u32,
    offset: usize,
    length: usize,
    data: *mut u8,
    error: *mut i32,
) -> c_int {
    if !valid_nand(nand)
        || page >= PHYSICAL_BLOCKS * PAGES_PER_BLOCK
        || offset > PAGE_SIZE
        || length > PAGE_SIZE.saturating_sub(offset)
        || (length != 0 && data.is_null())
    {
        unsafe { set_error(error, DHARA_E_ECC); }
        return -1;
    }

    let scratch = unsafe { &mut *core::ptr::addr_of_mut!(NAND_PAGE_SCRATCH) };
    if !nand_fs::read_data_page(page, &mut scratch[..PAGE_SIZE]) {
        unsafe { set_error(error, DHARA_E_ECC); }
        return -1;
    }
    if length != 0 {
        unsafe {
            core::ptr::copy_nonoverlapping(scratch.as_ptr().add(offset), data, length);
        }
    }
    unsafe { set_error(error, 0); }
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn dhara_nand_copy(
    nand: *const DharaNand,
    source: u32,
    destination: u32,
    error: *mut i32,
) -> c_int {
    if !valid_nand(nand)
        || source >= PHYSICAL_BLOCKS * PAGES_PER_BLOCK
        || destination >= PHYSICAL_BLOCKS * PAGES_PER_BLOCK
    {
        unsafe { set_error(error, DHARA_E_BAD_BLOCK); }
        return -1;
    }

    let scratch = unsafe { &mut *core::ptr::addr_of_mut!(NAND_PAGE_SCRATCH) };
    if !nand_fs::read_data_page(source, &mut scratch[..PAGE_SIZE]) {
        unsafe { set_error(error, DHARA_E_ECC); }
        return -1;
    }
    if !nand_fs::program_physical_page(destination, &scratch[..PAGE_SIZE]) {
        unsafe { set_error(error, DHARA_E_BAD_BLOCK); }
        return -1;
    }
    unsafe { set_error(error, 0); }
    0
}

pub struct DharaStorage {
    factory_bad_blocks: u16,
}

impl DharaStorage {
    pub fn new() -> Result<Self, crate::nand_fs::InitError> {
        nand_fs::configure_fsmc();
        nand_fs::command(0xFF);
        if !nand_fs::wait_ready_status() {
            return Err(crate::nand_fs::InitError::Timeout);
        }

        nand_fs::command(0x90);
        nand_fs::address(0);
        let id = [
            nand_fs::read_data(),
            nand_fs::read_data(),
            nand_fs::read_data(),
            nand_fs::read_data(),
            nand_fs::read_data(),
        ];
        if id[0] != 0xEC || id[1] != 0xF1 {
            return Err(crate::nand_fs::InitError::WrongDevice);
        }

        unsafe { xcore_dhara_map_init(); }
        let mut dhara_error = 0i32;
        let mut resumed = unsafe { xcore_dhara_map_resume(&mut dhara_error) } == 0;
        let mut bad_blocks = 0u16;

        let capacity = unsafe { xcore_dhara_map_capacity() } as usize;
        if capacity <= VOLUME_MARKER_SECTOR as usize {
            return Err(crate::nand_fs::InitError::DharaCapacity);
        }

        // A private logical sector outside LittleFS carries an application
        // signature. It prevents arbitrary raw-NAND contents from being
        // mistaken for a valid Dhara volume during first provisioning.
        if resumed {
            let marker = unsafe { &mut *core::ptr::addr_of_mut!(VOLUME_MARKER_BUFFER) };
            marker.fill(0xFF);
            let mut error = 0i32;
            resumed = unsafe {
                xcore_dhara_map_read(
                    VOLUME_MARKER_SECTOR,
                    marker.as_mut_ptr(),
                    &mut error,
                )
            } == 0 && &marker[..VOLUME_MARKER.len()] == VOLUME_MARKER;
        }

        if !resumed {
            #[cfg(feature = "dhara-format-nand")]
            {
                bad_blocks = erase_all_good_blocks()?;
                unsafe { xcore_dhara_map_init(); }

                let marker = unsafe { &mut *core::ptr::addr_of_mut!(VOLUME_MARKER_BUFFER) };
                marker.fill(0xFF);
                marker[..VOLUME_MARKER.len()].copy_from_slice(VOLUME_MARKER);
                let mut error = 0i32;
                if unsafe {
                    xcore_dhara_map_write(
                        VOLUME_MARKER_SECTOR,
                        marker.as_ptr(),
                        &mut error,
                    )
                } != 0 || unsafe { xcore_dhara_map_sync(&mut error) } != 0 {
                    return Err(crate::nand_fs::InitError::DharaFormat);
                }
            }
            #[cfg(not(feature = "dhara-format-nand"))]
            {
                let _ = dhara_error;
                return Err(crate::nand_fs::InitError::DharaNoJournal);
            }
        } else {
            for block in 0..PHYSICAL_BLOCKS {
                match block_is_bad(block) {
                    Ok(true) => bad_blocks = bad_blocks.saturating_add(1),
                    Ok(false) => {}
                    Err(()) => return Err(crate::nand_fs::InitError::MarkerRead),
                }
            }
        }

        let capacity = unsafe { xcore_dhara_map_capacity() } as usize;
        if capacity <= VOLUME_MARKER_SECTOR as usize
            || capacity < LITTLEFS_BLOCK_COUNT * SECTORS_PER_LITTLEFS_BLOCK
        {
            return Err(crate::nand_fs::InitError::DharaCapacity);
        }

        let mut storage = Self { factory_bad_blocks: bad_blocks };

        // This feature is explicitly destructive. On an uninitialized device,
        // the failed journal resume above caused a full erase and a fresh map.
        // If a valid Dhara journal exists but LittleFS is absent, format only
        // the Dhara logical volume; never erase the physical NAND again.
        #[cfg(feature = "dhara-format-nand")]
        if !Filesystem::<Self>::is_mountable(&mut storage) {
            Filesystem::<Self>::format(&mut storage)
                .map_err(|_| crate::nand_fs::InitError::DharaFormat)?;
            let mut error = 0i32;
            if unsafe { xcore_dhara_map_sync(&mut error) } != 0 {
                return Err(crate::nand_fs::InitError::DharaFormat);
            }
        }

        Ok(storage)
    }

    pub fn factory_bad_blocks(&self) -> u16 {
        self.factory_bad_blocks
    }

    pub fn logical_capacity(&self) -> usize {
        LITTLEFS_BLOCK_COUNT * ERASE_BLOCK_SIZE
    }
}

impl Storage for DharaStorage {
    type CACHE_SIZE = U2048;
    type LOOKAHEAD_SIZE = U32;

    const READ_SIZE: usize = PAGE_SIZE;
    const WRITE_SIZE: usize = PAGE_SIZE;
    const BLOCK_SIZE: usize = ERASE_BLOCK_SIZE;
    const BLOCK_COUNT: usize = LITTLEFS_BLOCK_COUNT;
    const BLOCK_CYCLES: isize = 100;

    fn read(&mut self, off: usize, buf: &mut [u8]) -> LfsResult<usize> {
        if off % PAGE_SIZE != 0 || buf.len() % PAGE_SIZE != 0 {
            return Err(LfsError::IO);
        }
        let capacity = Self::BLOCK_COUNT * ERASE_BLOCK_SIZE;
        if off.checked_add(buf.len()).filter(|&end| end <= capacity).is_none() {
            return Err(LfsError::IO);
        }

        for (index, page) in buf.chunks_exact_mut(PAGE_SIZE).enumerate() {
            let sector = (off / PAGE_SIZE + index) as u32;
            let mut error = 0i32;
            if unsafe { xcore_dhara_map_read(sector, page.as_mut_ptr(), &mut error) } != 0 {
                return Err(LfsError::IO);
            }
        }
        Ok(buf.len())
    }

    fn write(&mut self, off: usize, data: &[u8]) -> LfsResult<usize> {
        if off % PAGE_SIZE != 0 || data.is_empty() || data.len() % PAGE_SIZE != 0 {
            return Err(LfsError::IO);
        }
        let capacity = Self::BLOCK_COUNT * ERASE_BLOCK_SIZE;
        if off.checked_add(data.len()).filter(|&end| end <= capacity).is_none() {
            return Err(LfsError::IO);
        }

        for (index, page) in data.chunks_exact(PAGE_SIZE).enumerate() {
            let sector = (off / PAGE_SIZE + index) as u32;
            let mut error = 0i32;
            if unsafe { xcore_dhara_map_write(sector, page.as_ptr(), &mut error) } != 0 {
                return Err(LfsError::IO);
            }
        }

        // LittleFS has no separate sync callback in this Storage trait.
        // Commit each completed write call before reporting success.
        let mut error = 0i32;
        if unsafe { xcore_dhara_map_sync(&mut error) } != 0 {
            return Err(LfsError::IO);
        }
        Ok(data.len())
    }

    fn erase(&mut self, off: usize, len: usize) -> LfsResult<usize> {
        if off % ERASE_BLOCK_SIZE != 0 || len == 0 || len % ERASE_BLOCK_SIZE != 0 {
            return Err(LfsError::IO);
        }
        let capacity = Self::BLOCK_COUNT * ERASE_BLOCK_SIZE;
        if off.checked_add(len).filter(|&end| end <= capacity).is_none() {
            return Err(LfsError::IO);
        }

        let first_sector = off / PAGE_SIZE;
        let sector_count = len / PAGE_SIZE;
        for sector in first_sector..(first_sector + sector_count) {
            let mut error = 0i32;
            if unsafe { xcore_dhara_map_trim(sector as u32, &mut error) } != 0 {
                return Err(LfsError::IO);
            }
        }
        let mut error = 0i32;
        if unsafe { xcore_dhara_map_sync(&mut error) } != 0 {
            return Err(LfsError::IO);
        }
        Ok(len)
    }
}
