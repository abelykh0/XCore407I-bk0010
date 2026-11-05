//! Minimal C string routines required by littlefs when linked without libc.
//! These functions follow the C standard contracts: callers must pass valid
//! NUL-terminated strings and non-null pointers.

use core::ffi::{c_char, c_int};

/// Return the length of the initial segment of `s` containing only bytes in `accept`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn strspn(s: *const c_char, accept: *const c_char) -> usize {
    let mut length = 0usize;
    loop {
        let byte = unsafe { *s.add(length) } as u8;
        if byte == 0 {
            return length;
        }

        let mut found = false;
        let mut index = 0usize;
        loop {
            let candidate = unsafe { *accept.add(index) } as u8;
            if candidate == 0 {
                break;
            }
            if candidate == byte {
                found = true;
                break;
            }
            index += 1;
        }

        if !found {
            return length;
        }
        length += 1;
    }
}

/// Return the length of the initial segment of `s` containing no bytes in `reject`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn strcspn(s: *const c_char, reject: *const c_char) -> usize {
    let mut length = 0usize;
    loop {
        let byte = unsafe { *s.add(length) } as u8;
        if byte == 0 {
            return length;
        }

        let mut index = 0usize;
        loop {
            let candidate = unsafe { *reject.add(index) } as u8;
            if candidate == 0 {
                break;
            }
            if candidate == byte {
                return length;
            }
            index += 1;
        }
        length += 1;
    }
}

/// Locate the first occurrence of `character` in `s`, including its terminator.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn strchr(s: *const c_char, character: c_int) -> *mut c_char {
    let needle = character as u8;
    let mut index = 0usize;
    loop {
        let byte = unsafe { *s.add(index) } as u8;
        if byte == needle {
            return unsafe { s.add(index) as *mut c_char };
        }
        if byte == 0 {
            return core::ptr::null_mut();
        }
        index += 1;
    }
}
